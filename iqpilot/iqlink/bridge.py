#!/usr/bin/env python3
"""iqlink bridge: CP搭子/Carrot 即时参数 → iqNavState (BLE-only).

Started as process iqlinkd from system/manager/process_config.py.
WiFi discovery/UDP/HTTP (7705/7706/7713) removed — transport is BLE GATT only.

Product: no on-car nav session. Apply only when carrot data changes; sticky snapshot until next change
(R1). No-write timeouts warn only — do not clear exec envelope. Disable Iqlink clears leftovers.
"""

from __future__ import annotations

import json
import subprocess
import threading
import time
from typing import Any

from cereal import custom, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.iqpilot.iqlink import (
  DEFAULT_CANCEL_TIMEOUT_S,
  DEFAULT_WARN_TIMEOUT_S,
)
from openpilot.iqpilot.iqlink import protocol as proto
from openpilot.iqpilot.iqlink.road_limit_hold import IqlinkRoadLimitHold

NavState = custom.IQNavState
NavDir = custom.NavDirection
TurnDir = custom.IQTurnSignalDirection

BLE_ADAPTER_WAIT_S = 30.0


def _payload_fingerprint(payload: dict[str, Any]) -> str:
  """Stable compare for C1-b: ignore apply when carrot data unchanged."""
  return json.dumps(payload, ensure_ascii=True, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _bluez_adapter_powered() -> bool:
  try:
    r = subprocess.run(
      ["bluetoothctl", "show"],
      capture_output=True, text=True, timeout=5, check=False,
    )
    return r.returncode == 0 and "Powered: yes" in r.stdout
  except Exception:
    return False


def _wait_bluez_powered(timeout_s: float = BLE_ADAPTER_WAIT_S) -> None:
  """Wait for BlueZ adapter Powered only (IQ-link iqlink; do not gate on settings BLE)."""
  deadline = time.monotonic() + timeout_s
  logged = False
  while time.monotonic() < deadline:
    if _bluez_adapter_powered():
      cloudlog.info("iqlink: BlueZ adapter Powered — BLE GATT starting")
      return
    if not logged:
      cloudlog.info("iqlink: waiting for BlueZ adapter Powered")
      logged = True
    time.sleep(0.5)
  cloudlog.warning(
    f"iqlink: BlueZ Powered wait timed out after {timeout_s:.0f}s — proceeding"
  )


def _enum_dir(name: str):
  if name == "left":
    return NavDir.left, TurnDir.turnLeft
  if name == "right":
    return NavDir.right, TurnDir.turnRight
  return NavDir.none, TurnDir.none


def _enum_maneuver(name: str):
  return getattr(NavState.ManeuverType, name, NavState.ManeuverType.none)


def _enum_cam(name: str):
  return getattr(NavState.CameraType, name, NavState.CameraType.none)


def _enum_provider(name: str):
  return getattr(NavState.LongitudinalProvider, name, NavState.LongitudinalProvider.none)


def _enum_long_state(name: str):
  return getattr(NavState.LongitudinalState, name, NavState.LongitudinalState.disabled)


def _enum_phase(name: str):
  return getattr(NavState.ManeuverPhase, name, NavState.ManeuverPhase.none)


def _enum_command(name: str):
  return getattr(NavState.Command, name, NavState.Command.none)


_GPS_SERVICES = ("gpsLocationExternal", "gpsLocation")
_RENDER_ZOOM_HINT = 16.0


def _valid_render_coord(lat: float, lon: float) -> bool:
  return abs(lat) > 0.01 and abs(lon) > 0.01


def _ego_from_vp(raw: dict[str, Any] | None) -> tuple[float, float, bool]:
  if not raw:
    return 0.0, 0.0, False
  lat = float(raw.get("vpPosPointLat") or 0.0)
  lon = float(raw.get("vpPosPointLon") or 0.0)
  if abs(lat) > 90 and abs(lat) < 90000000:
    lat /= 1e6
  if abs(lon) > 180 and abs(lon) < 180000000:
    lon /= 1e6
  if _valid_render_coord(lat, lon):
    return lat, lon, True
  return 0.0, 0.0, False


def _position_from_gps_msg(msg) -> tuple[float, float, float, bool]:
  try:
    lat = float(getattr(msg, "latitude", 0.0))
    lon = float(getattr(msg, "longitude", 0.0))
    bearing = float(getattr(msg, "bearingDeg", 0.0))
    if _valid_render_coord(lat, lon):
      return lat, lon, bearing, True
  except Exception:
    pass
  return 0.0, 0.0, 0.0, False


def clear_stale_nav_params(params: Params, *, clear_exclusive: bool = False) -> None:
  """Drop leftover dest/active Params when iqlink is disabled (or keepalive with no snapshot).

  R1: no-write timeout does not call this. ponytail: keep IqlinkExclusive unless
  clear_exclusive (disable path; clearing on timeout historically flapped BLE).
  """
  try:
    params.remove("NavigationDestination")
  except Exception:
    pass
  params.put_bool("NavigationActive", False)
  params.put_bool("IqlinkLinkWarn", False)
  if clear_exclusive:
    params.put_bool("IqlinkExclusive", False)


# Legacy alias; R1: disable/keepalive cleanup only — not no-write timeout.
cancel_navigation = clear_stale_nav_params


class IqlinkBridge:
  def __init__(self):
    self.params = Params()
    self.pm = messaging.PubMaster(["iqNavState", "iqNavRenderState"])
    self.sm = messaging.SubMaster(["modelV2", *_GPS_SERVICES])
    self._lock = threading.Lock()
    self._latest: dict[str, Any] | None = None
    self._raw_payload: dict[str, Any] | None = None
    self._raw_fp: str | None = None
    self._last_rx = 0.0
    self._command_index = 0
    self._last_lc_cmd = False
    self._warn_logged = False
    self._road_limit_hold = IqlinkRoadLimitHold()

  def _timeouts(self) -> tuple[float, float]:
    try:
      warn_s = float(self.params.get("IqlinkWarnTimeoutS") or DEFAULT_WARN_TIMEOUT_S)
    except Exception:
      warn_s = DEFAULT_WARN_TIMEOUT_S
    try:
      cancel_s = float(self.params.get("IqlinkCancelTimeoutS") or DEFAULT_CANCEL_TIMEOUT_S)
    except Exception:
      cancel_s = DEFAULT_CANCEL_TIMEOUT_S
    return max(warn_s, 0.5), max(cancel_s, warn_s)

  def _ack_ble_rx(self) -> None:
    """Any live BLE write clears LinkWarn without dropping the R1 exec snapshot."""
    self._last_rx = time.monotonic()
    try:
      self.params.put_bool("IqlinkLinkWarn", False)
    except Exception:
      pass
    self._warn_logged = False

  def _vision_stop(self) -> bool:
    try:
      self.sm.update(0)
      if not self.sm.alive.get("modelV2", False):
        return False
      mv = self.sm["modelV2"]
      action = getattr(mv, "action", None)
      if action is not None and bool(getattr(action, "shouldStop", False)):
        return True
      pos = getattr(mv, "position", None)
      if pos is not None and len(getattr(pos, "x", [])) > 0 and float(pos.x[-1]) < 25.0:
        return True
    except Exception:
      return False
    return False

  def _ego_gps(self) -> tuple[float, float, float, bool]:
    try:
      self.sm.update(0)
      for svc in _GPS_SERVICES:
        if not self.sm.alive.get(svc, False):
          continue
        lat, lon, bearing, valid = _position_from_gps_msg(self.sm[svc])
        if valid:
          return lat, lon, bearing, True
    except Exception:
      pass
    return 0.0, 0.0, 0.0, False

  def ingest(self, payload: dict[str, Any]) -> None:
    if not isinstance(payload, dict):
      return
    # C1-b: same carrot data → refresh link heartbeat only, do not re-apply.
    fp = _payload_fingerprint(payload)
    with self._lock:
      same = fp == getattr(self, "_raw_fp", None) and self._raw_payload is not None
    if same:
      self._ack_ble_rx()
      return

    fields = proto.map_carrot_to_nav_fields(
      payload,
      aggressive_lc=True,
      command_index=self._command_index,
      vision_stop=self._vision_stop(),
    )
    if fields is None:
      return
    flat = proto.flatten_payload(payload)
    raw_kph = float(flat.get("nRoadLimitSpeed") or 0.0)
    if raw_kph > 0.0:
      held_kph = self._road_limit_hold.filter_kph(raw_kph, time.monotonic())
      if held_kph > 0.0:
        held_ms = held_kph / 3.6
        fields["roadSpeedLimit"] = held_ms
        fields["roadSpeedLimitValid"] = True
    # Phone BLE keepalive (nRoadLimitSpeed=0, no dest/maneuver): HMAC link only.
    # Do not arm nav params / NavigationActive from an empty envelope.
    if (
      not fields.get("longitudinalEngaged")
      and not fields.get("destinationValid")
      and not fields.get("nextManeuverValid")
    ):
      self._ack_ble_rx()
      # Keepalive never clears an existing exec snapshot (R1).
      if self._latest is None:
        try:
          if self.params.get_bool("IqlinkExclusive") or self.params.get_bool("NavigationActive"):
            clear_stale_nav_params(self.params)
        except Exception:
          pass
      return
    send_lc = bool(fields.get("shouldSendLaneChangeDesire"))
    if send_lc and not self._last_lc_cmd:
      self._command_index += 1
      fields["commandIndex"] = self._command_index
    self._last_lc_cmd = send_lc
    with self._lock:
      self._raw_payload = dict(payload)
      self._raw_fp = fp
      self._latest = fields
      self._last_rx = time.monotonic()
    # HUD resolver: debounced Gaode nRoadLimitSpeed (m/s). Avoid Params keys.h rebuild.
    try:
      with open("/dev/shm/iqlink_road_speed_ms", "w", encoding="utf-8") as f:
        f.write(f"{float(fields.get('roadSpeedLimit') or 0.0):.4f}")
    except Exception:
      pass
    self.params.put_bool("IqlinkExclusive", True)
    self.params.put_bool("NavigationActive", True)
    self._ack_ble_rx()
    # IQ-link on: no nav-exit auto LC / nudgeless ALC (product: driver or blinker ALC only).
    try:
      self.params.put_bool("NavExitLaneChange", False)
    except Exception:
      pass
    if fields.get("destinationValid"):
      # Remain-only valid often has no POI/coords — never invent a pin (esp. not ego GPS).
      # Also drop a stale pin left by an older session / prior ego-as-goal bug.
      lat = float(fields.get("destinationLatitude") or 0.0)
      lon = float(fields.get("destinationLongitude") or 0.0)
      name = str(fields.get("destinationName") or "").strip()
      has_pin = abs(lat) > 0.01 and abs(lon) > 0.01
      if has_pin:
        self.params.put("NavigationDestination", {
          "latitude": lat,
          "longitude": lon,
          "name": name,
        })
      else:
        try:
          self.params.remove("NavigationDestination")
        except Exception:
          pass

  def _publish_inactive(self) -> None:
    msg = messaging.new_message("iqNavState")
    msg.iqNavState.active = False
    msg.iqNavState.valid = False
    msg.iqNavState.longitudinalEngaged = False
    self.pm.send("iqNavState", msg)
    render = messaging.new_message("iqNavRenderState")
    render.iqNavRenderState.active = False
    self.pm.send("iqNavRenderState", render)

  def _fill_render_msg(
    self,
    fields: dict[str, Any],
    ego_lat: float,
    ego_lon: float,
    bearing: float,
    raw: dict[str, Any] | None = None,
  ):
    msg = messaging.new_message("iqNavRenderState")
    r = msg.iqNavRenderState
    r.active = bool(
      fields.get("active")
      or fields.get("destinationValid")
      or fields.get("nextManeuverValid")
    )
    ego_valid = _valid_render_coord(ego_lat, ego_lon)
    if not ego_valid:
      ego_lat, ego_lon, ego_valid = _ego_from_vp(raw)
      if ego_valid:
        bearing = 0.0
    r.currentLatitude = float(ego_lat) if ego_valid else 0.0
    r.currentLongitude = float(ego_lon) if ego_valid else 0.0
    r.bearingDeg = float(bearing) if ego_valid else 0.0

    dest_lat = float(fields.get("destinationLatitude") or 0.0)
    dest_lon = float(fields.get("destinationLongitude") or 0.0)
    r.destinationLatitude = dest_lat
    r.destinationLongitude = dest_lon
    dest_valid = _valid_render_coord(dest_lat, dest_lon)

    if ego_valid and dest_valid:
      r.init("routePolyline", 2)
      r.init("routePolylineSimplified", 2)
      for idx, (lat, lon) in enumerate(((ego_lat, ego_lon), (dest_lat, dest_lon))):
        r.routePolyline[idx].latitude = lat
        r.routePolyline[idx].longitude = lon
        r.routePolylineSimplified[idx].latitude = lat
        r.routePolylineSimplified[idx].longitude = lon
    else:
      r.init("routePolyline", 0)
      r.init("routePolylineSimplified", 0)

    r.nextManeuverType = _enum_maneuver(str(fields.get("nextManeuverType") or "none"))
    nav_d, _ = _enum_dir(str(fields.get("nextManeuverDirection") or "none"))
    r.nextManeuverDirection = nav_d
    r.nextManeuverDistance = float(fields.get("nextManeuverDistance") or 0.0)
    r.zoomHint = _RENDER_ZOOM_HINT
    return msg

  def _fill_msg(self, fields: dict[str, Any]):
    msg = messaging.new_message("iqNavState")
    n = msg.iqNavState
    n.active = bool(fields.get("active"))
    n.destinationValid = bool(fields.get("destinationValid"))
    n.distanceRemaining = float(fields.get("distanceRemaining") or 0.0)
    n.timeRemaining = float(fields.get("timeRemaining") or 0.0)
    n.nextManeuverValid = bool(fields.get("nextManeuverValid"))
    n.nextManeuverDistance = float(fields.get("nextManeuverDistance") or 0.0)
    n.nextManeuverType = _enum_maneuver(str(fields.get("nextManeuverType") or "none"))
    nav_d, turn_d = _enum_dir(str(fields.get("nextManeuverDirection") or "none"))
    n.nextManeuverDirection = turn_d
    n.nextManeuverDescription = str(fields.get("nextManeuverDescription") or "")
    n.secondNextManeuverValid = bool(fields.get("secondNextManeuverValid"))
    n.secondNextManeuverType = _enum_maneuver(str(fields.get("secondNextManeuverType") or "none"))
    n.secondNextManeuverDirection, _ = _enum_dir(str(fields.get("secondNextManeuverDirection") or "none"))
    n.secondNextManeuverDistance = float(fields.get("secondNextManeuverDistance") or 0.0)
    n.shouldSendTurnDesire = bool(fields.get("shouldSendTurnDesire"))
    _, n.turnDesireDirection = _enum_dir(str(fields.get("turnDesireDirection") or "none"))
    n.shouldSendLaneChangeDesire = bool(fields.get("shouldSendLaneChangeDesire"))
    _, n.laneChangeDesireDirection = _enum_dir(str(fields.get("laneChangeDesireDirection") or "none"))
    n.maneuverPhase = _enum_phase(str(fields.get("maneuverPhase") or "none"))
    n.maneuverDirection, _ = _enum_dir(str(fields.get("maneuverDirection") or "none"))
    n.command = _enum_command(str(fields.get("command") or "none"))
    n.commandDirection, _ = _enum_dir(str(fields.get("commandDirection") or "none"))
    n.commandIndex = int(fields.get("commandIndex") or 0)
    n.destinationLatitude = float(fields.get("destinationLatitude") or 0.0)
    n.destinationLongitude = float(fields.get("destinationLongitude") or 0.0)
    n.destinationName = str(fields.get("destinationName") or "")
    n.targetSpeed = float(fields.get("targetSpeed") or 0.0)
    n.targetSpeedValid = bool(fields.get("targetSpeedValid"))
    n.speedTarget = float(fields.get("speedTarget") or 0.0)
    n.accelTarget = float(fields.get("accelTarget") or 0.0)
    n.valid = bool(fields.get("valid"))
    n.longitudinalEngaged = bool(fields.get("longitudinalEngaged"))
    n.longitudinalProvider = _enum_provider(str(fields.get("longitudinalProvider") or "none"))
    n.longitudinalState = _enum_long_state(str(fields.get("longitudinalState") or "disabled"))
    n.navSpeedTargetActive = bool(fields.get("navSpeedTargetActive"))
    n.cameraValid = bool(fields.get("cameraValid"))
    n.cameraType = _enum_cam(str(fields.get("cameraType") or "none"))
    n.cameraDistance = float(fields.get("cameraDistance") or 0.0)
    n.cameraSpeedLimit = float(fields.get("cameraSpeedLimit") or 0.0)
    n.navTurnDesireDirection = nav_d if fields.get("shouldSendTurnDesire") else NavDir.none
    n.navLaneChangeDesireDirection, _ = _enum_dir(str(fields.get("navLaneChangeDesireDirection") or "none"))
    n.trafficLight = str(fields.get("trafficLight") or "none")
    n.trafficLightDistM = float(fields.get("trafficLightDistM") or 0.0)
    n.trafficLightRemainS = float(fields.get("trafficLightRemainS") or 0.0)
    n.roadSpeedLimit = float(fields.get("roadSpeedLimit") or 0.0)
    n.roadSpeedLimitValid = bool(fields.get("roadSpeedLimitValid"))
    n.leftTurnPending = bool(fields.get("leftTurnPending"))
    n.rightTurnPending = bool(fields.get("rightTurnPending"))
    n.laneRecommend = str(fields.get("laneRecommend") or "none")
    return msg

  def _maybe_timeout(self) -> None:
    """No-write soft warn only (R1 / C3-a). Never clear exec snapshot here."""
    warn_s, stale_s = self._timeouts()
    with self._lock:
      last = self._last_rx
      has = self._latest is not None
    if not has or last <= 0:
      return
    age = time.monotonic() - last
    if age >= warn_s and not self._warn_logged:
      self.params.put_bool("IqlinkLinkWarn", True)
      self._warn_logged = True
      cloudlog.warning("iqlink: no BLE write recently (exec snapshot kept)")
    # CancelTimeout used to clear envelope; R1 keeps sticky params until content changes
    # or IqlinkEnabled off. stale_s only escalates the same warn (no clear).
    _ = stale_s

  def publish_loop(self) -> None:
    rk = Ratekeeper(5)
    while True:
      if not self.params.get_bool("IqlinkEnabled"):
        with self._lock:
          if self._latest is not None or self._raw_payload is not None:
            self._latest = None
            self._raw_payload = None
            self._raw_fp = None
            self._last_rx = 0.0
        if self.params.get_bool("IqlinkExclusive") or self.params.get_bool("NavigationActive"):
          clear_stale_nav_params(self.params, clear_exclusive=True)
        self._publish_inactive()
        rk.keep_time()
        continue

      self._maybe_timeout()

      vision_stop = self._vision_stop()
      with self._lock:
        raw = dict(self._raw_payload) if self._raw_payload else None
        fields = dict(self._latest) if self._latest else None

      if raw is not None:
        remapped = proto.map_carrot_to_nav_fields(
          raw,
          aggressive_lc=True,
          command_index=self._command_index,
          vision_stop=vision_stop,
        )
        if remapped is not None and (
          remapped.get("longitudinalEngaged")
          or remapped.get("destinationValid")
          or remapped.get("nextManeuverValid")
        ):
          fields = remapped
          with self._lock:
            self._latest = remapped

      if fields is None:
        self._publish_inactive()
      else:
        ego_lat, ego_lon, bearing, _ = self._ego_gps()
        self.pm.send("iqNavState", self._fill_msg(fields))
        self.pm.send(
          "iqNavRenderState",
          self._fill_render_msg(fields, ego_lat, ego_lon, bearing, raw),
        )
      rk.keep_time()


def main() -> None:
  params = Params()
  bridge = IqlinkBridge()
  try:
    from openpilot.iqpilot.iqlink.ble_gatt import ensure_ble_psk, run_ble_gatt_loop
    from openpilot.iqpilot.iqlink.konn3kt_ble_exclusive import sync_konn3kt_ble_for_iqlink
    sync_konn3kt_ble_for_iqlink(params)
    if params.get_bool("IqlinkEnabled"):
      ensure_ble_psk(params)

    def _ble_runner() -> None:
      _wait_bluez_powered()
      run_ble_gatt_loop(bridge.ingest)

    threading.Thread(target=_ble_runner, daemon=True, name="iqlink_ble").start()
  except Exception as e:
    cloudlog.warning(f"iqlink ble thread not started: {e}")
  cloudlog.info("iqlink bridge started (BLE-only)")
  bridge.publish_loop()


if __name__ == "__main__":
  main()

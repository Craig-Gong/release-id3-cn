#!/usr/bin/env python3
"""IQ-link daemon: LAN UDP HMAC → /dev/shm/sp_nav.json.

Wi‑Fi UDP (:17710) only — AGNOS C3XL has no usable BLE HCI for phone nav.
Optional inject: /dev/shm/sp_nav_inject.json (must include nRoadLimitSpeed).
"""
from __future__ import annotations

import json
import os
import time

from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.nav.envelope import FIXED_BLE_PSK, EnvelopeVerifier
from openpilot.sunnypilot.nav.protocol import flatten_payload, parse_carrot
from openpilot.sunnypilot.nav.road_limit_hold import IqlinkRoadLimitHold
from openpilot.sunnypilot.nav.snapshot import (
  HMAC_FRESH_S,
  INJECT_SHM_PATH,
  NavSnapshot,
  write_snapshot,
)
from openpilot.sunnypilot.nav.udp_nav import UdpNavServer

LINK_OFF = 0
LINK_CONNECTING = 1
LINK_CONNECTED = 2
UDP_RETRY_S = 5.0
LIGHT_LOG_PATH = "/data/iqlink_light_log.csv"
LIGHT_LOG_MAX_BYTES = 2 * 1024 * 1024
_LIGHT_LOG_HEADER = "wall,mono,light,raw,dir,remain_s,dist_m,dist_ok,age_ms,stop,tbt_type,tbt_dist\n"


def _param_bool(params: Params, key: str, default: bool = False) -> bool:
  try:
    return bool(params.get_bool(key))
  except UnknownKeyName:
    return default
  except Exception:
    return default


def _param_put(params: Params, key: str, value) -> None:
  try:
    if isinstance(value, bool):
      params.put_bool(key, value)
    else:
      params.put(key, str(value))
  except Exception:
    pass


def _psk(params: Params) -> str:
  try:
    raw = params.get("IqlinkBlePsk")
    if raw is None:
      return FIXED_BLE_PSK
    if isinstance(raw, bytes):
      text = raw.decode("utf-8", errors="ignore")
    else:
      text = str(raw)
    text = text.strip()
    return text if len(text) == 6 and text.isdigit() else FIXED_BLE_PSK
  except Exception:
    return FIXED_BLE_PSK


class IqlinkDaemon:
  def __init__(self):
    self.params = Params()
    self.verifier = EnvelopeVerifier(_psk(self.params))
    self._last_snap = NavSnapshot()
    self._last_hmac_ts = 0.0
    self._was_hmac_fresh = False
    self._road_limit_hold = IqlinkRoadLimitHold()
    self._light_log_key: tuple | None = None
    self._inject_mtime = 0.0
    self.udp = UdpNavServer(self._on_udp_datagram)

  def _log_light(self, snap: NavSnapshot, data: dict, now: float) -> None:
    """One CSV row per light / countdown change, for matching against drives."""
    key = (snap.traffic_light, snap.light_raw, snap.light_dir, int(snap.remain_s), snap.stop_for_light)
    if key == self._light_log_key:
      return
    self._light_log_key = key
    try:
      if os.path.exists(LIGHT_LOG_PATH) and os.path.getsize(LIGHT_LOG_PATH) > LIGHT_LOG_MAX_BYTES:
        os.replace(LIGHT_LOG_PATH, LIGHT_LOG_PATH + ".1")
      new_file = not os.path.exists(LIGHT_LOG_PATH)
      age_ms = int(float(data.get("trafficLightAgeMs", -1) or -1))
      row = (f"{time.strftime('%Y-%m-%d %H:%M:%S')},{now:.2f},{snap.traffic_light},{snap.light_raw},"
             f"{snap.light_dir},{int(snap.remain_s)},{snap.dist_m:.0f},{int(snap.dist_ok)},{age_ms},"
             f"{int(snap.stop_for_light)},{int(float(data.get('nTBTTurnType', 0) or 0))},{snap.tbt_dist:.0f}\n")
      with open(LIGHT_LOG_PATH, "a", encoding="utf-8") as f:
        if new_file:
          f.write(_LIGHT_LOG_HEADER)
        f.write(row)
    except Exception:
      pass

  def _apply_road_limit_hold(self, snap: NavSnapshot, now: float) -> NavSnapshot:
    """Debounce Gaode nRoadLimitSpeed; keep red-light targets untouched."""
    raw = float(snap.road_limit_kph or 0.0)
    held = float(self._road_limit_hold.filter_kph(raw, now))
    if held <= 0.0 or abs(held - raw) < 0.05:
      if held > 0.0:
        snap.road_limit_kph = held
      return snap
    snap.road_limit_kph = held
    if not snap.stop_for_light and float(snap.speed_target or 0.0) > 0.0:
      snap.speed_target = held / 3.6
    return snap

  def _handle_envelope(self, blob: bytes) -> bool:
    """Returns True if the envelope was accepted (ok or replay heartbeat)."""
    status, data = self.verifier.inspect(blob)
    if status == "bad" or data is None:
      return False
    if status == "replay":
      self._heartbeat_only()
      return True
    self._ingest(data, hmac_ok=True)
    return True

  def _on_udp_datagram(self, raw: bytes, addr: tuple[str, int]) -> None:
    """One UDP datagram = one complete HMAC envelope."""
    if not raw:
      return
    if self._handle_envelope(raw):
      self.udp.reply(addr)

  def _ingest(self, data: dict, *, hmac_ok: bool) -> None:
    enabled = _param_bool(self.params, "IqlinkEnabled", True)
    now = time.monotonic()
    snap = parse_carrot(
      data, now=now, link_ok=hmac_ok, link_state=LINK_CONNECTED if hmac_ok else LINK_OFF,
      enabled=enabled,
    )
    if snap is None:
      if hmac_ok:
        self._last_hmac_ts = now
        self._last_snap.link_ok = True
        self._last_snap.link_state = LINK_CONNECTED
        self._last_snap.iqlink_enabled = enabled
        self._last_snap.ts = now
        write_snapshot(self._last_snap)
      return
    flat = flatten_payload(data)
    snap = self._apply_road_limit_hold(snap, now)
    if hmac_ok:
      self._last_hmac_ts = now
    self._last_snap = snap
    write_snapshot(snap)
    self._log_light(snap, flat, now)

  def _heartbeat_only(self) -> None:
    now = time.monotonic()
    self._last_hmac_ts = now
    self._last_snap.link_ok = True
    self._last_snap.link_state = LINK_CONNECTED
    self._last_snap.ts = now
    write_snapshot(self._last_snap)

  def _poll_inject(self) -> None:
    try:
      st = os.stat(INJECT_SHM_PATH)
    except OSError:
      return
    if st.st_mtime == self._inject_mtime:
      return
    self._inject_mtime = st.st_mtime
    try:
      with open(INJECT_SHM_PATH, encoding="utf-8") as f:
        obj = json.load(f)
    except (OSError, json.JSONDecodeError):
      return
    if isinstance(obj, dict):
      self._ingest(obj, hmac_ok=True)

  def _publish_link(self, enabled: bool) -> None:
    now = time.monotonic()
    hmac_fresh = bool(self._last_hmac_ts) and (now - self._last_hmac_ts) < HMAC_FRESH_S
    if self._was_hmac_fresh and not hmac_fresh:
      self.verifier = EnvelopeVerifier(_psk(self.params))
    self._was_hmac_fresh = hmac_fresh
    if not enabled and self._last_snap.iqlink_enabled:
      self._road_limit_hold.reset()
    transport_up = bool(self.udp.running)
    if hmac_fresh:
      state = LINK_CONNECTED
      ok = True
    else:
      state = LINK_CONNECTING if transport_up else LINK_OFF
      ok = False
    self._last_snap.iqlink_enabled = enabled
    self._last_snap.link_ok = ok
    self._last_snap.link_state = state
    if self._last_snap.ts <= 0.0:
      self._last_snap.ts = now
    write_snapshot(self._last_snap)
    # Legacy param names kept for UI; values are UDP link truth.
    _param_put(self.params, "IqlinkBleLinkState", state)
    _param_put(self.params, "IqlinkBleConnected", ok)

  def _ensure_udp(self, now: float, last_udp_try: float) -> float:
    if self.udp.running:
      return last_udp_try
    if (now - last_udp_try) < UDP_RETRY_S:
      return last_udp_try
    self.verifier = EnvelopeVerifier(_psk(self.params))
    udp_ok = self.udp.start()
    cloudlog.info(f"iqlinkd UDP start ok={udp_ok}")
    return now

  def run(self) -> None:
    rk = Ratekeeper(5)
    last_udp_try = 0.0
    # Never start BLE — AGNOS C3XL product path is UDP only.
    if _param_bool(self.params, "IqlinkBleEnabled", False):
      cloudlog.warning("iqlinkd: IqlinkBleEnabled ignored (UDP-only build)")
      _param_put(self.params, "IqlinkBleEnabled", False)
    while True:
      enabled = _param_bool(self.params, "IqlinkEnabled", True)
      now = time.monotonic()
      last_udp_try = self._ensure_udp(now, last_udp_try)
      self._poll_inject()
      self._publish_link(enabled)
      rk.keep_time()


def main() -> None:
  IqlinkDaemon().run()


if __name__ == "__main__":
  main()

#!/usr/bin/env python3
"""KL15 → EcoFlow 12V DC, plus parked GPU recover cycle.

C3XL stays always-on (harness). EcoFlow 12V is the eGPU rail: follow KL15 for
normal use, and allow an explicit OFF→ON cycle while parked so chestnut can
recover without rebooting the host. Never cycle while engaged / moving.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

# AGNOS venv is read-only; paho-mqtt lives in /data/python-packages on C3XL.
_DATA_PKGS = "/data/python-packages"
if os.path.isdir(_DATA_PKGS) and _DATA_PKGS not in sys.path:
  sys.path.append(_DATA_PKGS)

from openpilot.cereal import log
import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.system.ecoflow.kl15 import chestnut_superspeed_present, meb_ignition_from_can
from openpilot.sunnypilot.system.ecoflow.status import dc12v_from_telemetry, write_status
from openpilot.sunnypilot.system.ecoflow.recover import (
  GpuRecoverCycle, RecoverPhase, clear_recover_request, recover_allowed, recover_request_pending,
)

_VERIFY_ON_S = 2.0
_DC_OFF_DELAY_S = 60.0
_MQTT_RETRY_S = 5.0
_CRED_DIR = Path("/data/ecoflow_params")


def _enabled(params: Params) -> bool:
  try:
    return bool(params.get_bool("EcoflowEnabled"))
  except UnknownKeyName:
    return False
  except Exception:
    return False


def _param_bool(params: Params, key: str, default: bool = False) -> bool:
  try:
    return bool(params.get_bool(key))
  except UnknownKeyName:
    return default
  except Exception:
    return default


def _load_creds_into_env() -> None:
  mapping = {
    "phone": "ECOFLOW_PHONE",
    "password": "ECOFLOW_PASSWORD",
    "sn": "ECOFLOW_SN",
    "email": "ECOFLOW_EMAIL",
  }
  for fname, env in mapping.items():
    path = _CRED_DIR / fname
    try:
      val = path.read_text(encoding="utf-8").strip()
    except OSError:
      continue
    if val and env not in os.environ:
      os.environ[env] = val


def _network_up(network_type) -> bool:
  # capnp enums are not int(); comparing to the cereal none value is enough.
  try:
    return network_type != log.DeviceState.NetworkType.none
  except Exception:
    return False


class EcoflowDaemon:
  def __init__(self):
    self.params = Params()
    self.session = None
    self.last_on_ts = None
    self.kl15 = False
    self.saw_can = False
    self.off_deadline = None
    self.last_set_on = 0.0
    self.last_recover_off = 0.0
    self.last_mqtt_try = 0.0
    self.want_on = False
    self._cycle_blocked_logged = False
    self.recover = GpuRecoverCycle()
    self.started = False
    self.engaged = False
    self.v_ego = 0.0

  def _session(self):
    if self.session is not None:
      return self.session
    now = time.monotonic()
    if now - self.last_mqtt_try < _MQTT_RETRY_S:
      return None
    self.last_mqtt_try = now
    try:
      from openpilot.sunnypilot.system.ecoflow.client import EcoflowSession
      _load_creds_into_env()
      sess = EcoflowSession.from_env()
      sess.login()
      sess.connect_mqtt()
      self.session = sess
      cloudlog.info("ecoflowd MQTT up")
      return sess
    except Exception:
      cloudlog.exception("ecoflowd MQTT connect failed")
      self.session = None
      return None

  def _set_dc(self, on: bool, *, reason: str, allow_cut_while_superspeed: bool = False) -> bool:
    """Send DC command. Returns False if skipped (blocked) or MQTT unavailable."""
    if not on and chestnut_superspeed_present() and self.kl15 and not allow_cut_while_superspeed:
      # Healthy drive path: do not casually cut 12V under SuperSpeed+KL15.
      if not self._cycle_blocked_logged:
        cloudlog.warning("ecoflowd: skip DC off while chestnut SuperSpeed and KL15 (use Recover eGPU when parked)")
        self._cycle_blocked_logged = True
      return False
    if on:
      self._cycle_blocked_logged = False
    sess = self._session()
    if sess is None:
      return False
    try:
      sess.set_dc12v(on)
      cloudlog.info(f"ecoflowd DC {'on' if on else 'off'} ({reason})")
      return True
    except Exception:
      cloudlog.exception("ecoflowd set_dc12v failed")
      try:
        sess.disconnect()
      except Exception:
        pass
      self.session = None
      return False

  def _update_vehicle(self, sm) -> None:
    if sm.updated["deviceState"] or sm.recv_frame.get("deviceState", 0) > 0:
      try:
        self.started = bool(sm["deviceState"].started)
      except Exception:
        pass
    if sm.updated["selfdriveState"] or sm.recv_frame.get("selfdriveState", 0) > 0:
      try:
        self.engaged = bool(sm["selfdriveState"].enabled)
      except Exception:
        pass
    if sm.updated["carState"] or sm.recv_frame.get("carState", 0) > 0:
      try:
        self.v_ego = float(sm["carState"].vEgo)
      except Exception:
        pass

  def _begin_recover(self, now: float) -> None:
    if not recover_allowed(started=self.started, engaged=self.engaged, v_ego=self.v_ego):
      cloudlog.warning("ecoflowd: EcoflowGpuRecover ignored (engaged or moving)")
      clear_recover_request(self.params)
      return
    if not self._set_dc(False, reason="gpu recover off", allow_cut_while_superspeed=True):
      cloudlog.warning("ecoflowd: recover off not confirmed yet; will retry")
    else:
      self.last_recover_off = now
    self.recover.start(now)
    self.off_deadline = None
    cloudlog.info("ecoflowd: GPU recover cycle started (15s off → on)")

  def _tick_recover(self, now: float) -> bool:
    """Handle recover state machine. True if recover owns the DC policy this tick."""
    if not self.recover.active:
      return False

    if not recover_allowed(started=self.started, engaged=self.engaged, v_ego=self.v_ego):
      cloudlog.warning("ecoflowd: abort GPU recover (became unsafe)")
      self.recover.cancel()
      clear_recover_request(self.params)
      if self.kl15:
        self._set_dc(True, reason="recover abort → KL15")
      return True

    # While waiting in power_off, re-assert OFF at verify cadence (MQTT may lag).
    if self.recover.phase is RecoverPhase.power_off and now < self.recover.deadline:
      if now - self.last_recover_off >= _VERIFY_ON_S:
        if self._set_dc(False, reason="gpu recover off hold", allow_cut_while_superspeed=True):
          self.last_recover_off = now
      return True

    action = self.recover.tick(now)
    if action == "on":
      self._set_dc(True, reason="gpu recover on")
      return True
    if action == "done":
      clear_recover_request(self.params)
      self.last_set_on = now
      cloudlog.info("ecoflowd: GPU recover cycle done")
      return True
    return True

  def _publish_status(self) -> None:
    tel = {}
    mqtt = False
    if self.session is not None:
      mqtt = True
      tel = getattr(self.session, "_telemetry", None) or {}
    try:
      write_status(
        enabled=_enabled(self.params),
        mqtt=mqtt,
        dc12v=dc12v_from_telemetry(tel),
        kl15=self.kl15,
        want_on=self.want_on,
      )
    except Exception:
      cloudlog.exception("ecoflowd status shm write failed")

  def run(self) -> None:
    cloudlog.info("ecoflowd start")
    # vehicle sockets stay on SubMaster; CAN must be drain_sock — conflated
    # SubMaster["can"] often misses 0x3C0 for seconds and falsely drops KL15.
    sm = messaging.SubMaster(["deviceState", "selfdriveState", "carState"])
    can_sock = messaging.sub_sock("can", timeout=100)
    rk = Ratekeeper(10)
    while True:
      sm.update(0)
      now = time.monotonic()
      self._update_vehicle(sm)

      if not _enabled(self.params):
        if self.recover.active:
          cloudlog.warning("ecoflowd: EcoFlow disabled during GPU recover")
          self.recover.cancel()
          clear_recover_request(self.params)
          if self.kl15:
            # Leave the rail on if ignition is up — do not strand chestnut at 0 V.
            self._set_dc(True, reason="ecoflow disabled during recover")
        self._publish_status()
        rk.keep_time()
        continue

      try:
        packets = messaging.drain_sock(can_sock, wait_for_one=False)
      except Exception:
        packets = []
      self.kl15, self.last_on_ts, saw = meb_ignition_from_can(packets, now, self.last_on_ts)
      self.saw_can = self.saw_can or saw

      try:
        net = sm["deviceState"].networkType
      except Exception:
        net = log.DeviceState.NetworkType.none

      if _network_up(net):
        self._session()

      # Start recover when requested (param and/or /data/ecoflow_gpu_recover).
      if recover_request_pending(lambda k: _param_bool(self.params, k)) and not self.recover.active:
        self._begin_recover(now)

      if self._tick_recover(now):
        self._publish_status()
        rk.keep_time()
        continue

      if not self.saw_can:
        self._publish_status()
        rk.keep_time()
        continue

      if self.kl15:
        self.off_deadline = None
        self.want_on = True
        if now - self.last_set_on >= _VERIFY_ON_S:
          self._set_dc(True, reason="KL15")
          self.last_set_on = now
      else:
        self.want_on = False
        if self.off_deadline is None:
          self.off_deadline = now + _DC_OFF_DELAY_S
        elif now >= self.off_deadline:
          # Ignition already down: delayed off is allowed even if dock was up.
          self._cycle_blocked_logged = False
          self._set_dc(False, reason="KL15 delayed off", allow_cut_while_superspeed=True)
          self.off_deadline = now + 3600.0
      self._publish_status()
      rk.keep_time()


def main() -> None:
  EcoflowDaemon().run()


if __name__ == "__main__":
  main()

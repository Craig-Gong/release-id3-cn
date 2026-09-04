#!/usr/bin/env python3
"""KL15 → EcoFlow 12V DC. Never pulse/cycle 12V after chestnut SuperSpeed."""
from __future__ import annotations

import os
import time
from pathlib import Path

from cereal import log
from cereal import messaging
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.system.ecoflow.kl15 import chestnut_superspeed_present, meb_ignition_from_can

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


def _network_up(network_type: int) -> bool:
  return network_type != log.DeviceState.NetworkType.none


class EcoflowDaemon:
  def __init__(self):
    self.params = Params()
    self.session = None
    self.last_on_ts = None
    self.kl15 = False
    self.saw_can = False
    self.off_deadline = None
    self.last_set_on = 0.0
    self.last_mqtt_try = 0.0
    self.want_on = False
    self._cycle_blocked_logged = False

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

  def _set_dc(self, on: bool, *, reason: str) -> None:
    if not on and chestnut_superspeed_present() and self.kl15:
      # READY + SuperSpeed: never pulse 12V as a recover trick.
      if not self._cycle_blocked_logged:
        cloudlog.warning("ecoflowd: skip DC off/cycle while chestnut SuperSpeed and KL15")
        self._cycle_blocked_logged = True
      return
    if on:
      self._cycle_blocked_logged = False
    sess = self._session()
    if sess is None:
      return
    try:
      sess.set_dc12v(on)
      cloudlog.info(f"ecoflowd DC {'on' if on else 'off'} ({reason})")
    except Exception:
      cloudlog.exception("ecoflowd set_dc12v failed")
      try:
        sess.disconnect()
      except Exception:
        pass
      self.session = None

  def run(self) -> None:
    sm = messaging.SubMaster(["can", "deviceState"])
    rk = Ratekeeper(2)
    while True:
      sm.update(0)
      now = time.monotonic()
      if not _enabled(self.params):
        rk.keep_time()
        continue

      if sm.updated["can"]:
        packets = [sm["can"]]
        self.kl15, self.last_on_ts, saw = meb_ignition_from_can(packets, now, self.last_on_ts)
        self.saw_can = self.saw_can or saw

      net = 0
      try:
        net = int(sm["deviceState"].networkType)
      except Exception:
        pass

      if _network_up(net):
        self._session()

      if not self.saw_can:
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
          if chestnut_superspeed_present() and self.kl15:
            pass
          else:
            self._set_dc(False, reason="KL15 delayed off")
          self.off_deadline = now + 3600.0
      rk.keep_time()


def main() -> None:
  EcoflowDaemon().run()


if __name__ == "__main__":
  main()

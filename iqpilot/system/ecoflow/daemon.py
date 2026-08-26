#!/usr/bin/env python3
"""ecoflowd: MEB KL15 edges → EcoFlow Delta 3 12V DC (cloud MQTT).

Watch Klemmen_Status_01 (0x3C0) ZAS_Kl_15 — same bit as hardwared MebIgnitionWatch.
Rising edge → DC on; falling edge → DC off.

Gated by Params EcoflowEnabled (default off). Credentials: EcoflowPhone /
EcoflowPassword / EcoflowSn (or ECOFLOW_* env). Do not commit secrets.
"""
from __future__ import annotations

import time

from cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.iqpilot.system.ecoflow.client import EcoflowError, EcoflowSession

# Match hardwared.py (do not import hardwared — heavy HW side effects).
MEB_KLEMMEN_ADDR = 0x3C0
MEB_IGNITION_HOLD_S = 2.0

# Retry SET a few times on flaky LTE/hotspot.
_SET_RETRIES = 3
_SET_RETRY_SLEEP_S = 2.0
_MQTT_RECONNECT_S = 30.0


def meb_ignition_from_can(packets, now: float, last_on_ts: float | None) -> tuple[bool, float | None]:
  """ZAS_Kl_15 on 0x3C0 byte2 bit1. Do not OR ZAS_Kl_S."""
  for msg in packets:
    try:
      cans = msg.can
    except Exception:
      continue
    for c in cans:
      if getattr(c, "src", 0) >= 128:
        continue
      if c.address != MEB_KLEMMEN_ADDR:
        continue
      dat = bytes(c.dat)
      if len(dat) >= 3 and (dat[2] & 0x02):
        last_on_ts = now
  if last_on_ts is not None and (now - last_on_ts) < MEB_IGNITION_HOLD_S:
    return True, last_on_ts
  return False, last_on_ts


def _ecoflow_enabled(params: Params) -> bool:
  try:
    raw = params.get("EcoflowEnabled")
  except Exception:
    return False
  if raw is None:
    return False
  return params.get_bool("EcoflowEnabled")


class EcoflowDaemon:
  def __init__(self):
    self.params = Params()
    self.session: EcoflowSession | None = None
    self._want_dc: bool | None = None
    self._applied_dc: bool | None = None
    self._last_mqtt_ok = 0.0
    self._last_error_log = 0.0

  def _ensure_session(self) -> EcoflowSession | None:
    now = time.monotonic()
    if self.session is not None and self.session._client is not None:
      return self.session
    if now - self._last_mqtt_ok < 1.0 and self.session is not None:
      # Still connecting / recently failed — backoff handled by caller.
      pass
    try:
      session = EcoflowSession.from_params(self.params)
      session.login()
      session.connect_mqtt()
      self.session = session
      self._last_mqtt_ok = now
      cloudlog.info(f"ecoflowd: MQTT connected sn={session.sn} host={session.mqtt_url}")
      return session
    except EcoflowError as e:
      self._log_err_throttled(f"ecoflowd: connect failed: {e}")
      self._teardown_session()
      return None
    except Exception as e:
      self._log_err_throttled(f"ecoflowd: connect exception: {e}")
      self._teardown_session()
      return None

  def _teardown_session(self) -> None:
    if self.session is not None:
      try:
        self.session.disconnect()
      except Exception:
        pass
    self.session = None

  def _log_err_throttled(self, msg: str) -> None:
    now = time.monotonic()
    if now - self._last_error_log > 15.0:
      cloudlog.error(msg)
      self._last_error_log = now

  def _apply_dc(self, on: bool) -> bool:
    session = self._ensure_session()
    if session is None:
      return False
    for attempt in range(1, _SET_RETRIES + 1):
      try:
        ack = session.set_dc12v(on, wait_s=12.0)
        if ack is None:
          cloudlog.warning(f"ecoflowd: DC {'on' if on else 'off'} TIMEOUT attempt={attempt}")
        else:
          cloudlog.info(f"ecoflowd: DC {'on' if on else 'off'} ack={ack}")
          self._applied_dc = on
          self._last_mqtt_ok = time.monotonic()
          return True
      except Exception as e:
        cloudlog.warning(f"ecoflowd: DC set failed attempt={attempt}: {e}")
        self._teardown_session()
        session = self._ensure_session()
        if session is None:
          break
      time.sleep(_SET_RETRY_SLEEP_S)
    return False

  def tick_ignition(self, ignition: bool) -> None:
    if self._want_dc is None:
      # First sample: sync desired state without treating as an edge flood.
      self._want_dc = ignition
      cloudlog.info(f"ecoflowd: initial KL15={'on' if ignition else 'off'} → DC {'on' if ignition else 'off'}")
      self._apply_dc(ignition)
      return

    if ignition == self._want_dc:
      # Re-assert if we never got a successful apply (network lag after READY).
      if self._applied_dc != self._want_dc:
        if time.monotonic() - self._last_mqtt_ok > _MQTT_RECONNECT_S or self.session is None:
          self._apply_dc(self._want_dc)
      return

    self._want_dc = ignition
    edge = "rising" if ignition else "falling"
    cloudlog.info(f"ecoflowd: KL15 {edge} → DC {'on' if ignition else 'off'}")
    self._apply_dc(ignition)

  def close(self) -> None:
    self._teardown_session()


def main() -> None:
  cloudlog.info("ecoflowd: starting")
  daemon = EcoflowDaemon()
  sock = messaging.sub_sock("can", timeout=100)
  last_on_ts: float | None = None
  ignition = False
  rk = Ratekeeper(5)  # 5 Hz is enough; hold window is 2 s

  try:
    while True:
      if not _ecoflow_enabled(daemon.params):
        # Manager will stop us shortly; drop MQTT and idle.
        if daemon.session is not None:
          cloudlog.info("ecoflowd: EcoflowEnabled off — disconnect")
          daemon.close()
        time.sleep(1.0)
        continue

      try:
        packets = messaging.drain_sock(sock, wait_for_one=False)
      except Exception:
        packets = []
      now = time.monotonic()
      ignition, last_on_ts = meb_ignition_from_can(packets, now, last_on_ts)
      daemon.tick_ignition(ignition)
      rk.keep_time()
  finally:
    daemon.close()
    cloudlog.info("ecoflowd: stopped")


if __name__ == "__main__":
  main()

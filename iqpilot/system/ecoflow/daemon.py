#!/usr/bin/env python3
"""ecoflowd: MEB KL15 edges → EcoFlow Delta 3 12V DC (cloud MQTT).

Watch Klemmen_Status_01 (0x3C0) ZAS_Kl_15 — same bit as hardwared MebIgnitionWatch.
Rising edge → DC on immediately; falling edge → DC off after a short delay so brief
glitches / quick re-READY can cancel. Default delay 60 s (Delta 3 USB Wi‑Fi 24 h);
was 5 s when the car MIB hotspot died on lock.

While KL15 is on (READY), closed-loop verify: read MQTT telemetry and resend DC on
until cfg_dc12v_out_open / flow_info_12v confirms the output is on. This covers
overnight cold start when SET ack lies or WiFi comes up after the first edge.

MQTT is pre-connected as soon as EcoflowEnabled and the network is up. Do not
initial-sync KL15 until at least one 0x3C0 frame is seen.

Gated by Params EcoflowEnabled (default off). Do not commit secrets.
"""
from __future__ import annotations

import time

from iqpilot.cereal import log
from iqpilot.cereal import messaging
from iqpilot.common.params import Params
from iqpilot.common.realtime import Ratekeeper
from iqpilot.common.swaglog import cloudlog
from iqpilot.system.ecoflow.client import EcoflowError, EcoflowSession
from iqpilot.system.ecoflow.enabled import heal_enabled, is_enabled as _ecoflow_enabled

MEB_KLEMMEN_ADDR = 0x3C0
MEB_IGNITION_HOLD_S = 2.0

_SET_RETRIES = 3
_SET_RETRY_SLEEP_S = 1.0
_MQTT_RETRY_S = 5.0
_MQTT_RETRY_URGENT_S = 0.5
# READY closed-loop: re-check telemetry / resend ON at this cadence.
_VERIFY_ON_S = 2.0
_APPLY_OFF_RETRY_S = 1.0
# After KL15 falls, keep 12V this long (cancelled if KL15 rises).
# Delta 3 USB Wi‑Fi is up 24 h (not the MIB hotspot that died on lock).
_DC_OFF_DELAY_S = 60.0
_PENDING_ON_WARN_S = 15.0


def meb_ignition_from_can(
  packets, now: float, last_on_ts: float | None
) -> tuple[bool, float | None, bool]:
  saw_klemmen = False
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
      saw_klemmen = True
      dat = bytes(c.dat)
      if len(dat) >= 3 and (dat[2] & 0x02):
        last_on_ts = now
  if last_on_ts is not None and (now - last_on_ts) < MEB_IGNITION_HOLD_S:
    return True, last_on_ts, saw_klemmen
  return False, last_on_ts, saw_klemmen


def _network_up(network_type: int) -> bool:
  return network_type != log.DeviceState.NetworkType.none


class EcoflowDaemon:
  def __init__(self):
    self.params = Params()
    heal_enabled(self.params)
    self.session: EcoflowSession | None = None
    self._want_dc: bool | None = None
    self._applied_dc: bool | None = None
    self._klemmen_seen = False
    self._last_connect_attempt = 0.0
    self._last_apply_attempt = 0.0
    self._last_verify_attempt = 0.0
    self._last_error_log = 0.0
    self._last_pending_on_log = 0.0
    self._want_on_since: float | None = None
    self._mqtt_ready_logged = False
    self._was_mqtt_ready = False
    self._was_network_up = False
    self._off_deadline: float | None = None

  def mqtt_ready(self) -> bool:
    return self.session is not None and self.session._client is not None

  def wants_dc_on(self) -> bool:
    return self._want_dc is True

  def _telemetry_dc_on(self) -> bool | None:
    if not self.mqtt_ready() or self.session is None:
      return None
    return self.session.dc12v_is_on()

  def _dc_on_confirmed(self) -> bool:
    """True when MQTT telemetry says 12V DC is on."""
    state = self._telemetry_dc_on()
    if state is True:
      self._applied_dc = True
      self._want_on_since = None
      return True
    if state is False:
      self._applied_dc = False
    return False

  def _needs_apply(self) -> bool:
    if self._want_dc is None:
      return False
    if self._want_dc is True:
      return not self._dc_on_confirmed()
    return self._applied_dc is not False

  def _mqtt_retry_interval(self) -> float:
    if self.wants_dc_on() and self._needs_apply():
      return _MQTT_RETRY_URGENT_S
    return _MQTT_RETRY_S

  def _ensure_session(self, force: bool = False) -> EcoflowSession | None:
    now = time.monotonic()
    if self.mqtt_ready():
      return self.session
    if not force and (now - self._last_connect_attempt) < self._mqtt_retry_interval():
      return None
    self._last_connect_attempt = now
    try:
      session = EcoflowSession.from_params(self.params)
      session.login()
      session.connect_mqtt()
      self.session = session
      cloudlog.info(f"ecoflowd: MQTT connected sn={session.sn} host={session.mqtt_url}")
      self._mqtt_ready_logged = True
      return session
    except EcoflowError as e:
      self._log_err_throttled(f"ecoflowd: connect failed (wait for WiFi/network): {e}")
      self._teardown_session()
      return None
    except Exception as e:
      self._log_err_throttled(f"ecoflowd: connect exception: {e}")
      self._teardown_session()
      return None

  def maintain_mqtt(self, force: bool = False) -> None:
    if self.mqtt_ready():
      return
    if self._mqtt_ready_logged:
      cloudlog.warning("ecoflowd: MQTT session lost — reconnecting")
      self._mqtt_ready_logged = False
    self._ensure_session(force=force)

  def _teardown_session(self) -> None:
    if self.session is not None:
      try:
        self.session.disconnect()
      except Exception:
        pass
    self.session = None
    self._was_mqtt_ready = False

  def _log_err_throttled(self, msg: str) -> None:
    now = time.monotonic()
    if now - self._last_error_log > 15.0:
      cloudlog.error(msg)
      self._last_error_log = now

  def _cancel_off_delay(self, reason: str) -> None:
    if self._off_deadline is not None:
      cloudlog.info(f"ecoflowd: cancel delayed DC off ({reason})")
      self._off_deadline = None

  def _track_want_on(self) -> None:
    if self.wants_dc_on() and not self._dc_on_confirmed():
      if self._want_on_since is None:
        self._want_on_since = time.monotonic()
      now = time.monotonic()
      if (now - self._want_on_since) >= _PENDING_ON_WARN_S:
        if (now - self._last_pending_on_log) >= _PENDING_ON_WARN_S:
          cloudlog.warning(
            f"ecoflowd: READY still waiting for 12V confirm "
            f"({now - self._want_on_since:.0f}s, telemetry={self._telemetry_dc_on()}, "
            f"mqtt_ready={self.mqtt_ready()})"
          )
          self._last_pending_on_log = now
    else:
      self._want_on_since = None

  def _apply_dc(self, on: bool) -> bool:
    self._last_apply_attempt = time.monotonic()
    session = self._ensure_session(force=not self.mqtt_ready())
    if session is None:
      cloudlog.warning(
        f"ecoflowd: DC {'on' if on else 'off'} deferred — MQTT not ready yet"
      )
      return False

    for attempt in range(1, _SET_RETRIES + 1):
      try:
        ack = session.set_dc12v(on, wait_s=8.0)
        if ack is None:
          cloudlog.warning(f"ecoflowd: DC {'on' if on else 'off'} TIMEOUT attempt={attempt}")
          self._teardown_session()
          self._mqtt_ready_logged = False
          session = self._ensure_session(force=True)
          if session is None:
            break
          continue

        cloudlog.info(f"ecoflowd: DC {'on' if on else 'off'} ack={ack}")
        if on:
          if session.dc12v_is_on() is True:
            self._applied_dc = True
            self._off_deadline = None
            self._want_on_since = None
            return True
          # SET returned but telemetry not yet on — closed-loop will retry.
          cloudlog.info("ecoflowd: SET ack but telemetry not confirmed ON yet")
          return False

        if session.dc12v_is_off() is True or session.dc12v_is_on() is False:
          self._applied_dc = False
          return True
        self._applied_dc = False
        return True
      except Exception as e:
        cloudlog.warning(f"ecoflowd: DC set failed attempt={attempt}: {e}")
        self._teardown_session()
        self._mqtt_ready_logged = False
        session = self._ensure_session(force=True)
        if session is None:
          break
      time.sleep(_SET_RETRY_SLEEP_S)
    return False

  def tick_pending_off(self) -> None:
    if self._off_deadline is None:
      return
    now = time.monotonic()
    if now < self._off_deadline:
      return
    self._off_deadline = None
    cloudlog.info(
      f"ecoflowd: delayed DC off after {_DC_OFF_DELAY_S:.0f}s"
      f" (mqtt_ready={self.mqtt_ready()})"
    )
    self._want_dc = False
    self._apply_dc(False)

  def tick_network(self, network_type: int) -> None:
    up = _network_up(network_type)
    became_up = up and not self._was_network_up
    self._was_network_up = up
    if not became_up:
      return
    cloudlog.info(f"ecoflowd: network up (type={network_type}) — connect MQTT")
    self.maintain_mqtt(force=True)

  def tick_mqtt_ready(self) -> None:
    ready = self.mqtt_ready()
    became_ready = ready and not self._was_mqtt_ready
    self._was_mqtt_ready = ready
    if became_ready and self.wants_dc_on():
      cloudlog.info("ecoflowd: MQTT ready during READY — verify 12V")

  def tick_ensure_dc_on(self) -> None:
    """READY closed-loop: telemetry says off/unknown → resend until confirmed on."""
    if not self.wants_dc_on():
      return

    if self._dc_on_confirmed():
      return

    now = time.monotonic()
    if (now - self._last_verify_attempt) < _VERIFY_ON_S:
      return
    self._last_verify_attempt = now

    if not self.mqtt_ready():
      self.maintain_mqtt(force=True)
      return

    if (now - self._last_apply_attempt) < _VERIFY_ON_S:
      return

    tel = self._telemetry_dc_on()
    cloudlog.info(f"ecoflowd: READY verify — telemetry={tel}, resend DC on")
    self._apply_dc(True)

  def tick_ensure_dc_off(self) -> None:
    """After delayed off, retry SET off if we still think output is on."""
    if self._want_dc is not False:
      return
    if self._off_deadline is not None:
      return
    if self._applied_dc is False:
      return
    if not self.mqtt_ready():
      return
    now = time.monotonic()
    if (now - self._last_apply_attempt) < _APPLY_OFF_RETRY_S:
      return
    cloudlog.info("ecoflowd: retry DC off")
    self._apply_dc(False)

  def tick_ignition(self, ignition: bool, saw_klemmen: bool) -> None:
    if saw_klemmen:
      self._klemmen_seen = True
    if not self._klemmen_seen:
      return

    if self._want_dc is None:
      self._want_dc = ignition
      self._off_deadline = None
      cloudlog.info(
        f"ecoflowd: initial KL15={'on' if ignition else 'off'}"
        f" (mqtt_ready={self.mqtt_ready()})"
      )
      if ignition:
        self._last_verify_attempt = 0.0
      else:
        self._apply_dc(False)
      return

    if ignition == self._want_dc:
      return

    self._want_dc = ignition
    if ignition:
      self._cancel_off_delay("KL15 rising")
      self._applied_dc = False
      self._last_verify_attempt = 0.0
      cloudlog.info(f"ecoflowd: KL15 rising → ensure DC on (mqtt_ready={self.mqtt_ready()})")
    else:
      self._off_deadline = time.monotonic() + _DC_OFF_DELAY_S
      cloudlog.info(
        f"ecoflowd: KL15 falling → DC off in {_DC_OFF_DELAY_S:.0f}s"
        f" (mqtt_ready={self.mqtt_ready()})"
      )

  def close(self) -> None:
    self._teardown_session()
    self._mqtt_ready_logged = False
    self._was_network_up = False
    self._off_deadline = None


def main() -> None:
  cloudlog.info(
    f"ecoflowd: starting (READY verify every {_VERIFY_ON_S:.0f}s, "
    f"DC off delay {_DC_OFF_DELAY_S:.0f}s)"
  )
  daemon = EcoflowDaemon()
  heal_enabled(daemon.params)
  can_sock = messaging.sub_sock("can", timeout=100)
  ds_sock = messaging.sub_sock("deviceState", conflate=True)
  last_on_ts: float | None = None
  rk = Ratekeeper(5)

  try:
    while True:
      if not _ecoflow_enabled(daemon.params):
        if daemon.session is not None:
          cloudlog.info("ecoflowd: EcoflowEnabled off — disconnect")
          daemon.close()
        time.sleep(1.0)
        continue

      network_type = log.DeviceState.NetworkType.none
      ds_msgs = messaging.drain_sock(ds_sock, wait_for_one=False)
      if ds_msgs:
        try:
          network_type = int(ds_msgs[-1].deviceState.networkType)
        except Exception:
          pass

      daemon.tick_network(network_type)
      daemon.maintain_mqtt(force=daemon.wants_dc_on())

      try:
        packets = messaging.drain_sock(can_sock, wait_for_one=False)
      except Exception:
        packets = []
      now = time.monotonic()
      ignition, last_on_ts, saw_klemmen = meb_ignition_from_can(packets, now, last_on_ts)
      daemon.tick_ignition(ignition, saw_klemmen)
      daemon.tick_mqtt_ready()
      daemon.tick_pending_off()
      daemon.tick_ensure_dc_on()
      daemon.tick_ensure_dc_off()
      daemon._track_want_on()
      rk.keep_time()
  finally:
    daemon.close()
    cloudlog.info("ecoflowd: stopped")


if __name__ == "__main__":
  main()

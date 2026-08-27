#!/usr/bin/env python3
"""ecoflowd: MEB KL15 edges → EcoFlow Delta 3 12V DC (cloud MQTT).

Watch Klemmen_Status_01 (0x3C0) ZAS_Kl_15 — same bit as hardwared MebIgnitionWatch.
Rising edge → DC on immediately; falling edge → DC off after a short delay so brief
glitches / quick re-READY can cancel. Delay must stay short: after lock the MIB
hotspot drops and a long deferred MQTT off never sends (60 s road-test failed).

MQTT is pre-connected as soon as EcoflowEnabled and the network is up, so a KL15
edge ideally only sends SET (not login). If SET fails while MQTT is down, retry as
soon as MQTT is ready — do not wait on a stale reconnect timer (that made first
READY miss DC-on / feel slower).

After overnight lock, the car MIB hotspot can take tens of seconds to come up on
first READY; keep retrying DC-on until SET lands. Do not initial-sync KL15 until
at least one 0x3C0 frame is seen (empty CAN at boot used to look like KL15 off).

Gated by Params EcoflowEnabled (default off). Credentials: EcoflowPhone /
EcoflowPassword / EcoflowSn (or ECOFLOW_* env). Do not commit secrets.
"""
from __future__ import annotations

import time

from cereal import log
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
_SET_RETRY_SLEEP_S = 1.0
# Background preconnect while waiting for WiFi.
_MQTT_RETRY_S = 5.0
# Faster MQTT retry when KL15 wants DC but SET has not landed.
_MQTT_RETRY_URGENT_S = 0.5
# While want != applied, re-attempt SET at this cadence (mqtt must be up).
_APPLY_RETRY_S = 1.0
# After KL15 falls, keep 12V this long (cancelled if KL15 rises). Keep short:
# lock kills MIB WiFi; 60 s never got MQTT off. ~5 s + 2 s hold ≈ window while
# hotspot often still up. Longer short-stop keep-alive → hardware relay.
_DC_OFF_DELAY_S = 5.0
# Log if KL15 wants DC on but SET still has not landed.
_PENDING_ON_WARN_S = 15.0


def meb_ignition_from_can(
  packets, now: float, last_on_ts: float | None
) -> tuple[bool, float | None, bool]:
  """ZAS_Kl_15 on 0x3C0 byte2 bit1. Do not OR ZAS_Kl_S.

  Returns (ignition, last_on_ts, saw_klemmen) — saw_klemmen is True if any
  Klemmen_Status_01 frame was present in this batch."""
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


def _ecoflow_enabled(params: Params) -> bool:
  try:
    raw = params.get("EcoflowEnabled")
  except Exception:
    return False
  if raw is None:
    return False
  return params.get_bool("EcoflowEnabled")


def _network_up(network_type: int) -> bool:
  return network_type != log.DeviceState.NetworkType.none


class EcoflowDaemon:
  def __init__(self):
    self.params = Params()
    self.session: EcoflowSession | None = None
    self._want_dc: bool | None = None
    self._applied_dc: bool | None = None
    self._klemmen_seen = False
    self._last_connect_attempt = 0.0
    self._last_apply_attempt = 0.0
    self._last_error_log = 0.0
    self._last_pending_on_log = 0.0
    self._want_on_since: float | None = None
    self._mqtt_ready_logged = False
    self._was_mqtt_ready = False
    self._was_network_up = False
    self._off_deadline: float | None = None

  def mqtt_ready(self) -> bool:
    return self.session is not None and self.session._client is not None

  def _needs_apply(self) -> bool:
    return self._want_dc is not None and self._applied_dc is not self._want_dc

  def _mqtt_retry_interval(self) -> float:
    if self._needs_apply() and self._want_dc is True:
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
    """Keep a warm MQTT session; call every loop, independent of KL15."""
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
    if self._want_dc is True and self._applied_dc is not True:
      if self._want_on_since is None:
        self._want_on_since = time.monotonic()
      now = time.monotonic()
      if (now - self._want_on_since) >= _PENDING_ON_WARN_S:
        if (now - self._last_pending_on_log) >= _PENDING_ON_WARN_S:
          cloudlog.warning(
            f"ecoflowd: still waiting to turn DC on "
            f"({now - self._want_on_since:.0f}s, mqtt_ready={self.mqtt_ready()})"
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
          # Stale overnight MQTT often times out without raising — force reconnect.
          self._teardown_session()
          self._mqtt_ready_logged = False
          session = self._ensure_session(force=True)
          if session is None:
            break
        else:
          cloudlog.info(f"ecoflowd: DC {'on' if on else 'off'} ack={ack}")
          self._applied_dc = on
          if on:
            self._off_deadline = None
            self._want_on_since = None
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
    """Fire delayed DC off when deadline passes (needs network for MQTT SET)."""
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
    """When car WiFi/cell comes up after overnight, force MQTT + pending SET."""
    up = _network_up(network_type)
    became_up = up and not self._was_network_up
    self._was_network_up = up
    if not became_up:
      return
    cloudlog.info(f"ecoflowd: network up (type={network_type}) — connect MQTT")
    self.maintain_mqtt(force=True)
    if self._needs_apply():
      cloudlog.info(
        f"ecoflowd: network up → apply pending DC {'on' if self._want_dc else 'off'}"
      )
      self._apply_dc(bool(self._want_dc))

  def tick_mqtt_ready(self) -> None:
    """When MQTT just came up, immediately land any pending SET (first READY fix)."""
    ready = self.mqtt_ready()
    became_ready = ready and not self._was_mqtt_ready
    self._was_mqtt_ready = ready
    if not became_ready:
      return
    if not self._needs_apply():
      return
    cloudlog.info(
      f"ecoflowd: MQTT ready → apply pending DC {'on' if self._want_dc else 'off'}"
    )
    self._apply_dc(bool(self._want_dc))

  def tick_retry_apply(self) -> None:
    """Periodic retry while want != applied (no 30s MQTT-ok gate)."""
    if not self._needs_apply():
      return
    if not self.mqtt_ready():
      self.maintain_mqtt(force=False)
      return
    now = time.monotonic()
    if (now - self._last_apply_attempt) < _APPLY_RETRY_S:
      return
    cloudlog.info(
      f"ecoflowd: retry DC {'on' if self._want_dc else 'off'}"
      f" (want={self._want_dc} applied={self._applied_dc})"
    )
    self._apply_dc(bool(self._want_dc))

  def tick_ignition(self, ignition: bool, saw_klemmen: bool) -> None:
    if saw_klemmen:
      self._klemmen_seen = True

    if not self._klemmen_seen:
      return

    if self._want_dc is None:
      self._want_dc = ignition
      self._off_deadline = None
      cloudlog.info(
        f"ecoflowd: initial KL15={'on' if ignition else 'off'} → DC {'on' if ignition else 'off'}"
        f" (mqtt_ready={self.mqtt_ready()})"
      )
      self._apply_dc(ignition)
      return

    if ignition == self._want_dc:
      return

    self._want_dc = ignition
    if ignition:
      self._cancel_off_delay("KL15 rising")
      cloudlog.info(f"ecoflowd: KL15 rising → DC on (mqtt_ready={self.mqtt_ready()})")
      self._apply_dc(True)
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
    f"ecoflowd: starting (MQTT preconnect, DC off delay {_DC_OFF_DELAY_S:.0f}s, "
    f"urgent MQTT retry {_MQTT_RETRY_URGENT_S:.1f}s)"
  )
  daemon = EcoflowDaemon()
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
      daemon.maintain_mqtt(force=daemon._needs_apply() and daemon._want_dc is True)

      try:
        packets = messaging.drain_sock(can_sock, wait_for_one=False)
      except Exception:
        packets = []
      now = time.monotonic()
      ignition, last_on_ts, saw_klemmen = meb_ignition_from_can(packets, now, last_on_ts)
      daemon.tick_ignition(ignition, saw_klemmen)
      daemon.tick_mqtt_ready()
      daemon.tick_pending_off()
      daemon.tick_retry_apply()
      daemon._track_want_on()
      rk.keep_time()
  finally:
    daemon.close()
    cloudlog.info("ecoflowd: stopped")


if __name__ == "__main__":
  main()

#!/usr/bin/env python3
"""IQ-link daemon: BLE + LAN UDP HMAC → /dev/shm/sp_nav.json. Isolated from modeld."""
from __future__ import annotations

import json
import os
import time

from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.nav.ble_gatt import BleGattServer
from openpilot.sunnypilot.nav.envelope import FIXED_BLE_PSK, EnvelopeVerifier
from openpilot.sunnypilot.nav.gatt_json import MAX_GATT_BUF, pop_complete_json
from openpilot.sunnypilot.nav.protocol import parse_carrot
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
BLE_RETRY_S = 5.0


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
    self._buf = bytearray()
    self.ble = BleGattServer(self._on_gatt_write)
    self.udp = UdpNavServer(self._on_udp_datagram)

  def _handle_envelope(self, blob: bytes) -> None:
    status, data = self.verifier.inspect(blob)
    if status == "bad" or data is None:
      return
    if status == "replay":
      self._heartbeat_only()
      return
    self._ingest(data, hmac_ok=True)

  def _on_udp_datagram(self, raw: bytes) -> None:
    """One UDP datagram = one complete HMAC envelope (no GATT fragmentation)."""
    if not raw:
      return
    self._handle_envelope(raw)

  def _on_gatt_write(self, raw: bytes) -> None:
    if not raw:
      return
    self._buf.extend(raw)
    if len(self._buf) > MAX_GATT_BUF:
      self._buf.clear()
      return
    while True:
      blob = pop_complete_json(self._buf)
      if blob is None:
        return
      self._handle_envelope(blob)

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
        self._last_snap.ts = now
        write_snapshot(self._last_snap)
      return
    self._last_snap = snap
    if hmac_ok:
      self._last_hmac_ts = now
    write_snapshot(snap)
    _param_put(self.params, "IqlinkBleLinkState", LINK_CONNECTED if hmac_ok else LINK_OFF)
    _param_put(self.params, "IqlinkBleConnected", bool(hmac_ok))

  def _heartbeat_only(self) -> None:
    """Valid HMAC, same seq: refresh link, keep last TBT / lights / limit."""
    now = time.monotonic()
    self._last_hmac_ts = now
    self._last_snap.link_ok = True
    self._last_snap.link_state = LINK_CONNECTED
    self._last_snap.ts = now
    write_snapshot(self._last_snap)
    _param_put(self.params, "IqlinkBleLinkState", LINK_CONNECTED)
    _param_put(self.params, "IqlinkBleConnected", True)

  def _poll_inject(self) -> None:
    try:
      st = os.stat(INJECT_SHM_PATH)
    except OSError:
      return
    if st.st_mtime <= getattr(self, "_inject_mtime", 0):
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
      # Phone may restart seq after a drop; do not seq-replay the next session forever.
      self.verifier = EnvelopeVerifier(_psk(self.params))
    self._was_hmac_fresh = hmac_fresh
    transport_up = bool(self.ble.running or self.udp.running)
    if not enabled:
      state = LINK_OFF
      ok = False
    elif hmac_fresh:
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
    _param_put(self.params, "IqlinkBleLinkState", state)
    _param_put(self.params, "IqlinkBleConnected", ok)

  def run(self) -> None:
    rk = Ratekeeper(5)
    transport_wanted = False
    last_ble_try = 0.0
    last_udp_try = 0.0
    while True:
      enabled = _param_bool(self.params, "IqlinkEnabled", True)
      now = time.monotonic()
      if enabled:
        if not self.udp.running and (now - last_udp_try) >= BLE_RETRY_S:
          last_udp_try = now
          if not self.ble.running:
            self.verifier = EnvelopeVerifier(_psk(self.params))
          udp_ok = self.udp.start()
          cloudlog.info(f"iqlinkd UDP start ok={udp_ok}")
        if not self.ble.running and (now - last_ble_try) >= BLE_RETRY_S:
          last_ble_try = now
          if not self.udp.running:
            self.verifier = EnvelopeVerifier(_psk(self.params))
          started = self.ble.start()
          cloudlog.info(f"iqlinkd BLE start ok={started}")
        transport_wanted = True
      elif transport_wanted:
        self.ble.stop()
        self.udp.stop()
        transport_wanted = False
        self._last_hmac_ts = 0.0
        self._was_hmac_fresh = False
        last_ble_try = 0.0
        last_udp_try = 0.0
      self._poll_inject()
      self._publish_link(enabled)
      rk.keep_time()


def main() -> None:
  IqlinkDaemon().run()


if __name__ == "__main__":
  main()

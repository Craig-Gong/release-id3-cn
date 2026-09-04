"""HMAC envelope from IQ-link PROTOCOL.md. PSK is never logged."""
from __future__ import annotations

import hashlib
import hmac
import json
import time
from typing import Any

FIXED_BLE_PSK = "999999"
MAX_SKEW_MS = 120_000
CLOCK_BROKEN_SKEW_MS = 3_600_000
SEQ_REPLAY_WINDOW = 128
MAX_ENVELOPE_BYTES = 64 * 1024
_PLAUSIBLE_TS_MS_MIN = 1_704_067_200_000
_PLAUSIBLE_TS_MS_MAX = 1_893_456_000_000


def canonical_json(data: dict[str, Any]) -> bytes:
  return json.dumps(data, ensure_ascii=True, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def envelope_hmac(psk: str, seq: int, ts: int, data: dict[str, Any]) -> str:
  msg = f"{seq}:{ts}:".encode() + canonical_json(data)
  return hmac.new(psk.encode(), msg, hashlib.sha256).hexdigest()[:32]


class EnvelopeVerifier:
  def __init__(self, psk: str = FIXED_BLE_PSK):
    self.psk = psk or FIXED_BLE_PSK
    self._seen: list[int] = []

  def accept(self, raw: bytes, *, now_ms: int | None = None) -> dict[str, Any] | None:
    if not raw or len(raw) > MAX_ENVELOPE_BYTES:
      return None
    try:
      obj = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
      return None
    if not isinstance(obj, dict):
      return None
    try:
      seq = int(obj["seq"])
      ts = int(obj["ts"])
      data = obj["data"]
      digest = str(obj["hmac"]).lower()
    except (KeyError, TypeError, ValueError):
      return None
    if not isinstance(data, dict) or len(digest) != 32:
      return None
    expect = envelope_hmac(self.psk, seq, ts, data)
    if not hmac.compare_digest(expect, digest):
      return None
    clock = int(time.time() * 1000) if now_ms is None else now_ms
    skew = abs(clock - ts)
    ts_plausible = _PLAUSIBLE_TS_MS_MIN <= ts <= _PLAUSIBLE_TS_MS_MAX
    if skew > MAX_SKEW_MS and not (skew > CLOCK_BROKEN_SKEW_MS and ts_plausible):
      return None
    if seq in self._seen:
      return None
    self._seen.append(seq)
    if len(self._seen) > SEQ_REPLAY_WINDOW:
      self._seen = self._seen[-SEQ_REPLAY_WINDOW:]
    return data

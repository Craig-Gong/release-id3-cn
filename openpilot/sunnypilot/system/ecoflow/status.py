"""EcoFlow 12V DC status for the onroad HUD.

ecoflowd writes /dev/shm/sp_ecoflow.json; the UI process only reads it.
"""
from __future__ import annotations

import json
import os
import tempfile
import time
from dataclasses import dataclass
from typing import Any

SHM_PATH = "/dev/shm/sp_ecoflow.json"
STALE_S = 5.0
_FLOW_DC_ON = 14
_FLOW_DC_OFF = 4


@dataclass(frozen=True)
class EcoflowStatus:
  ts: float = 0.0
  enabled: bool = False
  mqtt: bool = False
  dc12v: bool | None = None
  kl15: bool = False
  want_on: bool = False

  def fresh(self, now: float | None = None, stale_s: float = STALE_S) -> bool:
    clock = time.monotonic() if now is None else now
    return self.ts > 0.0 and (clock - self.ts) <= stale_s


def dc12v_from_telemetry(telemetry: dict[str, Any] | None) -> bool | None:
  tel = telemetry or {}
  raw = tel.get("cfg_dc12v_out_open")
  if raw is not None:
    try:
      return int(raw) != 0
    except (TypeError, ValueError):
      pass
  flow = tel.get("flow_info_12v")
  if flow == _FLOW_DC_ON:
    return True
  if flow == _FLOW_DC_OFF:
    return False
  return None


def ecoflow_dc_label(*, enabled: bool, snap: EcoflowStatus, now: float | None = None) -> str:
  if not enabled:
    return "12V 未启用"
  if not snap.fresh(now) or snap.dc12v is None:
    return "12V 未知"
  return "12V 开" if snap.dc12v else "12V 关"


def _atomic_write(path: str, payload: dict[str, Any]) -> None:
  directory = os.path.dirname(path) or "."
  fd, tmp = tempfile.mkstemp(prefix=".sp_ecoflow_", dir=directory, text=True)
  try:
    with os.fdopen(fd, "w", encoding="utf-8") as f:
      json.dump(payload, f, ensure_ascii=True, separators=(",", ":"))
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
  except Exception:
    try:
      os.unlink(tmp)
    except OSError:
      pass
    raise


def write_status(*, enabled: bool, mqtt: bool, dc12v: bool | None, kl15: bool = False,
                 want_on: bool = False, path: str = SHM_PATH, now: float | None = None) -> None:
  _atomic_write(path, {
    "ts": time.monotonic() if now is None else now,
    "enabled": bool(enabled),
    "mqtt": bool(mqtt),
    "dc12v": None if dc12v is None else bool(dc12v),
    "kl15": bool(kl15),
    "want_on": bool(want_on),
  })


def read_status(path: str = SHM_PATH) -> EcoflowStatus:
  try:
    with open(path, encoding="utf-8") as f:
      obj = json.load(f)
  except (OSError, json.JSONDecodeError, TypeError):
    return EcoflowStatus()
  if not isinstance(obj, dict):
    return EcoflowStatus()
  dc = obj.get("dc12v")
  dc12v: bool | None
  if dc is None:
    dc12v = None
  else:
    dc12v = bool(dc)
  try:
    ts = float(obj.get("ts") or 0.0)
  except (TypeError, ValueError):
    ts = 0.0
  return EcoflowStatus(
    ts=ts,
    enabled=bool(obj.get("enabled")),
    mqtt=bool(obj.get("mqtt")),
    dc12v=dc12v,
    kl15=bool(obj.get("kl15")),
    want_on=bool(obj.get("want_on")),
  )

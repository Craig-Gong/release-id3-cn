"""Atomic lane-type snapshot in /dev/shm for desire_helper + UI."""
from __future__ import annotations

import json
import os
import tempfile
import time
from dataclasses import asdict, dataclass

LANE_TYPE_SHM_PATH = "/dev/shm/sp_lane_type.json"
# Match carrot card merge: ignore vision older than this.
LANE_TYPE_FRESH_S = 4.0
# Toast hold when solid blocks LC.
TOAST_HOLD_S = 2.5

LINE_UNKNOWN = -1
LINE_DASHED = 0
LINE_SOLID = 1


@dataclass
class LaneTypeSnapshot:
  ts: float = 0.0
  left: int = LINE_UNKNOWN
  right: int = LINE_UNKNOWN
  left_conf: float = 0.0
  right_conf: float = 0.0
  valid: bool = False
  err: str = ""
  toast: str = ""
  toast_until: float = 0.0


def write_lane_type(snap: LaneTypeSnapshot, path: str = LANE_TYPE_SHM_PATH) -> None:
  directory = os.path.dirname(path) or "."
  tmp = ""
  try:
    if directory and not os.path.isdir(directory):
      return
    fd, tmp = tempfile.mkstemp(prefix=".sp_lane_", dir=directory, text=True)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
      json.dump(asdict(snap), f, ensure_ascii=True, separators=(",", ":"))
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
  except Exception:
    if tmp:
      try:
        os.unlink(tmp)
      except OSError:
        pass


def read_lane_type(path: str = LANE_TYPE_SHM_PATH, *, now: float | None = None) -> LaneTypeSnapshot:
  try:
    with open(path, encoding="utf-8") as f:
      obj = json.load(f)
  except (OSError, json.JSONDecodeError, TypeError):
    return LaneTypeSnapshot()
  if not isinstance(obj, dict):
    return LaneTypeSnapshot()
  snap = LaneTypeSnapshot()
  for key in asdict(snap):
    if key not in obj:
      continue
    try:
      setattr(snap, key, type(getattr(snap, key))(obj[key]))
    except (TypeError, ValueError):
      continue
  clock = time.monotonic() if now is None else float(now)
  if snap.ts <= 0.0 or (clock - snap.ts) > LANE_TYPE_FRESH_S:
    return LaneTypeSnapshot(err=snap.err or "stale")
  return snap


def toast_if_fresh(snap: LaneTypeSnapshot, *, now: float | None = None) -> str:
  clock = time.monotonic() if now is None else float(now)
  if snap.toast and snap.toast_until > clock:
    return snap.toast
  return ""

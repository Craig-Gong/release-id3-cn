"""Shared helpers for desire_helper + selfdrived (shm + param)."""
from __future__ import annotations

from openpilot.common.file_params import read_file_param
from openpilot.common.params import Params, UnknownKeyName
from openpilot.sunnypilot.vision.lane_type.snapshot import (
  LINE_SOLID, LINE_UNKNOWN, LaneTypeSnapshot, read_lane_type,
)

PARAM_KEY = "LaneTypeOnnx"


def lane_type_enabled(params: Params | None = None) -> bool:
  p = params or Params()
  try:
    return bool(p.get_bool(PARAM_KEY))
  except UnknownKeyName:
    return bool(read_file_param(PARAM_KEY, False))
  except Exception:
    return bool(read_file_param(PARAM_KEY, False))


def side_line(snap: LaneTypeSnapshot, going_left: bool) -> int:
  return int(snap.left if going_left else snap.right)


def side_is_solid_raw(snap: LaneTypeSnapshot, going_left: bool) -> bool:
  """Raw solid (no hysteresis) — for toast; DesireHelper owns the hard gate."""
  return bool(snap.valid) and side_line(snap, going_left) == LINE_SOLID


def side_is_unknown(snap: LaneTypeSnapshot, going_left: bool) -> bool:
  if not snap.valid:
    return True
  return side_line(snap, going_left) == LINE_UNKNOWN

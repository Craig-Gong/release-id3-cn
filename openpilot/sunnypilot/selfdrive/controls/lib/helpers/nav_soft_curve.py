"""Highway TBT soft cap. Does not replace urban turn_prep."""
from __future__ import annotations

import math

from openpilot.common.constants import CV
from openpilot.sunnypilot.nav.snapshot import NavSnapshot

HIGHWAY_LIMIT_KPH = 70.0
HIGHWAY_V_EGO_KPH = 65.0
CURVE_DECEL = 1.2
MIN_TBT_M = 8.0


def nav_soft_curve_ms(snap: NavSnapshot, v_ego: float) -> float | None:
  if snap.tbt_dist <= MIN_TBT_M:
    return None
  if snap.road_limit_kph < HIGHWAY_LIMIT_KPH and (v_ego * CV.MS_TO_KPH) <= HIGHWAY_V_EGO_KPH:
    return None
  if snap.maneuver not in ("turn", "fork", "exit"):
    return None
  cap = math.sqrt(2.0 * CURVE_DECEL * float(snap.tbt_dist))
  road_ms = max(snap.road_limit_kph, 0.0) / 3.6
  if road_ms > 0.0:
    cap = min(cap, road_ms)
  return float(cap)

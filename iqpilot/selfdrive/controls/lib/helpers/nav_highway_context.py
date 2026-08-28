"""Shared IQ-link highway/urban context (soft-curve cap, lane guide, fork LC)."""

from __future__ import annotations

from openpilot.iqpilot.iqlink.protocol import TURN_DESIRE_WINDOW_M
from openpilot.iqpilot.selfdrive.controls.lib.helpers.turn_prep import (
  HIGHWAY_LIMIT_MS,
  URBAN_V_MAX_MS,
)

T_ARM_S = 3.75
D_MIN_M = 50.0
D_MAX_M = 120.0


def arm_distance_m(v_ego_mps: float) -> float:
  dist = float(v_ego_mps) * T_ARM_S
  return min(max(dist, D_MIN_M), D_MAX_M, float(TURN_DESIRE_WINDOW_M))


def is_highway_fast_context(road_limit_ms: float, v_ego_mps: float) -> bool:
  limit = float(road_limit_ms or 0.0)
  return limit >= HIGHWAY_LIMIT_MS or float(v_ego_mps) >= URBAN_V_MAX_MS

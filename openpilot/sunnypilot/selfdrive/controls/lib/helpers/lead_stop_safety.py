"""Hard kinematic brake / hold behind leads — independent of MPC comfort.

MPC alone uses a soft comfort envelope (A_CHANGE_COST / J_EGO) and can coast
into a stationary bumper until late. This layer caps accel from geometry:

  a <= -closing^2 / (2 * max(d - stop_gap, min_slack))

Effective stop gap matches LongitudinalTuning.stop_distance default (3.5 m).
"""
from __future__ import annotations

from typing import Any

# Keep in sync with LongitudinalTuning.stop_distance default.
STOP_GAP_M = 3.5
NEAR_HOLD_M = 4.0
SLOW_LEAD_MPS = 1.0
MIN_SLACK_M = 0.75
HARD_A_FLOOR = -3.5
CLOSE_BRAKE_A = -2.2
HOLD_A = -1.2
STANDSTILL_V = 0.35


def apply_lead_stop_safety(sm: Any, v_ego: float, a_target: float, should_stop: bool) -> tuple[float, bool]:
  try:
    lead = sm["radarState"].leadOne
  except Exception:
    return float(a_target), bool(should_stop)

  if not bool(getattr(lead, "present", False)):
    return float(a_target), bool(should_stop)

  try:
    d_rel = float(getattr(lead, "dRel", 0.0) or 0.0)
    v_lead = float(getattr(lead, "vLead", 0.0) or 0.0)
  except (TypeError, ValueError):
    return float(a_target), bool(should_stop)

  if d_rel <= 0.1:
    return float(a_target), bool(should_stop)

  v_ego = float(v_ego)
  closing = max(v_ego - max(v_lead, 0.0), 0.0)
  a_out = float(a_target)
  stop = bool(should_stop)

  if v_lead < SLOW_LEAD_MPS:
    slack = max(d_rel - STOP_GAP_M, MIN_SLACK_M)
    if closing > 0.05 or v_ego > STANDSTILL_V:
      a_need = -(closing * closing) / (2.0 * slack)
      a_out = min(a_out, max(a_need, HARD_A_FLOOR))
    if d_rel < NEAR_HOLD_M:
      if v_ego <= STANDSTILL_V:
        stop = True
        a_out = min(a_out, HOLD_A)
      elif d_rel < STOP_GAP_M:
        stop = True
        a_out = min(a_out, CLOSE_BRAKE_A)
    return a_out, stop

  # Moving lead: still enforce a kinematic floor when closing hard.
  t_follow = 1.45
  slack = max(d_rel - STOP_GAP_M - t_follow * max(v_lead, 0.0), MIN_SLACK_M)
  if closing > 0.5:
    a_need = -(closing * closing) / (2.0 * slack)
    a_out = min(a_out, max(a_need, HARD_A_FLOOR))
  return a_out, stop

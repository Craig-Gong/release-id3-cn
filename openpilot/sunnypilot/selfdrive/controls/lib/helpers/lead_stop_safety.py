"""Kinematic brake / hold behind leads — independent of MPC comfort.

MPC alone uses a soft comfort envelope (A_CHANGE_COST / J_EGO) and can coast
into a stationary bumper until late. Slow leads use the continuous comfort
stop profile (lead_comfort_stop); constant hard brakes only apply inside the
hard minimum gap or when the profile reports urgency.

Effective stop gap matches LongitudinalTuning.stop_distance default (3.5 m).
"""
from __future__ import annotations

from typing import Any

from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_comfort_stop import (
  MIN_GAP_M, STOP_V_MPS, comfort_stop_accel, settle_cap,
)

# Keep in sync with LongitudinalTuning.stop_distance default.
STOP_GAP_M = 3.5
NEAR_HOLD_M = 4.0
SLOW_LEAD_MPS = 1.0
MIN_SLACK_M = 0.75
HARD_A_FLOOR = -3.5
CLOSE_BRAKE_A = -2.2
STANDSTILL_V = 0.35


# Last-word radar floor: closer than this, no launch floor may request accel
# unless the lead is clearly pulling away.
RADAR_NO_PUNCH_M = 5.0
OPENING_V_MPS = 0.40
# ~3 km/h and past the settle gap: lead has actually left, not radar creep.
LEAD_DEPARTED_V_MPS = 0.8
LEAD_DEPARTED_GAP_M = STOP_GAP_M


def _lead_geometry(sm: Any) -> tuple[float, float] | None:
  """Radar dRel first; vision only if radar has no usable track."""
  try:
    lead = sm["radarState"].leadOne
    if bool(getattr(lead, "present", False)):
      d_rel = float(getattr(lead, "dRel", 0.0) or 0.0)
      v_lead = float(getattr(lead, "vLead", 0.0) or 0.0)
      if d_rel > 0.2:
        return d_rel, v_lead
  except Exception:
    pass
  try:
    ml = sm["modelV2"].leadsV3[0]
    if float(ml.prob) < 0.5:
      return None
    d_rel = float(ml.x[0]) - 1.52
    v_lead = float(ml.v[0])
    if 0.5 < d_rel <= 20.0:
      return d_rel, v_lead
  except Exception:
    return None
  return None


def radar_lead_departed(sm: Any) -> bool:
  """Radar says the bumper ahead has left the settle gap and is really moving."""
  geom = _lead_geometry(sm)
  if geom is None:
    return False
  d_rel, v_lead = geom
  return d_rel >= LEAD_DEPARTED_GAP_M and v_lead >= LEAD_DEPARTED_V_MPS


def apply_radar_range_floor(sm: Any, v_ego: float, a_target: float, should_stop: bool, *,
                            gas: bool = False) -> tuple[float, bool]:
  """Radar distance is the last word. Launch / green floors cannot raise past it.

  Only lowers accel. Skipped on driver gas so a manual close is still possible.
  A departed lead (moving, gap already open) is allowed to be followed — the
  floor still kinematic-brakes a hard close, but does not pin CLOSE_BRAKE
  just because dRel is under 5 m.
  """
  if gas:
    return float(a_target), bool(should_stop)
  geom = _lead_geometry(sm)
  if geom is None:
    return float(a_target), bool(should_stop)
  d_rel, v_lead = geom
  v_ego = float(v_ego)
  opening = v_lead > v_ego + OPENING_V_MPS
  departed = d_rel >= LEAD_DEPARTED_GAP_M and v_lead >= LEAD_DEPARTED_V_MPS
  a_out, stop = apply_lead_stop_safety(sm, v_ego, a_target, should_stop)
  if departed:
    return a_out, stop
  if d_rel < RADAR_NO_PUNCH_M and not opening:
    if v_ego <= STANDSTILL_V:
      stop = True
      cap = settle_cap(v_ego)
    elif d_rel < MIN_GAP_M and v_ego > v_lead + 0.05:
      stop = True
      cap = CLOSE_BRAKE_A
    else:
      # No punch toward the bumper; the comfort profile owns the decel.
      cap = 0.0
      if v_ego <= STOP_V_MPS and v_lead < SLOW_LEAD_MPS:
        stop = True
    a_out = min(a_out, cap)
  return a_out, stop


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
    a_cap, urgent = comfort_stop_accel(v_ego, d_rel, v_lead)
    if a_cap is not None:
      a_out = min(a_out, a_cap)
    if urgent and d_rel < MIN_GAP_M:
      stop = True
    if d_rel < NEAR_HOLD_M and v_ego <= STOP_V_MPS:
      stop = True
      a_out = min(a_out, settle_cap(v_ego))
    return a_out, stop

  # Moving lead: still enforce a kinematic floor when closing hard.
  t_follow = 1.45
  slack = max(d_rel - STOP_GAP_M - t_follow * max(v_lead, 0.0), MIN_SLACK_M)
  if closing > 0.5:
    a_need = -(closing * closing) / (2.0 * slack)
    a_out = min(a_out, max(a_need, HARD_A_FLOOR))
  return a_out, stop

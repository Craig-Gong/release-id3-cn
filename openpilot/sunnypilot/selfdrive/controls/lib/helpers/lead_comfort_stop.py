"""Continuous comfort stop profile behind a stopped / crawling lead.

Replaces distance-zoned constant brakes (−1.5 at 8 m, −2.2 at 5 m, −2.5 at
3.5 m) that stepped the request in one frame and felt like a stamp on the
pedal. Mirrors the vision red-light path: kinematic decel toward the settle
gap (front-loaded by a speed margin), released toward −0.5 before standstill.

  soft target : COMFORT_GAP_M (LongitudinalTuning.stop_distance default)
  hard minimum: MIN_GAP_M (ISO 22179 cmin ≥ 2 m, plus margin)

If stopping at the soft gap needs more than COMFORT_MAX_A, the profile may end
closer (down to MIN_GAP_M) instead of braking harder. Only when stopping at
MIN_GAP_M needs ≥ URGENT_REQ_A is the request urgent (unsmoothed, firm).
"""
from __future__ import annotations

from typing import Any

COMFORT_GAP_M = 3.5
MIN_GAP_M = 2.5
COMFORT_MAX_A = 1.5
URGENT_REQ_A = 2.5
HARD_A_FLOOR = -3.5
INSIDE_MIN_FAST_A = -2.2
INSIDE_MIN_SLOW_A = -1.2
INSIDE_MIN_FAST_CLOSING = 0.5
MIN_SLACK_M = 0.3
CLOSING_EPS = 0.05
PLAN_MARGIN_S = 0.4
FAST_PLAN_BOOST = 1.25
OVERSHOOT_BLEND_LO = 3.5
OVERSHOOT_BLEND_HI = 5.0

# Release the brake before the car settles (human "ease off" before stop).
TAPER_V_MPS = 1.2
TAPER_A = 0.5

# LongControl stopping (MEB HALTEN + ACC_Anhalten, Anhalteweg 0) only below
# this speed; above it the profile alone slows the car.
STOP_V_MPS = 0.5
# Full hold pin only once actually settled; rolling the last few cm at −1.2
# is the nod at the end of a stop.
SETTLED_V_MPS = 0.1
SETTLED_HOLD_A = -1.2

# Brake-onset slew for the planner output (non-urgent, non-light stops).
JERK_ONSET = 1.0
JERK_LOW_V = 2.5
JERK_HIGH_V = 2.0
JERK_BP_V = (5.0, 20.0)
JERK_ONSET_A = (-1.0, -0.3)
SLEW_MIN_V = 0.5

URGENT_TTC_S = 3.0
URGENT_MIN_GAP_M = 2.0
URGENT_MIN_GAP_A = 2.0
URGENT_LEAD_BRAKE_A = -1.5
URGENT_LEAD_BRAKE_D_M = 40.0


def comfort_stop_accel(v_ego: float, d_rel: float, v_lead: float) -> tuple[float | None, bool]:
  """(accel cap or None, urgent) for a slow lead. None = not closing."""
  v = max(float(v_ego), 0.0)
  closing = max(v - max(float(v_lead), 0.0), 0.0)
  if closing <= CLOSING_EPS:
    return None, False

  rem_hard = float(d_rel) - MIN_GAP_M
  if rem_hard <= 0.0:
    if closing > INSIDE_MIN_FAST_CLOSING:
      return INSIDE_MIN_FAST_A, True
    return INSIDE_MIN_SLOW_A, False

  a_req_hard = closing * closing / (2.0 * max(rem_hard, MIN_SLACK_M))
  if a_req_hard >= URGENT_REQ_A:
    return max(-a_req_hard, HARD_A_FLOOR), True

  # Speed-proportional margin shrinks to 0 at rest: required decel then falls
  # over the approach instead of climbing when the car under-delivers.
  rem_soft = float(d_rel) - COMFORT_GAP_M
  a_req_soft = closing * closing / (2.0 * max(rem_soft, MIN_SLACK_M))
  rem_plan = rem_soft - PLAN_MARGIN_S * closing
  a_req_plan = closing * closing / (2.0 * max(rem_plan, MIN_SLACK_M))
  a_need = a_req_plan
  if a_req_plan > COMFORT_MAX_A:
    # Beyond comfort: fast arrivals keep some front-loading but never inflate
    # far past the soft-gap physics; slow arrivals settle closer (down to
    # MIN_GAP_M) instead of stamping the brake.
    fast = min(a_req_plan, max(COMFORT_MAX_A, a_req_soft) * FAST_PLAN_BOOST, -HARD_A_FLOOR)
    slow = max(COMFORT_MAX_A, a_req_hard)
    w = min(max((closing - OVERSHOOT_BLEND_LO) / (OVERSHOOT_BLEND_HI - OVERSHOOT_BLEND_LO), 0.0), 1.0)
    a_need = w * fast + (1.0 - w) * slow

  if v < TAPER_V_MPS and a_need > TAPER_A:
    taper = TAPER_A + (a_need - TAPER_A) * (v / TAPER_V_MPS)
    a_need = max(taper, a_req_hard)
  return -a_need, False


def settle_cap(v_ego: float) -> float:
  """Accel cap while finishing a stop behind a lead (below STOP_V_MPS)."""
  return SETTLED_HOLD_A if float(v_ego) <= SETTLED_V_MPS else -TAPER_A


def _interp(x: float, xp: tuple[float, float], fp: tuple[float, float]) -> float:
  if x <= xp[0]:
    return fp[0]
  if x >= xp[1]:
    return fp[1]
  t = (x - xp[0]) / (xp[1] - xp[0])
  return fp[0] + t * (fp[1] - fp[0])


def brake_jerk_limit(v_ego: float, a_prev: float) -> float:
  """Allowed brake-increase rate (m/s³): gentle onset, firmer once braking."""
  j_main = _interp(float(v_ego), JERK_BP_V, (JERK_LOW_V, JERK_HIGH_V))
  return _interp(float(a_prev), JERK_ONSET_A, (j_main, min(JERK_ONSET, j_main)))


def lead_brake_urgent(d_rel: float, v_lead: float, a_lead: float, v_ego: float) -> bool:
  """Physics check that bypasses smoothing: TTC, min-gap need, or lead braking hard."""
  d = float(d_rel)
  closing = float(v_ego) - max(float(v_lead), 0.0)
  if d <= URGENT_MIN_GAP_M and closing > CLOSING_EPS:
    return True
  if closing > CLOSING_EPS:
    if d / closing < URGENT_TTC_S:
      return True
    if closing * closing / (2.0 * max(d - URGENT_MIN_GAP_M, MIN_SLACK_M)) >= URGENT_MIN_GAP_A:
      return True
  return float(a_lead) <= URGENT_LEAD_BRAKE_A and d < URGENT_LEAD_BRAKE_D_M and closing > -1.0


class BrakeOnsetLimiter:
  """Slew only the brake-increase direction of the final planner accel."""

  def __init__(self) -> None:
    self.a_prev: float | None = None

  def reset(self) -> None:
    self.a_prev = None

  def update(self, a_target: float, v_ego: float, dt: float, *, bypass: bool) -> float:
    a = float(a_target)
    if bypass or self.a_prev is None or float(v_ego) < SLEW_MIN_V:
      self.a_prev = a
      return a
    # Lifting off throttle to 0 is free; only brake build-up is slewed.
    start = min(self.a_prev, 0.0)
    floor = start - brake_jerk_limit(v_ego, start) * max(float(dt), 1e-3)
    out = max(a, floor)
    self.a_prev = out
    return out


def read_lead_kinematics(sm: Any) -> tuple[float, float, float] | None:
  """(dRel, vLead, aLead) from radar leadOne, vision fallback when radar has none."""
  try:
    lead = sm["radarState"].leadOne
    if bool(getattr(lead, "present", False)):
      d_rel = float(getattr(lead, "dRel", 0.0) or 0.0)
      if d_rel > 0.2:
        v_lead = float(getattr(lead, "vLead", 0.0) or 0.0)
        a_lead = float(getattr(lead, "aLeadK", 0.0) or 0.0)
        return d_rel, v_lead, a_lead
  except Exception:
    pass
  try:
    ml = sm["modelV2"].leadsV3[0]
    if float(ml.prob) < 0.5:
      return None
    d_rel = float(ml.x[0]) - 1.52
    if 0.5 < d_rel <= 60.0:
      a_lead = float(ml.a[0]) if len(ml.a) else 0.0
      return d_rel, float(ml.v[0]), a_lead
  except Exception:
    return None
  return None

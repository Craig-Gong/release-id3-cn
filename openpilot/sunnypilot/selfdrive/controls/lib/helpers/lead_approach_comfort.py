"""Lead Approach Comfort (LAC): far deadband, then lift → coast → track.

Far mild closes are left to Lead MPC (no early −0.4 stamp). LAC only engages
when gap error / TTC says a human would ease off. Slow/stopped leads stay on
lead_comfort_stop.

Priority (caller): gas > urgent > red/CUT > LAC > MPC > follow floor.
"""
from __future__ import annotations

from typing import Any

from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_comfort_stop import (
  COMFORT_GAP_M, HARD_A_FLOOR, lead_brake_urgent, read_lead_kinematics,
)

T_FOLLOW = 1.45
MIN_SLACK_M = 0.75
CLOSING_EPS = 0.05

# Engage only when closing actually needs attention (not far light creep).
ENGAGE_REQ = 0.45
TTC_ENGAGE_S = 8.0
# Soft tiers once engaged: lift (0) → coast → track −a_req.
LIFT_REQ = 0.90
COAST_REQ = 1.40
LIFT_A = 0.0
COAST_A = -0.55

# Rising-edge lock: start at lift (0), reach full profile over LOCK_FADE_S.
LOCK_FADE_S = 2.0
LOCK_ABSENT_S = 0.35

# Follow accel cap vs speed (slightly above empty-road; allows catching lights).
A_FOLLOW_BP = (0.0, 10.0, 25.0, 40.0)
A_FOLLOW_VALS = (1.15, 0.90, 0.65, 0.45)
FOLLOW_LEAD_GATE_M = 50.0

# Softer positive onset while the gap is opening.
FOLLOW_OPEN_JERK = 0.80
OPENING_V_MPS = 0.30


def _clip(x: float, lo: float, hi: float) -> float:
  return lo if x < lo else hi if x > hi else x


def _interp(x: float, xp: tuple[float, ...], fp: tuple[float, ...]) -> float:
  if x <= xp[0]:
    return fp[0]
  if x >= xp[-1]:
    return fp[-1]
  for i in range(1, len(xp)):
    if x <= xp[i]:
      t = (x - xp[i - 1]) / (xp[i] - xp[i - 1])
      return fp[i - 1] + t * (fp[i] - fp[i - 1])
  return fp[-1]


def follow_accel_cap(v_ego: float) -> float:
  return float(_interp(float(v_ego), A_FOLLOW_BP, A_FOLLOW_VALS))


def soft_slack_m(d_rel: float, v_lead: float) -> float:
  return max(float(d_rel) - COMFORT_GAP_M - T_FOLLOW * max(float(v_lead), 0.0), MIN_SLACK_M)


def approach_a_req(v_ego: float, d_rel: float, v_lead: float) -> float:
  """Required decel magnitude to settle at soft follow gap (0 if not closing)."""
  closing = max(float(v_ego) - max(float(v_lead), 0.0), 0.0)
  if closing <= CLOSING_EPS:
    return 0.0
  return closing * closing / (2.0 * soft_slack_m(d_rel, v_lead))


def approach_accel_profile(v_ego: float, d_rel: float, v_lead: float) -> float | None:
  """Accel cap for a moving lead. None = far/mild — leave to MPC."""
  closing = max(float(v_ego) - max(float(v_lead), 0.0), 0.0)
  if closing <= CLOSING_EPS:
    return None
  a_req = approach_a_req(v_ego, d_rel, v_lead)
  ttc = float(d_rel) / closing
  # Far light creep: do not pre-brake (was −0.4 with TTC 20–40 s).
  if a_req < ENGAGE_REQ or ttc > TTC_ENGAGE_S:
    return None
  if a_req <= LIFT_REQ:
    return LIFT_A
  if a_req <= COAST_REQ:
    w = (a_req - LIFT_REQ) / max(COAST_REQ - LIFT_REQ, 1e-3)
    return LIFT_A + w * (COAST_A - LIFT_A)
  return max(-a_req, HARD_A_FLOOR)


class LeadApproachComfort:
  """Stateful lock fade + per-frame LAC apply."""

  def __init__(self) -> None:
    self._present_s = 0.0
    self._absent_s = LOCK_ABSENT_S
    self._was_present = False
    self.open_jerk: float | None = None

  def reset(self) -> None:
    self._present_s = 0.0
    self._absent_s = LOCK_ABSENT_S
    self._was_present = False
    self.open_jerk = None

  def _update_lock(self, present: bool, dt: float) -> float:
    """Return fade weight 0..1 (0 = just locked, 1 = full authority)."""
    dt = max(float(dt), 1e-3)
    if present:
      if not self._was_present and self._absent_s >= LOCK_ABSENT_S:
        self._present_s = 0.0
      else:
        self._present_s += dt
      self._absent_s = 0.0
      self._was_present = True
      return _clip(self._present_s / LOCK_FADE_S, 0.0, 1.0)
    self._absent_s += dt
    if self._absent_s >= LOCK_ABSENT_S:
      self._was_present = False
      self._present_s = 0.0
    return 1.0

  def apply(self, sm: Any, v_ego: float, a_target: float, *,
            dt: float = 0.05, gas: bool = False,
            light_stop: bool = False) -> float:
    """Lower accel only. Skipped on gas / active light-stop path."""
    self.open_jerk = None
    if gas or light_stop:
      self._update_lock(False, dt)
      return float(a_target)

    kin = read_lead_kinematics(sm)
    if kin is None:
      self._update_lock(False, dt)
      return float(a_target)

    d_rel, v_lead, a_lead = kin
    if d_rel > FOLLOW_LEAD_GATE_M:
      self._update_lock(False, dt)
      return float(a_target)

    fade_w = self._update_lock(True, dt)
    a_out = float(a_target)
    urgent = lead_brake_urgent(d_rel, v_lead, a_lead, v_ego)
    profile: float | None = None

    # Moving leads: optional soft profile (None → MPC owns far closes).
    if float(v_lead) >= 1.0:
      profile = approach_accel_profile(v_ego, d_rel, v_lead)
      if profile is not None:
        if urgent:
          a_cap = profile
        else:
          a_cap = LIFT_A + fade_w * (profile - LIFT_A)
        a_out = min(a_out, a_cap)

    # Soften speculative hard bites only while the physics profile is still
    # lift/coast. If profile already asks for more than coast (or stop_safety
    # already stamped it), do not hold the pedal at COAST for ~1 s — that
    # delayed real cut-ins (e.g. 20→12 @25–35 m, TTC just above urgent).
    if not urgent and fade_w < 0.5 and (profile is None or profile >= COAST_A - 1e-6):
      closing = float(v_ego) - max(float(v_lead), 0.0)
      if closing > CLOSING_EPS and a_out < COAST_A:
        a_out = max(a_out, COAST_A)

    opening = float(v_lead) > float(v_ego) + OPENING_V_MPS
    if a_out > 0.0:
      a_out = min(a_out, follow_accel_cap(v_ego))
    if opening and a_out > 0.0:
      self.open_jerk = FOLLOW_OPEN_JERK

    return a_out

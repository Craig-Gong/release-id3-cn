"""Congestion follow takeoff: soft floor, accel onset slew, lead-decel anticipate.

Fixes the accelerate-then-brake jab when a slow lead starts, pauses, and
starts again. Does **not** touch nav/head-car MEB ANFAHREN floors
(`_GO_LAUNCH_FLOOR_A` 0.9) or the comfort brake profile.
"""
from __future__ import annotations

from typing import Any

from openpilot.sunnypilot.selfdrive.controls.lib.helpers.green_follow_lead import (
  FOLLOW_LEAD_LAUNCH_V_EGO,
  FOLLOW_LEAD_START_ACCEL,
  LEAD_GO_SPEED_MPS,
  STOPPED_LEAD_CREEP_M,
  STOPPED_LEAD_GAP_M,
  read_follow_lead,
)
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_comfort_stop import (
  SLEW_MIN_V,
  read_lead_kinematics,
)

# Soft follow floor (was a hard 1.2 whenever the bumper inched forward).
FOLLOW_FLOOR_SOFT_A = 0.85
FOLLOW_FLOOR_BASE_A = 0.30
FOLLOW_FLOOR_A_LEAD_GAIN = 0.55
FOLLOW_FLOOR_V_REL_GAIN = 0.35
# Confirm the lead is really rolling before applying any positive floor.
LAUNCH_CONFIRM_V = 0.50
LAUNCH_CONFIRM_S = 0.25
# Drop the floor immediately when the lead is braking or we are closing.
FOLLOW_DECEL_KILL_A = -0.30
FOLLOW_CLOSING_KILL_V = 0.40

# Accel-onset slew (positive half-axis only). Below car-side 4.0 m/s³.
ACCEL_JERK_ONSET = 1.2

# Phase 3: ease off when the lead starts braking (human coast).
ANTICIPATE_A_LEAD = -0.40
ANTICIPATE_HARD_A = -0.80
ANTICIPATE_V_MAX = 7.0   # ~25 km/h
ANTICIPATE_D_MAX = 25.0
ANTICIPATE_CAP_A = 0.0
ANTICIPATE_HARD_CAP_A = -0.35


def _clip(x: float, lo: float, hi: float) -> float:
  return lo if x < lo else hi if x > hi else x


class FollowGoConfirm:
  """Hysteresis: lead must be clearly rolling before a follow floor engages."""

  def __init__(self) -> None:
    self._go_s = 0.0
    self.confirmed = False

  def reset(self) -> None:
    self._go_s = 0.0
    self.confirmed = False

  def update(self, v_lead: float, dt: float) -> bool:
    v = float(v_lead)
    dt = max(float(dt), 1e-3)
    if v >= LAUNCH_CONFIRM_V:
      self._go_s += dt
      if self._go_s >= LAUNCH_CONFIRM_S:
        self.confirmed = True
    elif v < LEAD_GO_SPEED_MPS:
      self._go_s = 0.0
      self.confirmed = False
    # Band LEAD_GO..CONFIRM: keep previous confirmed / timer state.
    return self.confirmed


def compute_follow_floor(v_ego: float, d_rel: float, v_lead: float, a_lead: float,
                         *, confirmed: bool) -> float | None:
  """Positive accel floor for slow follow takeoff, or None = do not raise."""
  if float(v_ego) > FOLLOW_LEAD_LAUNCH_V_EGO:
    return None
  if float(d_rel) < STOPPED_LEAD_CREEP_M:
    return None
  if not confirmed:
    return None
  if float(v_lead) < LEAD_GO_SPEED_MPS:
    return None
  if float(a_lead) <= FOLLOW_DECEL_KILL_A:
    return None
  closing = float(v_ego) - float(v_lead)
  if closing > FOLLOW_CLOSING_KILL_V:
    return None

  v_rel = float(v_lead) - float(v_ego)  # >0 opening / lead faster
  a_floor = (
    FOLLOW_FLOOR_BASE_A
    + FOLLOW_FLOOR_A_LEAD_GAIN * max(float(a_lead), 0.0)
    + FOLLOW_FLOOR_V_REL_GAIN * max(v_rel, 0.0)
  )
  # Past settle gap with a clearly rolling lead: allow a slightly firmer floor.
  if float(d_rel) >= STOPPED_LEAD_GAP_M and float(v_lead) >= LAUNCH_CONFIRM_V:
    a_floor = max(a_floor, FOLLOW_FLOOR_BASE_A)
  return _clip(a_floor, 0.0, FOLLOW_FLOOR_SOFT_A)


def compute_anticipate_cap(v_ego: float, d_rel: float, v_lead: float,
                           a_lead: float) -> float | None:
  """Upper accel cap when the lead is decelerating in congestion (coast)."""
  if float(v_ego) > ANTICIPATE_V_MAX or float(d_rel) > ANTICIPATE_D_MAX:
    return None
  if float(d_rel) < 0.5:
    return None
  # Only while still roughly matching the bumper — not a high-speed cut-in.
  if float(v_ego) > float(v_lead) + 2.5:
    return None
  a = float(a_lead)
  if a <= ANTICIPATE_HARD_A:
    return ANTICIPATE_HARD_CAP_A
  if a <= ANTICIPATE_A_LEAD:
    # Blend toward coast as lead brake builds from −0.4 → −0.8.
    w = _clip((ANTICIPATE_A_LEAD - a) / (ANTICIPATE_A_LEAD - ANTICIPATE_HARD_A), 0.0, 1.0)
    return ANTICIPATE_CAP_A * (1.0 - w) + ANTICIPATE_HARD_CAP_A * w
  return None


class AccelOnsetLimiter:
  """Slew only positive accel increases of the final planner output."""

  def __init__(self) -> None:
    self.a_prev: float | None = None

  def reset(self) -> None:
    self.a_prev = None

  def update(self, a_target: float, v_ego: float, dt: float, *, bypass: bool) -> float:
    a = float(a_target)
    if bypass or self.a_prev is None or float(v_ego) < SLEW_MIN_V:
      self.a_prev = a
      return a
    # Dropping toward 0 / brake is free here (brake limiter owns that half).
    if a <= self.a_prev:
      self.a_prev = a
      return a
    start = max(self.a_prev, 0.0)
    ceil = start + ACCEL_JERK_ONSET * max(float(dt), 1e-3)
    out = min(a, ceil)
    # Never hold a negative a_prev from blocking a fresh positive request past 0
    # for more than one slew step — standstill go uses bypass.
    if self.a_prev < 0.0 and a > 0.0:
      out = min(a, max(ceil, 0.0))
    self.a_prev = out
    return out


class FollowLaunchController:
  """Stateful follow takeoff: confirm + soft floor + anticipate cap."""

  def __init__(self) -> None:
    self.confirm = FollowGoConfirm()

  def reset(self) -> None:
    self.confirm.reset()

  def apply(self, sm: Any, v_ego: float, a_target: float, dt: float = 0.05) -> float:
    if float(v_ego) > FOLLOW_LEAD_LAUNCH_V_EGO:
      self.confirm.reset()
      return float(a_target)

    lead = read_follow_lead(sm)
    if not lead.present:
      self.confirm.reset()
      return float(a_target)

    kin = read_lead_kinematics(sm)
    a_lead = float(kin[2]) if kin is not None else 0.0
    confirmed = self.confirm.update(lead.v_lead, dt)

    a_out = float(a_target)

    # Critically closed: never add a positive floor. Stopped lead forbids accel.
    if lead.d_rel < STOPPED_LEAD_CREEP_M:
      if lead.v_lead < LEAD_GO_SPEED_MPS:
        return min(a_out, 0.0)
      # Lead rolling but bumper close: pass through (range floor still owns).
      a_cap = compute_anticipate_cap(v_ego, lead.d_rel, lead.v_lead, a_lead)
      return min(a_out, a_cap) if a_cap is not None else a_out

    floor = compute_follow_floor(
      v_ego, lead.d_rel, lead.v_lead, a_lead, confirmed=confirmed,
    )
    if floor is not None:
      a_out = max(a_out, floor)

    # Soft launch while still nose-ish and lead stopped: keep the old cap.
    if lead.v_lead < LEAD_GO_SPEED_MPS:
      a_out = min(a_out, FOLLOW_LEAD_START_ACCEL)

    a_cap = compute_anticipate_cap(v_ego, lead.d_rel, lead.v_lead, a_lead)
    if a_cap is not None:
      a_out = min(a_out, a_cap)
    return a_out

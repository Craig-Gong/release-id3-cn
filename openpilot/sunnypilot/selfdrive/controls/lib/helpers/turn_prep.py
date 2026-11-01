"""Urban turn-prep: cap planned speed before the corner, never MAX.

Two stages, matching Low-Speed Turn Planning:
  1. Blinker (or IQ-link urban send_turn) while still above G → approach ~G-3.
  2. Below G and path / steering / near nav turn matches → ~20 km/h.
     Applies to small and big models. E2E alone was too fast on 90° corners.

Planner must only min() this onto v_cruise. No auto blinker / NavExit ALC.
"""
from __future__ import annotations

import time

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.sunnypilot.nav.snapshot import NavSnapshot
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_turn import (
  nav_led_approach,
  nav_near_matching_turn,
)

TURN_TRIGGER_MPS = 45.0 * CV.KPH_TO_MS
DEFAULT_TURN_GATE_MPS = 40.0 * CV.KPH_TO_MS

LC_STARTING = 2
LC_FINISHING = 3

APPROACH_BELOW_GATE_MS = 3.0 * CV.KPH_TO_MS
ENTER_ABOVE_GATE_MS = 1.0 * CV.KPH_TO_MS
TURN_IN_MS = 20.0 * CV.KPH_TO_MS
# Staged exit: 30 → 40 over ~5 s so post-corner accel is not a full MAX punch.
POST_TURN_LO_MS = 30.0 * CV.KPH_TO_MS
POST_TURN_HI_MS = 40.0 * CV.KPH_TO_MS
POST_TURN_HOLD_S = 5.0
POST_TURN_MID_S = 2.5
# Back-compat alias for tests / callers.
POST_TURN_MS = POST_TURN_LO_MS
URBAN_V_MAX_MS = 65.0 * CV.KPH_TO_MS
HIGHWAY_LIMIT_MS = 70.0 * CV.KPH_TO_MS
PATH_LATERAL_M = 2.2
PATH_RANGE_MIN_M = 12.0
PATH_RANGE_MAX_M = 45.0
STEER_IN_TURN_DEG = 20.0
STEER_STRAIGHT_DEG = 12.0
PATH_STRAIGHT_M = 1.0

STAGE_OFF = 0
STAGE_APPROACH = 1
STAGE_TURN_IN = 2
STAGE_POST = 3


def _mph_param_to_mps(raw_value) -> float:
  try:
    return float(raw_value) * CV.MPH_TO_MS
  except (TypeError, ValueError):
    return DEFAULT_TURN_GATE_MPS


def _as_int(value) -> int:
  if value is None:
    return 0
  raw = getattr(value, "raw", value)
  try:
    return int(raw)
  except (TypeError, ValueError):
    return 0


def _one_blinker(left: bool, right: bool) -> bool:
  return bool(left) ^ bool(right)


def _path_lateral_m(path_x, path_y) -> float | None:
  if path_x is None or path_y is None:
    return None
  xs = list(path_x)
  ys = list(path_y)
  if len(xs) < 4 or len(xs) != len(ys):
    return None
  samples = [float(ys[i]) for i, x in enumerate(xs) if PATH_RANGE_MIN_M <= float(x) <= PATH_RANGE_MAX_M]
  if not samples:
    mid = len(ys) // 2
    return float(ys[mid])
  return max(samples, key=abs)


def _path_matches_blinker(lat_m: float | None, left: bool, right: bool) -> bool:
  if lat_m is None:
    return False
  if left and lat_m >= PATH_LATERAL_M:
    return True
  if right and lat_m <= -PATH_LATERAL_M:
    return True
  return False


def _path_straight(lat_m: float | None) -> bool:
  return lat_m is not None and abs(lat_m) <= PATH_STRAIGHT_M


def _steer_into_blinker(angle_deg: float, left: bool, right: bool) -> bool:
  if left and angle_deg >= STEER_IN_TURN_DEG:
    return True
  if right and angle_deg <= -STEER_IN_TURN_DEG:
    return True
  return False


def _steer_straight(angle_deg: float) -> bool:
  return abs(angle_deg) <= STEER_STRAIGHT_DEG


class UrbanTurnPrep:
  def __init__(self, params: Params | None = None):
    self.params = params if params is not None else Params()
    self.stage = STAGE_OFF
    self._post_until = 0.0
    self._post_started = 0.0
    self._turn_planning_on = True
    self._gate_mps = DEFAULT_TURN_GATE_MPS
    self._refresh_tick = 0
    self.read_params()

  def read_params(self) -> None:
    self._turn_planning_on = self.params.get_bool("LaneTurnDesire")
    requested = _mph_param_to_mps(self.params.get("LaneTurnValue", return_default=True))
    self._gate_mps = min(TURN_TRIGGER_MPS, max(requested, 5.0 * CV.MPH_TO_MS))

  def _maybe_refresh_params(self) -> None:
    if self._refresh_tick % 50 == 0:
      self.read_params()
    self._refresh_tick += 1

  def reset(self) -> None:
    self.stage = STAGE_OFF
    self._post_until = 0.0
    self._post_started = 0.0

  def _approach_target(self) -> float:
    return max(self._gate_mps - APPROACH_BELOW_GATE_MS, TURN_IN_MS)

  def _post_target(self) -> float:
    """30 km/h for ~2.5 s, then 40 until hold expires."""
    elapsed = time.monotonic() - float(self._post_started or 0.0)
    if elapsed < POST_TURN_MID_S:
      return POST_TURN_LO_MS
    return POST_TURN_HI_MS

  def _enter_post(self) -> float:
    self.stage = STAGE_POST
    now = time.monotonic()
    self._post_started = now
    self._post_until = now + POST_TURN_HOLD_S
    return self._post_target()

  def update(
    self,
    *,
    v_ego: float,
    enabled: bool,
    left_blinker: bool,
    right_blinker: bool,
    gas_pressed: bool,
    steering_angle_deg: float,
    posted_limit_ms: float = 0.0,
    lane_change_state: int = 0,
    path_x=None,
    path_y=None,
    big: bool = False,
    snap: NavSnapshot | None = None,
  ) -> float | None:
    self._maybe_refresh_params()
    _ = big
    nav = snap if snap is not None else NavSnapshot()
    nav_led = nav_led_approach(nav)

    if self.stage == STAGE_POST:
      if gas_pressed or not enabled or time.monotonic() >= self._post_until:
        self.reset()
        return None
      return self._post_target()

    if not enabled or gas_pressed or not self._turn_planning_on:
      self.reset()
      return None
    if not _one_blinker(left_blinker, right_blinker) and not nav_led:
      # Typical: cancel the stalk on exit. Keep the staged exit only after a
      # real turn-in, not after an aborted approach / lane-change blinker.
      if self.stage == STAGE_TURN_IN:
        return self._enter_post()
      self.reset()
      return None
    if _as_int(lane_change_state) in (LC_STARTING, LC_FINISHING):
      self.reset()
      return None
    if posted_limit_ms >= HIGHWAY_LIMIT_MS:
      self.reset()
      return None
    if v_ego > URBAN_V_MAX_MS:
      self.reset()
      return None

    lat_m = _path_lateral_m(path_x, path_y)
    nav_left = nav_led and nav.maneuver_dir == "left"
    nav_right = nav_led and nav.maneuver_dir == "right"
    left = bool(left_blinker or nav_left)
    right = bool(right_blinker or nav_right)
    turning = (
      _path_matches_blinker(lat_m, left, right)
      or _steer_into_blinker(steering_angle_deg, left, right)
      or nav_near_matching_turn(left=left, right=right, snap=nav)
    )
    below_gate = v_ego < self._gate_mps
    finished = (
      self.stage == STAGE_TURN_IN
      and _steer_straight(steering_angle_deg)
      and _path_straight(lat_m)
    )
    if finished:
      return self._enter_post()

    if self.stage == STAGE_OFF:
      if v_ego > self._gate_mps + ENTER_ABOVE_GATE_MS:
        self.stage = STAGE_APPROACH
      else:
        return None

    if below_gate and turning:
      self.stage = STAGE_TURN_IN

    if self.stage == STAGE_TURN_IN:
      return TURN_IN_MS
    return self._approach_target()

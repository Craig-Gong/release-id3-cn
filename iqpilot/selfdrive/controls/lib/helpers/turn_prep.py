"""Urban blinker turn-prep: cap planned speed before the corner, never MAX.

Two stages, matching Low-Speed Turn Planning:
  1. Blinker on while still above the turn gate G → approach ~G (command G-3 so
     desire can latch without sitting on the gate). Typical G is 40 km/h, hard cap 45.
  2. Below G and the path / nav / steering shows the matching turn → ~20 km/h
     turn-in. E2E may still go slower via min() in the planner.

Does not switch IQ.Dynamic modes. Planner must only min() this onto v_target.

Enum integers match cereal (IQTurnSignalDirection, NavDirection, ManeuverType,
ManeuverPhase, LaneChangeState) so capnp values can be passed through as-is.
"""

from __future__ import annotations

from iqpilot.common.constants import CV
from iqpilot.common.params import Params
from iqpilot.iqlink.protocol import TURN_DESIRE_WINDOW_M

# Keep in sync with lane_turn.py (Low-Speed Turn Planning).
TURN_TRIGGER_MPS = 45.0 * CV.KPH_TO_MS
DEFAULT_TURN_GATE_MPS = 40.0 * CV.KPH_TO_MS

# cereal.custom.IQTurnSignalDirection / NavDirection
TURN_LEFT = 1
TURN_RIGHT = 2
NAV_LEFT = 1
NAV_RIGHT = 2
# cereal.custom.IQNavState.ManeuverType
MANEUVER_TURN = 1
MANEUVER_EXIT = 2
MANEUVER_MERGE = 3
MANEUVER_ROUNDABOUT = 7
# cereal.custom.IQNavState.ManeuverPhase
PHASE_TURN_ACTIVE = 2
PHASE_HIGHWAY_PREPARE = 3
PHASE_HIGHWAY_COMMIT = 4
# cereal.log.LaneChangeState
LC_STARTING = 2
LC_FINISHING = 3

APPROACH_BELOW_GATE_MS = 3.0 * CV.KPH_TO_MS
ENTER_ABOVE_GATE_MS = 1.0 * CV.KPH_TO_MS
TURN_IN_MS = 20.0 * CV.KPH_TO_MS
URBAN_V_MAX_MS = 65.0 * CV.KPH_TO_MS
HIGHWAY_LIMIT_MS = 70.0 * CV.KPH_TO_MS
PATH_LATERAL_M = 2.2
PATH_RANGE_MIN_M = 12.0
PATH_RANGE_MAX_M = 45.0
STEER_IN_TURN_DEG = 20.0
STEER_STRAIGHT_DEG = 12.0
PATH_STRAIGHT_M = 1.0
NAV_NEAR_TURN_M = 80.0


def eval_nav_turn_desire(
  *,
  direction_raw: int,
  turn_dist_m: float,
  v_ego_mps: float,
  left_blinker: bool,
  right_blinker: bool,
  left_blindspot: bool,
  right_blindspot: bool,
  suppress_highway_blinker: bool = False,
) -> int:
  """Nav turn execution gate (A1): toast can fire earlier; desire only when confirmed."""
  if direction_raw not in (TURN_LEFT, TURN_RIGHT):
    return 0
  if direction_raw == TURN_LEFT and left_blindspot:
    return 0
  if direction_raw == TURN_RIGHT and right_blindspot:
    return 0
  near_exec = 0.0 < float(turn_dist_m) <= NAV_NEAR_TURN_M
  slow_enough = float(v_ego_mps) < TURN_TRIGGER_MPS
  blinker_ok = (
    (direction_raw == TURN_LEFT and left_blinker and not right_blinker) or
    (direction_raw == TURN_RIGHT and right_blinker and not left_blinker)
  )
  if blinker_ok:
    if suppress_highway_blinker and float(v_ego_mps) >= TURN_TRIGGER_MPS:
      blinker_ok = False
  if blinker_ok or (near_exec and slow_enough):
    return direction_raw
  return 0


STAGE_OFF = 0
STAGE_APPROACH = 1
STAGE_TURN_IN = 2


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


def _nav_is_lane_change(nav_phase, nav_maneuver_type, nav_send_lc: bool) -> bool:
  if nav_send_lc:
    return True
  if _as_int(nav_phase) in (PHASE_HIGHWAY_PREPARE, PHASE_HIGHWAY_COMMIT):
    return True
  if _as_int(nav_maneuver_type) in (MANEUVER_EXIT, MANEUVER_MERGE):
    return True
  return False


def _nav_led_approach(*, iqlink_on: bool, nav_send_turn: bool, nav_phase: int,
                      nav_turn_dist_m: float, nav_send_lc: bool) -> bool:
  """IQ-link nav turn: longitudinal approach without a blinker (toast window)."""
  if not iqlink_on or not nav_send_turn or nav_send_lc:
    return False
  if _as_int(nav_phase) != PHASE_TURN_ACTIVE:
    return False
  return 0.0 < float(nav_turn_dist_m) <= TURN_DESIRE_WINDOW_M


def _nav_dir_left(nav_maneuver_dir, nav_phase_dir) -> bool:
  return _as_int(nav_maneuver_dir) == TURN_LEFT or _as_int(nav_phase_dir) == NAV_LEFT


def _nav_dir_right(nav_maneuver_dir, nav_phase_dir) -> bool:
  return _as_int(nav_maneuver_dir) == TURN_RIGHT or _as_int(nav_phase_dir) == NAV_RIGHT


def _nav_near_matching_turn(*, left: bool, right: bool, nav_maneuver_type,
                            nav_maneuver_dir, nav_phase_dir, nav_turn_dist_m: float,
                            nav_send_turn: bool = False, nav_phase: int = 0) -> bool:
  if not (0.0 < float(nav_turn_dist_m) <= NAV_NEAR_TURN_M):
    return False
  nav_active_turn = bool(nav_send_turn) and _as_int(nav_phase) == PHASE_TURN_ACTIVE
  if nav_active_turn:
    if left and _nav_dir_left(nav_maneuver_dir, nav_phase_dir):
      return True
    if right and _nav_dir_right(nav_maneuver_dir, nav_phase_dir):
      return True
    return False
  if _as_int(nav_maneuver_type) not in (MANEUVER_TURN, MANEUVER_ROUNDABOUT):
    return False
  if left and _nav_dir_left(nav_maneuver_dir, nav_phase_dir):
    return True
  if right and _nav_dir_right(nav_maneuver_dir, nav_phase_dir):
    return True
  return False


class UrbanTurnPrep:
  def __init__(self, params=None):
    self._params = params
    self._turn_planning_on = True
    self._gate_mps = DEFAULT_TURN_GATE_MPS
    self._refresh_tick = 0
    self.stage = STAGE_OFF

  def _params_store(self):
    if self._params is not None:
      return self._params
    try:
      self._params = Params()
    except Exception:
      self._params = None
    return self._params

  def read_params(self) -> None:
    store = self._params_store()
    if store is None:
      return
    stored = store.get("IQLaneTurnDesire")
    if stored is None:
      self._turn_planning_on = True
    else:
      self._turn_planning_on = bool(store.get_bool("IQLaneTurnDesire"))
    requested = _mph_param_to_mps(store.get("IQLaneTurnValue", return_default=True))
    self._gate_mps = min(TURN_TRIGGER_MPS, max(requested, 5.0 * CV.MPH_TO_MS))

  def _maybe_refresh_params(self) -> None:
    if self._refresh_tick % 50 == 0:
      self.read_params()
    self._refresh_tick += 1

  def reset(self) -> None:
    self.stage = STAGE_OFF

  @property
  def gate_mps(self) -> float:
    return self._gate_mps

  def _approach_target(self) -> float:
    return max(self._gate_mps - APPROACH_BELOW_GATE_MS, TURN_IN_MS)

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
    nav_phase: int = 0,
    nav_maneuver_type: int = 0,
    nav_maneuver_dir: int = 0,
    nav_phase_dir: int = 0,
    nav_turn_dist_m: float = 0.0,
    nav_send_lc: bool = False,
    nav_send_turn: bool = False,
    iqlink_on: bool = False,
  ) -> float | None:
    self._maybe_refresh_params()

    if not enabled or gas_pressed or not self._turn_planning_on:
      self.reset()
      return None
    nav_led = _nav_led_approach(
      iqlink_on=iqlink_on,
      nav_send_turn=nav_send_turn,
      nav_phase=nav_phase,
      nav_turn_dist_m=nav_turn_dist_m,
      nav_send_lc=nav_send_lc,
    )
    if not _one_blinker(left_blinker, right_blinker) and not nav_led:
      self.reset()
      return None
    if _as_int(lane_change_state) in (LC_STARTING, LC_FINISHING):
      self.reset()
      return None
    if _nav_is_lane_change(nav_phase, nav_maneuver_type, nav_send_lc):
      self.reset()
      return None
    if posted_limit_ms >= HIGHWAY_LIMIT_MS:
      self.reset()
      return None
    if v_ego > URBAN_V_MAX_MS:
      self.reset()
      return None

    lat_m = _path_lateral_m(path_x, path_y)
    nav_left = nav_led and _nav_dir_left(nav_maneuver_dir, nav_phase_dir)
    nav_right = nav_led and _nav_dir_right(nav_maneuver_dir, nav_phase_dir)
    turning = (
      _path_matches_blinker(lat_m, left_blinker, right_blinker)
      or _steer_into_blinker(steering_angle_deg, left_blinker, right_blinker)
      or _nav_near_matching_turn(
        left=left_blinker or nav_left,
        right=right_blinker or nav_right,
        nav_maneuver_type=nav_maneuver_type,
        nav_maneuver_dir=nav_maneuver_dir,
        nav_phase_dir=nav_phase_dir,
        nav_turn_dist_m=nav_turn_dist_m,
        nav_send_turn=nav_send_turn,
        nav_phase=nav_phase,
      )
    )
    below_gate = v_ego < self._gate_mps
    finished = (
      self.stage == STAGE_TURN_IN
      and _steer_straight(steering_angle_deg)
      and _path_straight(lat_m)
    )
    if finished:
      self.reset()
      return None

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

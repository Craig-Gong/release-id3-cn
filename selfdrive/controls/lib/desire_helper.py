"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""

from __future__ import annotations

from cereal import car, custom, log

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.iqpilot.selfdrive.controls.lib.helpers.lane_change import (
  IQLaneSwapController,
  AutoLaneChangeMode,
  NavExitLaneChangeController,
  NavForkLaneChangeController,
)
from openpilot.iqpilot.selfdrive.controls.lib.helpers.lane_turn import IQNavTurnController, TURN_TRIGGER_MPS
from openpilot.iqpilot.selfdrive.controls.lib.helpers.nav_auto_blinker import nav_auto_blinker_blocks_lc
from openpilot.iqpilot.selfdrive.controls.lib.helpers.turn_prep import eval_nav_turn_desire

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = custom.IQTurnSignalDirection
NavManeuverPhase = custom.IQNavState.ManeuverPhase

# Lane-change FSM is for highway swaps. Below the turn-desire gate, blinker means
# intersection turn — keep LC cleared so it cannot starve turnLeft/turnRight.
LANE_CHANGE_SPEED_MIN = TURN_TRIGGER_MPS
LANE_CHANGE_TIME_MAX = 10.0
TURN_DESIRE_STOP_HOLD_TIME = 4.8
TURN_DESIRE_STOP_GAP_TIME = 1.0
TURN_DESIRE_STOP_CYCLE_TIME = TURN_DESIRE_STOP_HOLD_TIME + TURN_DESIRE_STOP_GAP_TIME
TURN_DESIRE_STOP_SPEED_EPS = 0.1


_LANE_CHANGE_DESIRES = {
  (LaneChangeDirection.none, LaneChangeState.off): log.Desire.none,
  (LaneChangeDirection.none, LaneChangeState.preLaneChange): log.Desire.none,
  (LaneChangeDirection.none, LaneChangeState.laneChangeStarting): log.Desire.none,
  (LaneChangeDirection.none, LaneChangeState.laneChangeFinishing): log.Desire.none,
  (LaneChangeDirection.left, LaneChangeState.off): log.Desire.none,
  (LaneChangeDirection.left, LaneChangeState.preLaneChange): log.Desire.keepLeft,
  (LaneChangeDirection.left, LaneChangeState.laneChangeStarting): log.Desire.laneChangeLeft,
  (LaneChangeDirection.left, LaneChangeState.laneChangeFinishing): log.Desire.laneChangeLeft,
  (LaneChangeDirection.right, LaneChangeState.off): log.Desire.none,
  (LaneChangeDirection.right, LaneChangeState.preLaneChange): log.Desire.keepRight,
  (LaneChangeDirection.right, LaneChangeState.laneChangeStarting): log.Desire.laneChangeRight,
  (LaneChangeDirection.right, LaneChangeState.laneChangeFinishing): log.Desire.laneChangeRight,
}

_TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}

_STOP_CYCLING_TURN_DESIRES = {
  log.Desire.turnLeft,
  log.Desire.turnRight,
}


def turn_desire(turn_direction) -> log.Desire:
  return _TURN_DESIRES[getattr(turn_direction, "raw", turn_direction)]


def _direction_from_blinkers(carstate) -> int:
  if carstate.leftBlinker:
    return LaneChangeDirection.left
  if carstate.rightBlinker:
    return LaneChangeDirection.right
  return LaneChangeDirection.none


def _steering_nudge_matches(carstate, direction: int) -> bool:
  if not carstate.steeringPressed:
    return False
  return (
    (direction == LaneChangeDirection.left and carstate.steeringTorque > 0) or
    (direction == LaneChangeDirection.right and carstate.steeringTorque < 0)
  )


def _blindspot_matches(carstate, direction: int) -> bool:
  return (
    (direction == LaneChangeDirection.left and carstate.leftBlindspot) or
    (direction == LaneChangeDirection.right and carstate.rightBlindspot)
  )


def _read_enable_bsm() -> bool:
  try:
    with car.CarParams.from_bytes(Params().get("CarParams")) as cp:
      return bool(cp.enableBsm)
  except Exception:
    return False


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.prev_nav_exit_active = False
    self.prev_nav_fork_active = False
    self.desire = log.Desire.none

    self.alc = IQLaneSwapController(self)
    self.lane_turn_controller = IQNavTurnController(self)
    self.nav_exit = NavExitLaneChangeController(_read_enable_bsm())
    self.nav_fork = NavForkLaneChangeController(_read_enable_bsm())
    self._fork_lc_engaged = False
    self.lane_turn_direction = TurnDirection.none
    self.nav_turn_direction = TurnDirection.none
    self.turn_desire_stop_timer = 0.0
    self.turn_desire_stop_active = False

  @staticmethod
  def get_lane_change_direction(carstate):
    return _direction_from_blinkers(carstate)

  @staticmethod
  def _nav_turn_desire(nav_state, carstate, suppress_highway_blinker: bool = False):
    if nav_state is None or not getattr(nav_state, "active", False):
      return TurnDirection.none
    if getattr(nav_state, "maneuverPhase", NavManeuverPhase.none) != NavManeuverPhase.turnActive:
      return TurnDirection.none
    if not getattr(nav_state, "shouldSendTurnDesire", False):
      return TurnDirection.none
    direction = getattr(nav_state, "turnDesireDirection", TurnDirection.none)
    dir_raw = getattr(direction, "raw", direction)
    if dir_raw in (TurnDirection.none, 0):
      return TurnDirection.none
    turn_dist = float(getattr(nav_state, "nextManeuverDistance", 0.0) or 0.0)
    allowed = eval_nav_turn_desire(
      direction_raw=int(dir_raw),
      turn_dist_m=turn_dist,
      v_ego_mps=float(carstate.vEgo),
      left_blinker=bool(carstate.leftBlinker),
      right_blinker=bool(carstate.rightBlinker),
      left_blindspot=bool(getattr(carstate, "leftBlindspot", False)),
      right_blindspot=bool(getattr(carstate, "rightBlindspot", False)),
      suppress_highway_blinker=suppress_highway_blinker,
    )
    if allowed:
      return direction
    return TurnDirection.none

  def _clear_lane_change(self) -> None:
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none

  def _refresh_turn_overrides(self, carstate, nav_state) -> bool:
    speed_mps = carstate.vEgo
    self.lane_turn_controller.update_params()
    self.lane_turn_controller.update_lane_turn(
      blindspot_left=carstate.leftBlindspot,
      blindspot_right=carstate.rightBlindspot,
      left_blinker=carstate.leftBlinker,
      right_blinker=carstate.rightBlinker,
      v_ego=speed_mps,
    )
    self.lane_turn_direction = self.lane_turn_controller.get_turn_direction()

    self.nav_exit.update_params()
    self.nav_exit.update(nav_state, carstate)
    self.nav_fork.update_params()
    self.nav_fork.update(nav_state, carstate)
    self.nav_fork.note_lane_change_state(self.lane_change_state)
    suppress_fork_blinker = self.nav_fork.blocks_nav_turn_blinker()
    self.nav_turn_direction = self._nav_turn_desire(
      nav_state, carstate, suppress_highway_blinker=suppress_fork_blinker,
    )
    return bool(self.nav_exit.active or self.nav_fork.active)

  def _reset_required(self, lateral_active: bool, nav_lc_active: bool) -> bool:
    timed_out = self.lane_change_timer > LANE_CHANGE_TIME_MAX
    feature_disabled = (
      self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF and not nav_lc_active
    )
    return (not lateral_active) or timed_out or feature_disabled

  def _begin_from_idle(self, one_blinker: bool, nav_lc_active: bool, below_speed: bool) -> None:
    if below_speed:
      return
    if one_blinker and not self.prev_one_blinker:
      self.lane_change_state = LaneChangeState.preLaneChange
      self.lane_change_direction = _direction_from_blinkers(self._last_carstate)
      self.lane_change_ll_prob = 1.0
      self._fork_lc_engaged = False
      return
    nav_fork_active = self.nav_fork.active
    nav_exit_active = self.nav_exit.active
    if nav_fork_active and not self.prev_nav_fork_active:
      self.lane_change_state = LaneChangeState.preLaneChange
      self.lane_change_direction = self.nav_fork.direction
      self.lane_change_ll_prob = 1.0
      self._fork_lc_engaged = True
    elif nav_exit_active and not self.prev_nav_exit_active:
      self.lane_change_state = LaneChangeState.preLaneChange
      self.lane_change_direction = self.nav_exit.direction
      self.lane_change_ll_prob = 1.0
      self._fork_lc_engaged = False

  def _refresh_requested_direction(self, one_blinker: bool, nav_lc_active: bool) -> None:
    if one_blinker:
      self.lane_change_direction = _direction_from_blinkers(self._last_carstate)
    elif self.nav_fork.active:
      self.lane_change_direction = self.nav_fork.direction
    elif self.nav_exit.active:
      self.lane_change_direction = self.nav_exit.direction

  def _step_pre_lane_change(self, one_blinker: bool, nav_lc_active: bool, below_speed: bool) -> None:
    self._refresh_requested_direction(one_blinker, nav_lc_active)
    blindspot_detected = _blindspot_matches(self._last_carstate, self.lane_change_direction)
    steering_ready = _steering_nudge_matches(self._last_carstate, self.lane_change_direction)
    nav_auto_start = (
      (self.nav_exit.active and self.nav_exit.auto_allowed) or
      (self.nav_fork.active and self.nav_fork.auto_allowed)
    )

    self.alc.update_lane_change(blindspot_detected=blindspot_detected, brake_pressed=self._last_carstate.brakePressed)
    allowed_to_launch = steering_ready or self.alc.auto_lane_change_allowed or nav_auto_start

    if (not (one_blinker or nav_lc_active)) or below_speed:
      self._clear_lane_change()
      self._fork_lc_engaged = False
    elif allowed_to_launch and not blindspot_detected:
      self.lane_change_state = LaneChangeState.laneChangeStarting

  def _step_lane_change_starting(self, lane_change_prob: float) -> None:
    self.lane_change_ll_prob = max(self.lane_change_ll_prob - (2.0 * DT_MDL), 0.0)
    if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
      self.lane_change_state = LaneChangeState.laneChangeFinishing

  def _step_lane_change_finishing(self, one_blinker: bool) -> None:
    self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
    if self.lane_change_ll_prob <= 0.99:
      return
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_state = LaneChangeState.preLaneChange if one_blinker else LaneChangeState.off

  def _advance_lane_change_machine(self, one_blinker: bool, nav_lc_active: bool, below_speed: bool, lane_change_prob: float) -> None:
    if self.lane_change_state == LaneChangeState.off:
      self._begin_from_idle(one_blinker, nav_lc_active, below_speed)
      return
    if self.lane_change_state == LaneChangeState.preLaneChange:
      self._step_pre_lane_change(one_blinker, nav_lc_active, below_speed)
      return
    if self.lane_change_state == LaneChangeState.laneChangeStarting:
      self._step_lane_change_starting(lane_change_prob)
      return
    if self.lane_change_state == LaneChangeState.laneChangeFinishing:
      self._step_lane_change_finishing(one_blinker)

  def _update_timer(self) -> None:
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

  def _clear_turn_desire_stop_cycle(self) -> None:
    self.turn_desire_stop_timer = 0.0
    self.turn_desire_stop_active = False

  def _is_standstill(self) -> bool:
    return bool(getattr(self._last_carstate, "standstill", False) or self._last_carstate.vEgo <= TURN_DESIRE_STOP_SPEED_EPS)

  def _cycle_turn_desire_when_stopped(self, desired_output: log.Desire) -> log.Desire:
    if desired_output not in _STOP_CYCLING_TURN_DESIRES:
      self._clear_turn_desire_stop_cycle()
      return desired_output

    if not self._is_standstill():
      self._clear_turn_desire_stop_cycle()
      return desired_output

    if not self.turn_desire_stop_active:
      self.turn_desire_stop_active = True
      self.turn_desire_stop_timer = 0.0

    cycle_phase = self.turn_desire_stop_timer % TURN_DESIRE_STOP_CYCLE_TIME
    self.turn_desire_stop_timer += DT_MDL
    if cycle_phase >= TURN_DESIRE_STOP_HOLD_TIME:
      return log.Desire.none
    return desired_output

  def _pick_desire_output(self) -> None:
    desired_output = log.Desire.none
    if self.nav_turn_direction != TurnDirection.none:
      desired_output = turn_desire(self.nav_turn_direction)
    elif self.lane_turn_direction != TurnDirection.none:
      desired_output = turn_desire(self.lane_turn_direction)
    else:
      desired_output = _LANE_CHANGE_DESIRES[(self.lane_change_direction, self.lane_change_state)]

    self.desire = self._cycle_turn_desire_when_stopped(desired_output)

  def _apply_keep_pulse(self) -> None:
    """evo/stock: during preLaneChange hold keepLeft/Right, pulse none ~1 Hz so the desire buffer stays fresh."""
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
      return
    if self.lane_change_state != LaneChangeState.preLaneChange:
      self.keep_pulse_timer = 0.0
      return

    self.keep_pulse_timer += DT_MDL
    if self.keep_pulse_timer > 1.0:
      self.keep_pulse_timer = 0.0
    elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
      self.desire = log.Desire.none

  def update(self, carstate, lateral_active, lane_change_prob, nav_state=None, modeldata=None, radar_state=None):
    self._last_carstate = carstate
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    if nav_auto_blinker_blocks_lc(nav_state):
      # EA_02 nav blink feeds back on BM_links; do not treat as driver lane-change intent.
      one_blinker = False
    # Below the turn-desire gate, blinker is an intersection turn — do not run LC.
    below_speed = carstate.vEgo < LANE_CHANGE_SPEED_MIN
    nav_lc_active = self._refresh_turn_overrides(carstate, nav_state)
    fork_lc_busy = self._fork_lc_engaged or self.nav_fork.latched
    turn_owns_blinker = (
      self.lane_turn_direction != TurnDirection.none or
      (self.nav_turn_direction != TurnDirection.none and not fork_lc_busy)
    )

    self.alc.update_params()
    if turn_owns_blinker or self._reset_required(lateral_active, nav_lc_active):
      self._clear_lane_change()
      if turn_owns_blinker:
        self._fork_lc_engaged = False
        self.nav_fork.note_lane_change_state(LaneChangeState.off)
    else:
      self._advance_lane_change_machine(one_blinker, nav_lc_active, below_speed, lane_change_prob)

    self._update_timer()
    self.prev_one_blinker = one_blinker and lateral_active
    self.prev_nav_exit_active = self.nav_exit.active
    self.prev_nav_fork_active = self.nav_fork.active
    self.alc.update_state()
    self._pick_desire_output()
    self._apply_keep_pulse()

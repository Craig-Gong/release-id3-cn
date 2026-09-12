from openpilot.cereal import log, custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.nav.snapshot import read_snapshot
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeController, AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_turn import (
  TURN_DESIRE_COMMIT_YAW_RATE,
  TURN_DESIRE_CYCLE_SPEED_MAX,
  TURN_DESIRE_STOP_CYCLE_S,
  TURN_DESIRE_STOP_HOLD_S,
  eval_nav_turn_desire,
  nav_blinker_matches_turn,
  nav_intersection_turn,
  nav_turn_commit_hold,
  nav_turn_keep_pulse_s,
  snapshot_long_ok,
)
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = custom.ModelDataV2SP.TurnDirection

LANE_CHANGE_SPEED_MIN = 45 * CV.KPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}

_STOP_CYCLING_TURN_DESIRES = {
  log.Desire.turnLeft,
  log.Desire.turnRight,
}


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.alc = AutoLaneChangeController(self)
    self.lane_turn_controller = LaneTurnController(self)
    self.lane_turn_direction = TurnDirection.none
    self.nav_turn_direction = TurnDirection.none
    # IQ release-style: yaw commit + standstill rising-edge cycle for turns.
    self.turn_desire_committed = False
    self.turn_desire_stop_timer = 0.0
    self.turn_desire_stop_active = False
    self.turn_desire_cycle_input = log.Desire.none

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob, left_edge_detected=False, right_edge_detected=False,
             path_x=None, path_y=None):
    self.alc.update_params()
    self.lane_turn_controller.update_params()
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    steer_deg = float(getattr(carstate, "steeringAngleDeg", 0.0) or 0.0)
    yaw_rate = float(getattr(carstate, "yawRate", 0.0) or 0.0)

    force_blinker_turn = False
    tbt_dist = 0.0
    try:
      snap = read_snapshot()
      force_blinker_turn = nav_blinker_matches_turn(
        snap, left_blinker=bool(carstate.leftBlinker), right_blinker=bool(carstate.rightBlinker),
      )
      tbt_dist = float(snap.tbt_dist or 0.0)
    except Exception:
      snap = None

    # Lane turn controller update
    self.lane_turn_controller.update_lane_turn(
      blindspot_left=carstate.leftBlindspot, blindspot_right=carstate.rightBlindspot,
      left_blinker=carstate.leftBlinker, right_blinker=carstate.rightBlinker, v_ego=v_ego,
      path_x=path_x, path_y=path_y, steering_angle_deg=steer_deg,
      force_turn=force_blinker_turn,
    )
    turn_raw = self.lane_turn_controller.get_turn_direction()
    self.nav_turn_direction = self._nav_turn_desire(carstate, snap)
    if self.nav_turn_direction != TurnDirection.none:
      turn_raw = self.nav_turn_direction
    # Nav / blinker intersection turn owns the desire and clears LC.
    # A lane-change path yaws into the next lane; that is not an intersection turn.
    if turn_raw != TurnDirection.none:
      self.lane_turn_direction = turn_raw
      turn_active = True
    elif self.lane_change_state != LaneChangeState.off:
      self.lane_turn_direction = TurnDirection.none
      turn_active = False
    else:
      self.lane_turn_direction = TurnDirection.none
      turn_active = False

    if not turn_active:
      self._clear_turn_desire_cycle()

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
    else:
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not turn_active:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(carstate)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = (((carstate.leftBlindspot or left_edge_detected) and self.lane_change_direction == LaneChangeDirection.left) or
                              ((carstate.rightBlindspot or right_edge_detected) and self.lane_change_direction == LaneChangeDirection.right))

        self.alc.update_lane_change(blindspot_detected, carstate.brakePressed)

        auto_lc = self.alc.auto_lane_change_allowed
        if not one_blinker or turn_active:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
        elif (torque_applied or auto_lc) and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        if lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(carstate)
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none

    self.prev_one_blinker = one_blinker and lateral_active

    if self.lane_turn_direction != TurnDirection.none:
      desired = TURN_DESIRES[self.lane_turn_direction]
      self.desire = self._emit_turn_desire(
        desired,
        v_ego=float(v_ego),
        yaw_rate=yaw_rate,
        nav_active=self.nav_turn_direction != TurnDirection.none,
        tbt_dist_m=tbt_dist,
      )
    else:
      self.desire = log.Desire.none
      if self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.keep_pulse_timer = 0.0
        if self.lane_change_direction == LaneChangeDirection.left:
          self.desire = log.Desire.laneChangeLeft
        elif self.lane_change_direction == LaneChangeDirection.right:
          self.desire = log.Desire.laneChangeRight
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        if self.lane_change_direction == LaneChangeDirection.left:
          self.desire = log.Desire.keepLeft
        elif self.lane_change_direction == LaneChangeDirection.right:
          self.desire = log.Desire.keepRight
        # ~1 Hz keep pulse so the model does not sit on none while waiting
        self.keep_pulse_timer += DT_MDL
        if self.keep_pulse_timer > 1.0:
          self.keep_pulse_timer = 0.0
        elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
          self.desire = log.Desire.none
      else:
        self.keep_pulse_timer = 0.0

    self.alc.update_state()

  def _clear_turn_desire_cycle(self) -> None:
    self.turn_desire_committed = False
    self.turn_desire_stop_timer = 0.0
    self.turn_desire_stop_active = False
    self.turn_desire_cycle_input = log.Desire.none
    self.keep_pulse_timer = 0.0

  def _emit_turn_desire(self, desired, *, v_ego: float, yaw_rate: float, nav_active: bool, tbt_dist_m: float):
    """modeld rising-edge: approach may pulse; near corner / yaw commit → hold; creep → cycle."""
    if desired not in _STOP_CYCLING_TURN_DESIRES:
      self._clear_turn_desire_cycle()
      return desired

    if desired != self.turn_desire_cycle_input:
      # Direction change or fresh turn — allow a new rising edge / cycle.
      self.turn_desire_stop_timer = 0.0
      self.turn_desire_stop_active = False
      self.turn_desire_cycle_input = desired
      self.turn_desire_committed = False
      self.keep_pulse_timer = 0.0

    if abs(yaw_rate) >= TURN_DESIRE_COMMIT_YAW_RATE:
      self.turn_desire_committed = True

    # Yaw-committed: continuous hold (IQ release). Do not clear with stop-cycle.
    if self.turn_desire_committed:
      self.turn_desire_stop_timer = 0.0
      self.turn_desire_stop_active = False
      self.keep_pulse_timer = 0.0
      return desired

    # Creeping / stopped: hold then gap-none so launch still gets a rising edge.
    # Takes priority over distance hold — waiting at ≤50 m must not freeze on one edge.
    if v_ego <= TURN_DESIRE_CYCLE_SPEED_MAX:
      if not self.turn_desire_stop_active:
        self.turn_desire_stop_active = True
        self.turn_desire_stop_timer = 0.0
      cycle_phase = self.turn_desire_stop_timer % TURN_DESIRE_STOP_CYCLE_S
      self.turn_desire_stop_timer += DT_MDL
      if cycle_phase >= TURN_DESIRE_STOP_HOLD_S:
        return log.Desire.none
      return desired

    # Moving + nav near corner: hold (re-assert at the bend).
    if nav_active and nav_turn_commit_hold(turn_dist_m=tbt_dist_m, committed=False):
      self.keep_pulse_timer = 0.0
      return desired

    # Blinker-only while moving: hold (IQ release / matches stalk feel).
    if not nav_active:
      self.keep_pulse_timer = 0.0
      return desired

    # Nav approach (still >50 m): keep-pulse so the edge is not spent far out.
    pulse_s = float(nav_turn_keep_pulse_s(tbt_dist_m))
    self.keep_pulse_timer += DT_MDL
    if self.keep_pulse_timer > pulse_s:
      self.keep_pulse_timer = 0.0
      return desired
    return log.Desire.none

  @staticmethod
  def _nav_turn_desire(carstate, snap=None):
    try:
      if snap is None:
        snap = read_snapshot()
    except Exception:
      return TurnDirection.none
    if not snapshot_long_ok(snap, getattr(carstate, "gearShifter", None)):
      return TurnDirection.none
    if not nav_intersection_turn(snap):
      return TurnDirection.none
    allowed = eval_nav_turn_desire(
      direction=snap.maneuver_dir,
      turn_dist_m=float(snap.tbt_dist),
      v_ego_mps=float(carstate.vEgo),
      left_blinker=bool(carstate.leftBlinker),
      right_blinker=bool(carstate.rightBlinker),
      left_blindspot=bool(getattr(carstate, "leftBlindspot", False)),
      right_blindspot=bool(getattr(carstate, "rightBlindspot", False)),
    )
    if allowed == "left":
      return TurnDirection.turnLeft
    if allowed == "right":
      return TurnDirection.turnRight
    return TurnDirection.none

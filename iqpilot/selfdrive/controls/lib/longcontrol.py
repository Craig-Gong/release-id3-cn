import numpy as np

from iqpilot.selfdrive.controls.lib.helpers.green_follow_lead import (
  FOLLOW_LEAD_START_ACCEL,
  LEAD_QUEUE_M,
  FOLLOW_LEAD_LAUNCH_V_EGO,
)
from iqpilot.cereal import car
from iqpilot.common.realtime import DT_CTRL
from iqpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from iqpilot.common.pid import PIDController
from iqpilot.selfdrive.iqmodeld.config import ModelConstants
from iqpilot.selfdrive.controls.lib.smooth_stops import SmoothStopController

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

# Linear pull-away ramp while in starting (m/s^2 per second). Snappier than PID climbing from ~0.
START_ACCEL_RAMP = 5.0

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, CP_IQ, active, long_control_state, v_ego, should_stop, brake_pressed, cruise_standstill):
  # Gas Interceptor
  cruise_standstill = cruise_standstill and not CP_IQ.enableGasInterceptor

  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      elif CP.startingState:
        long_control_state = LongCtrlState.starting
      else:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
  return long_control_state


class LongControl:
  def __init__(self, CP, CP_IQ):
    self.CP = CP
    self.CP_IQ = CP_IQ
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self.stopping_decel_rate = CP_IQ.stoppingDecelRateOverride or 1.0
    self.smooth = SmoothStopController()

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits, lead_distance=0.0, has_lead=False, gas_override=False):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]
    self.smooth.update()

    if self.smooth.enabled and active and self.long_control_state != LongCtrlState.stopping:
      stop_now = self.smooth.want_hold(should_stop, CS.vEgo, CS.standstill)
    else:
      stop_now = should_stop

    self.long_control_state = long_control_state_trans(
      self.CP, self.CP_IQ, active, self.long_control_state, CS.vEgo, stop_now,
      CS.brakePressed, CS.cruiseState.standstill)

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      self.smooth.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.stopping_decel_rate * DT_CTRL
      self.reset()
      self.smooth.reset()

    elif self.long_control_state == LongCtrlState.starting:
      floor = float(self.CP.startAccel)
      if (has_lead and LEAD_QUEUE_M >= lead_distance > 0.0
          and CS.vEgo < FOLLOW_LEAD_LAUNCH_V_EGO):
        floor = min(floor, FOLLOW_LEAD_START_ACCEL)
      desired = max(floor, float(a_target))
      if self.last_output_accel < floor:
        output_accel = floor
      else:
        output_accel = min(desired, self.last_output_accel + START_ACCEL_RAMP * DT_CTRL)
      self.reset()
      self.smooth.reset()

    else:  # LongCtrlState.pid
      if self.smooth.enabled and active and should_stop:
        output_accel = self.smooth.settle(a_target, CS.vEgo, lead_distance, has_lead, self.last_output_accel)
        self.reset()
      else:
        error = a_target - CS.aEgo
        output_accel = self.pid.update(error, speed=CS.vEgo,
                                       feedforward=a_target,
                                       freeze_integrator=gas_override)
        self.smooth.reset()

    if gas_override:
      output_accel = max(output_accel, 0.0)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel

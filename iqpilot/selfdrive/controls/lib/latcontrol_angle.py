import math

from iqpilot.cereal import log
from iqpilot.common.params import Params
from iqpilot.selfdrive.controls.lib.latcontrol import LatControl
from iqpilot.selfdrive.controls.lib.drive_helpers import clip_curvature

# TODO This is speed dependent
STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees


class LatControlAngle(LatControl):
  def __init__(self, CP, CP_IQ, CI, dt):
    super().__init__(CP, CP_IQ, CI, dt)
    self.sat_check_min_speed = 5.
    self.use_steer_limited_by_safety = CP.brand == "tesla"
    self.curvature_lookahead_enabled = Params().get_bool("IQLateralCurvatureLookahead")
    self.target_curvature_last = 0.0

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay,
             lookahead_curvature=None):
    angle_log = log.ControlsState.LateralAngleState.new_message()

    # the rack has ~70 ms of dead time before the wheel moves (measured on VW MQB), so track the
    # curvature the path will need after lat_delay rather than the one it needs now. controlsd passes
    # lookahead_curvature=None in maneuver mode, which keeps the maneuver report measuring raw response.
    # controlsd only runs clip_curvature on desired_curvature, so the lookahead has to be bounded here
    # or the ISO jerk/accel limits are bypassed on the way to the rack.
    target_curvature = desired_curvature
    if active and self.curvature_lookahead_enabled and lookahead_curvature is not None:
      target_curvature, _ = clip_curvature(CS.vEgo, self.target_curvature_last, lookahead_curvature, params.roll)
    self.target_curvature_last = target_curvature

    if not active:
      angle_log.active = False
      angle_steers_des = float(CS.steeringAngleDeg)
    else:
      angle_log.active = True
      angle_steers_des = math.degrees(VM.get_steer_from_curvature(-target_curvature, CS.vEgo, params.roll))
      angle_steers_des += params.angleOffsetDeg

    if self.use_steer_limited_by_safety:
      # these cars' carcontrollers calculate max lateral accel and jerk, so we can rely on carOutput for saturation
      angle_control_saturated = steer_limited_by_safety
    else:
      # for cars which use a method of limiting torque such as a torque signal (Nissan and Toyota)
      # or relying on EPS (Ford Q3), carOutput does not capture maxing out torque  # TODO: this can be improved
      angle_control_saturated = abs(angle_steers_des - CS.steeringAngleDeg) > STEER_ANGLE_SATURATION_THRESHOLD
    angle_log.saturated = bool(self._check_saturation(angle_control_saturated, CS, False, curvature_limited))
    angle_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    angle_log.steeringAngleDesiredDeg = angle_steers_des
    return 0, float(angle_steers_des), angle_log

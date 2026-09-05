"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import custom

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.turn_prep import (
  _path_lateral_m, _path_straight, _steer_into_blinker,
)

TurnDirection = custom.ModelDataV2SP.TurnDirection

LANE_CHANGE_SPEED_MIN = 45 * CV.KPH_TO_MS


class LaneTurnController:
  def __init__(self, desire_helper):
    self.DH = desire_helper
    self.turn_direction = TurnDirection.none
    self.params = Params()
    self.lane_turn_value = self._lane_turn_value_ms()
    self.param_read_counter = 0
    self.enabled = self.params.get_bool("LaneTurnDesire")

  def _lane_turn_value_ms(self) -> float:
    raw = self.params.get("LaneTurnValue", return_default=True)
    try:
      value = float(raw) * CV.MPH_TO_MS
    except (TypeError, ValueError):
      value = 28.0 * CV.MPH_TO_MS
    return min(float(LANE_CHANGE_SPEED_MIN), value)

  def read_params(self):
    self.enabled = self.params.get_bool("LaneTurnDesire")
    self.lane_turn_value = self._lane_turn_value_ms()

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.read_params()
    self.param_read_counter += 1

  def update_lane_turn(self, blindspot_left: bool, blindspot_right: bool, left_blinker: bool, right_blinker: bool,
                       v_ego: float, path_x=None, path_y=None, steering_angle_deg: float = 0.0) -> None:
    # Turn vs lane-change split is fixed at 45 km/h (same as DesireHelper). Do not let LaneTurnValue open a 40–45 gap.
    # Below 45 + blinker is a turn unless the path is clearly straight (lane change).
    # Ambiguous / missing path stays a turn so intersections are not late.
    below_turn_speed = v_ego < LANE_CHANGE_SPEED_MIN
    left = bool(left_blinker) and not bool(right_blinker) and below_turn_speed and not blindspot_left
    right = bool(right_blinker) and not bool(left_blinker) and below_turn_speed and not blindspot_right
    if not left and not right:
      self.turn_direction = TurnDirection.none
      return

    lat_m = _path_lateral_m(path_x, path_y)
    if lat_m is not None and _path_straight(lat_m) and not _steer_into_blinker(float(steering_angle_deg), left, right):
      self.turn_direction = TurnDirection.none
      return

    if left:
      self.turn_direction = TurnDirection.turnLeft
    else:
      self.turn_direction = TurnDirection.turnRight

  def get_turn_direction(self):
    if not self.enabled:
      return TurnDirection.none
    return self.turn_direction

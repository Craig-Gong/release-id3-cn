"""Vision red / model-stop offset that does not change lead follow gap.

IQTrafficStopOffset (meters, 0..6, default 3): when IQ.Dynamic has a filtered
model stop, no lead, and the plan is holding, brake toward a point this far
short of model.position.x[-1] and hold there.

Does not touch radard / IQCustomStopDistance / STOP_DISTANCE. Skips:
  - offset 0 (off)
  - a tracked or radar lead (keep the 4.0 m follow stop)
  - right blinker (China right-on-red wait)
  - IQ-link nav red (already has trafficLightDistM)
  - plans that still have end velocity (stop-sign go-through)
"""
from iqdbc.car.interfaces import ACCEL_MIN
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.iqpilot.selfdrive.controls.lib.custom_stop_distance import (
  E2E_STOP_HOLD_BUFFER,
  E2E_STOP_HOLD_MAX_V,
  E2E_STOP_MIN_DIST,
  E2E_STOP_PLAN_VEL_THRESHOLD,
  get_sanitize_int_param,
)

TRAFFIC_STOP_OFFSET_PARAM = "IQTrafficStopOffset"
MIN_OFFSET_M = 0
MAX_OFFSET_M = 6
DEFAULT_OFFSET_M = 3


class TrafficStopOffset:
  def __init__(self, params: Params | None = None):
    self.params = params if params is not None else Params()
    self.frame = 0
    self.distance = float(DEFAULT_OFFSET_M)
    self.read_params()

  def read_params(self) -> None:
    try:
      stored = get_sanitize_int_param(
        TRAFFIC_STOP_OFFSET_PARAM, MIN_OFFSET_M, MAX_OFFSET_M, self.params)
      self.distance = float(stored)
    except (TypeError, ValueError, UnknownKeyName):
      self.distance = float(DEFAULT_OFFSET_M)

  def update(self) -> None:
    if self.frame % int(3 / DT_MDL) == 0:
      self.read_params()
    self.frame += 1

  def adjust(self, a_target: float, should_stop: bool, v_ego: float, model_msg,
             *, stop_light: bool, has_lead: bool, right_blinker: bool,
             nav_red: bool) -> tuple[float, bool]:
    if self.distance <= 0. or not stop_light or has_lead or right_blinker or nav_red:
      return a_target, should_stop

    x = model_msg.position.x
    v = model_msg.velocity.x
    if len(x) != ModelConstants.IDX_N or len(v) != ModelConstants.IDX_N:
      return a_target, should_stop

    # Same hold test as Custom Stop: proceeding plans (stop signs) are left alone
    # so an early stop does not look "done" and roll through.
    if float(v[-1]) > E2E_STOP_PLAN_VEL_THRESHOLD:
      return a_target, should_stop

    stop_distance = float(x[-1])

    if v_ego < E2E_STOP_HOLD_MAX_V:
      if stop_distance <= self.distance + E2E_STOP_HOLD_BUFFER:
        should_stop = True
    else:
      adjusted_distance = max(stop_distance - self.distance, E2E_STOP_MIN_DIST)
      a_required = max(-(v_ego ** 2) / (2 * adjusted_distance), ACCEL_MIN)
      if a_required < a_target:
        a_target = float(a_required)

    return a_target, should_stop

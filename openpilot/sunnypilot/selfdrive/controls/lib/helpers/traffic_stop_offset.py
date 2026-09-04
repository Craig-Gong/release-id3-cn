"""Vision red / model-stop offset that does not change lead follow gap.

TrafficStopOffset (meters, 0..6 in 0.5 steps, default 3): when the model
wants to stop, no lead, and the plan is holding, brake toward a point this
far short of model.position.x[-1] and hold there.
"""
from opendbc.car.interfaces import ACCEL_MIN
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants

TRAFFIC_STOP_OFFSET_PARAM = "TrafficStopOffset"
MIN_OFFSET_M = 0.0
MAX_OFFSET_M = 6.0
DEFAULT_OFFSET_M = 3.0
OFFSET_STEP_M = 0.5

E2E_STOP_PLAN_VEL_THRESHOLD = 1.0
E2E_STOP_MIN_DIST = 2.0
E2E_STOP_HOLD_MAX_V = 0.5
E2E_STOP_HOLD_BUFFER = 2.0


def _sanitize_offset_m(raw) -> float:
  try:
    value = float(raw)
  except (TypeError, ValueError):
    return DEFAULT_OFFSET_M
  bounded = min(max(value, MIN_OFFSET_M), MAX_OFFSET_M)
  return round(bounded / OFFSET_STEP_M) * OFFSET_STEP_M


class TrafficStopOffset:
  def __init__(self, params: Params | None = None):
    self.params = params if params is not None else Params()
    self.frame = 0
    self.distance = float(DEFAULT_OFFSET_M)
    self.read_params()

  def read_params(self) -> None:
    try:
      stored = self.params.get(TRAFFIC_STOP_OFFSET_PARAM, return_default=True)
      snapped = _sanitize_offset_m(stored if stored is not None else DEFAULT_OFFSET_M)
      if stored is not None and snapped != float(stored):
        self.params.put(TRAFFIC_STOP_OFFSET_PARAM, snapped)
      self.distance = snapped
    except (TypeError, ValueError, UnknownKeyName):
      self.distance = float(DEFAULT_OFFSET_M)

  def update(self) -> None:
    if self.frame % int(3 / DT_MDL) == 0:
      self.read_params()
    self.frame += 1

  def adjust(self, a_target: float, should_stop: bool, v_ego: float, model_msg,
             *, stop_light: bool, has_lead: bool, right_blinker: bool) -> tuple[float, bool]:
    if self.distance <= 0. or not stop_light or has_lead or right_blinker:
      return a_target, should_stop

    x = model_msg.position.x
    v = model_msg.velocity.x
    if len(x) != ModelConstants.IDX_N or len(v) != ModelConstants.IDX_N:
      return a_target, should_stop

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

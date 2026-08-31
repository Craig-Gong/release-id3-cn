import math

from iqpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_curvature_from_plan
from iqpilot.selfdrive.iqmodeld.config import ModelConstants


LOOKAHEAD_SECONDS = 0.20


def get_lookahead_curvature(model_v2, v_ego: float, lat_delay: float) -> float | None:
  try:
    yaws = model_v2.orientation.z
    yaw_rates = model_v2.orientationRate.z
    if len(yaws) < CONTROL_N or len(yaw_rates) < CONTROL_N:
      return None
    if not all(math.isfinite(value) for value in yaws) or not all(math.isfinite(value) for value in yaw_rates):
      return None
    horizon = max(0.0, lat_delay) + LOOKAHEAD_SECONDS
    return get_curvature_from_plan(yaws, yaw_rates, ModelConstants.T_IDXS, v_ego, horizon)
  except (AttributeError, TypeError, ValueError):
    return None

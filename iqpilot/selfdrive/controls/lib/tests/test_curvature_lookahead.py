from types import SimpleNamespace

import numpy as np

from iqpilot.selfdrive.controls.lib.curvature_lookahead import LOOKAHEAD_SECONDS, get_lookahead_curvature
from iqpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan
from iqpilot.selfdrive.iqmodeld.config import ModelConstants


def test_lookahead_samples_total_delay_horizon():
  yaws = np.square(np.asarray(ModelConstants.T_IDXS)) * 0.02
  yaw_rates = np.asarray(ModelConstants.T_IDXS) * 0.04
  model_v2 = SimpleNamespace(
    orientation=SimpleNamespace(z=yaws.tolist()),
    orientationRate=SimpleNamespace(z=yaw_rates.tolist()),
  )
  lat_delay = 0.3
  expected = get_curvature_from_plan(yaws, yaw_rates, ModelConstants.T_IDXS, 20.0, lat_delay + LOOKAHEAD_SECONDS)
  assert get_lookahead_curvature(model_v2, 20.0, lat_delay) == expected


def test_invalid_trajectory_falls_back_to_none():
  model_v2 = SimpleNamespace(
    orientation=SimpleNamespace(z=[0.0]),
    orientationRate=SimpleNamespace(z=[0.0]),
  )
  assert get_lookahead_curvature(model_v2, 20.0, 0.3) is None

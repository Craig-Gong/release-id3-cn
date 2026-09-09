import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.modeld_v2.constants import Plan
from openpilot.sunnypilot.modeld_v2.model_smoothing import (
  MODEL_SMOOTHING_MAX_TOTAL_SEC,
  dynamic_lat_smooth_extra_seconds,
  lat_smooth_total_seconds,
  plan_y_std_1s,
)


class TestModelSmoothing(OpenpilotTestCase):
  def test_dynamic_extra_ramps_with_y_std(self):
    assert dynamic_lat_smooth_extra_seconds(0.10, 0.15) == 0.0
    assert dynamic_lat_smooth_extra_seconds(0.15, 0.15) == 0.0
    mid = dynamic_lat_smooth_extra_seconds(0.20, 0.15)
    assert 0.07 < mid < 0.08
    assert dynamic_lat_smooth_extra_seconds(0.25, 0.15) == 0.15
    assert dynamic_lat_smooth_extra_seconds(0.40, 0.15) == 0.15
    assert dynamic_lat_smooth_extra_seconds(0.40, 0.0) == 0.0

  def test_plan_y_std_1s_reads_lateral(self):
    plan_stds = np.zeros((1, 33, 15), dtype=np.float32)
    plan_stds[0, 10, Plan.POSITION] = (0.0, 0.22, 0.0)
    assert plan_y_std_1s({"plan_stds": plan_stds}) == 0.22
    assert plan_y_std_1s({}) == 0.0

  def test_total_capped(self):
    plan_stds = np.zeros((1, 33, 15), dtype=np.float32)
    plan_stds[0, 10, Plan.POSITION] = (0.0, 0.30, 0.0)
    total, extra = lat_smooth_total_seconds(0.50, {"plan_stds": plan_stds}, 0.30)
    assert extra == 0.30
    assert total == MODEL_SMOOTHING_MAX_TOTAL_SEC

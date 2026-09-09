"""IQ.Pilot-style dynamic lateral smoothing (plan y_std → extra tau).

Ported from iqmodeld + drive_helpers ModelSmoothing. Lives under modeld_v2 so
stock modeld / eGPU load paths stay untouched; only post-inference curvature
smoothing and lat_delay accounting change.
"""
from __future__ import annotations

import numpy as np

from openpilot.common.params import Params, UnknownKeyName
from openpilot.sunnypilot.modeld_v2.constants import Plan

# When policy plan_stds for 1s-ahead lateral position spikes, temporarily lengthen
# desiredCurvature smoothing so noisy model output does not jerk the wheel.
MODEL_SMOOTHING_STD_LOW = 0.15  # m — below: no extra smoothing
MODEL_SMOOTHING_STD_HIGH = 0.25  # m — at/above: full max_extra_seconds
MODEL_SMOOTHING_MAX_TOTAL_SEC = 0.60  # hard ceiling on base + dynamic lat tau

# Defaults when Params keys are missing (prebuilt libparams) or unset.
# ModelLatSmoothSec is centiseconds (IQ): 15 → 0.15 s max extra.
_DEFAULT_ENABLED = True
_DEFAULT_LAT_SMOOTH_SEC = 15


def dynamic_lat_smooth_extra_seconds(y_std_1s: float, max_extra_seconds: float) -> float:
  if max_extra_seconds <= 0.0:
    return 0.0
  return float(np.interp(y_std_1s, [MODEL_SMOOTHING_STD_LOW, MODEL_SMOOTHING_STD_HIGH], [0.0, max_extra_seconds]))


def plan_y_std_1s(outputs: dict) -> float:
  # plan_stds is (batch, IDX_N, PLAN_WIDTH); index 10 ~= 1s ahead (ModelConstants.T_IDXS),
  # POSITION is (x, y, z) so [1] is lateral (y) std.
  try:
    return float(outputs["plan_stds"][0, 10, Plan.POSITION][1])
  except (KeyError, IndexError, TypeError, ValueError):
    return 0.0


def model_lat_smooth_max_sec(params: Params | None = None) -> float:
  """Max extra lat-smooth seconds from Params (IQ ModelSmoothingEnabled + ModelLatSmoothSec)."""
  if params is None:
    params = Params()

  try:
    enabled = bool(params.get_bool("ModelSmoothingEnabled"))
  except UnknownKeyName:
    enabled = _DEFAULT_ENABLED
  if not enabled:
    return 0.0

  try:
    raw = params.get("ModelLatSmoothSec", return_default=True)
    raw = _DEFAULT_LAT_SMOOTH_SEC if raw is None else int(raw)
  except (ValueError, TypeError, UnknownKeyName):
    raw = _DEFAULT_LAT_SMOOTH_SEC
  return min(max(raw, 0), 30) * 0.01


def lat_smooth_total_seconds(base_lat_smooth: float, outputs: dict, max_extra_seconds: float) -> tuple[float, float]:
  """Returns (total_tau, extra_tau) capped at MODEL_SMOOTHING_MAX_TOTAL_SEC."""
  extra = dynamic_lat_smooth_extra_seconds(plan_y_std_1s(outputs), max_extra_seconds)
  total = min(float(base_lat_smooth) + extra, MODEL_SMOOTHING_MAX_TOTAL_SEC)
  return total, extra

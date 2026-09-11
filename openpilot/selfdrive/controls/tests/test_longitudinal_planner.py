from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  A_CRUISE_MAX_BP, A_EMPTY_ROAD_MAX_VALS, J_CRUISE_VALS,
  get_cruise_accel, get_empty_road_max_accel, get_max_accel,
)


def test_e2e_cruise_accel_respects_jerk_limit():
  v_ego = 15.0
  a_cruise_prev = -1.0
  dt = 0.05

  accel = get_cruise_accel(
    True,
    v_cruise=40.0,
    v_ego=v_ego,
    a_cruise_prev=a_cruise_prev,
    angle_steers=0.0,
    CP=SimpleNamespace(steerRatio=1.0, wheelbase=1.0),
    dt=dt,
    accel_coast=0.0,
    allow_throttle=True,
  )

  jerk_limit = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
  assert accel == pytest.approx(a_cruise_prev + jerk_limit * dt)


def test_empty_road_max_softer_than_stock():
  for v in (0.0, 10.0, 25.0, 40.0):
    assert get_empty_road_max_accel(v) < float(get_max_accel(v))
  assert get_empty_road_max_accel(0.0) == pytest.approx(A_EMPTY_ROAD_MAX_VALS[0])
  assert get_empty_road_max_accel(10.0) == pytest.approx(A_EMPTY_ROAD_MAX_VALS[1])


def test_empty_road_override_caps_e2e_catchup():
  """Blended/e2e used ACCEL_MAX=2.0; empty-road override must still bind."""
  v_ego = 10.0
  cap = get_empty_road_max_accel(v_ego)
  accel = get_cruise_accel(
    True,
    v_cruise=40.0,
    v_ego=v_ego,
    a_cruise_prev=cap,
    angle_steers=0.0,
    CP=SimpleNamespace(steerRatio=1.0, wheelbase=1.0),
    dt=0.05,
    accel_coast=0.0,
    allow_throttle=True,
    max_accel_override=cap,
  )
  assert accel <= cap + 1e-6


def test_empty_road_override_caps_acc_catchup():
  v_ego = 10.0
  cap = get_empty_road_max_accel(v_ego)
  accel = get_cruise_accel(
    False,
    v_cruise=40.0,
    v_ego=v_ego,
    a_cruise_prev=cap,
    angle_steers=0.0,
    CP=SimpleNamespace(steerRatio=1.0, wheelbase=1.0),
    dt=0.05,
    accel_coast=0.0,
    allow_throttle=True,
    max_accel_override=cap,
  )
  assert accel <= cap + 1e-6
  # Without override, stock ceiling is higher.
  stock = get_cruise_accel(
    False,
    v_cruise=40.0,
    v_ego=v_ego,
    a_cruise_prev=float(get_max_accel(v_ego)),
    angle_steers=0.0,
    CP=SimpleNamespace(steerRatio=1.0, wheelbase=1.0),
    dt=0.05,
    accel_coast=0.0,
    allow_throttle=True,
  )
  assert stock > accel

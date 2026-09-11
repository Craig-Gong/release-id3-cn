from openpilot.sunnypilot.nav.protocol import (
  NAV_STOP_MARGIN_M,
  nav_red_accel_cap,
  nav_red_speed_ms,
  nav_stop_margin_m,
)


def test_nav_margin_fixed_ignores_vision_slider():
  assert nav_stop_margin_m(8.5) == NAV_STOP_MARGIN_M == 3.0
  assert nav_stop_margin_m(10.0) == 3.0
  assert nav_stop_margin_m(0.0) == 3.0
  assert nav_stop_margin_m(None) == 3.0


def test_already_inside_margin_is_hard_stop():
  assert nav_red_speed_ms(2.0, 13.9, 3.0) == 0.0
  assert nav_red_speed_ms(0.0, 13.9, 3.0) == 0.0
  assert nav_red_speed_ms(3.0, 13.9, 3.0) == 0.0


def test_far_light_has_approach_speed():
  v = nav_red_speed_ms(80.0, 0.0, 3.0)
  assert v > 10.0  # √(2*2*77) ≈ 17.5


def test_nav_red_accel_only_when_above_curve():
  # 50 km/h ego, far light approach ~17 m/s → needs -2
  assert nav_red_accel_cap(14.0, 10.0, -2.0) == -2.0
  # ego already at/below approach target → do not force brake
  assert nav_red_accel_cap(9.0, 10.0, -2.0) is None
  assert nav_red_accel_cap(10.4, 10.0, -2.0) is None  # within 0.5 slack

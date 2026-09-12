from openpilot.sunnypilot.nav.protocol import (
  NAV_STOP_MARGIN_M,
  NAV_STOP_MARGIN_MAX_M,
  nav_red_accel_cap,
  nav_red_force_stop,
  nav_red_speed_ms,
  nav_stop_margin_m,
)


def test_nav_margin_uses_slider_with_floor_and_cap():
  assert nav_stop_margin_m(None) == NAV_STOP_MARGIN_M == 3.0
  assert nav_stop_margin_m(0.0) == 3.0
  assert nav_stop_margin_m(4.5) == 4.5
  assert nav_stop_margin_m(3.0) == 3.0
  # Vision-only 8–10 m must not pull nav past the cap.
  assert nav_stop_margin_m(8.5) == NAV_STOP_MARGIN_MAX_M == 6.0
  assert nav_stop_margin_m(10.0) == 6.0


def test_already_inside_margin_is_hard_stop():
  assert nav_red_speed_ms(2.0, 13.9, 3.0) == 0.0
  assert nav_red_speed_ms(0.0, 13.9, 3.0) == 0.0
  assert nav_red_speed_ms(3.0, 13.9, 3.0) == 0.0
  assert nav_red_speed_ms(4.0, 13.9, 4.5) == 0.0


def test_far_light_has_approach_speed():
  v = nav_red_speed_ms(80.0, 0.0, 3.0)
  assert v > 10.0  # √(2*2*77) ≈ 17.5


def test_kinematic_brake_scales_with_remaining():
  # Far + slow: near-zero demand (no absurd early -2).
  assert nav_red_accel_cap(5.0, 100.0, 4.5) > -0.3
  # On a ~2 m/s² curve for remaining=50 → v≈14.1, a≈-2.
  a_on = nav_red_accel_cap(14.1, 54.5, 4.5)
  assert -2.3 <= a_on <= -1.7
  # Above the curve: harder than -2.
  assert nav_red_accel_cap(18.0, 54.5, 4.5) < -2.5
  # Past the stop point: hold brake.
  assert nav_red_accel_cap(1.0, 3.0, 4.5) <= -1.5


def test_force_stop_in_final_meters():
  assert nav_red_force_stop(0.4, 4.5, 4.5) is True   # remaining 0
  assert nav_red_force_stop(1.0, 7.5, 4.5) is True   # remaining 3.0 ≤ 3.5 m near zone
  assert nav_red_force_stop(5.0, 20.0, 4.5) is False  # far, still approaching
  assert nav_red_force_stop(2.0, 0.0, 4.5) is True    # no usable light range
  assert nav_red_force_stop(3.0, 8.5, 4.5) is False  # remaining 4.0 > 3.5

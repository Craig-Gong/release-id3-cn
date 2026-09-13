from openpilot.sunnypilot.nav.protocol import (
  NAV_STOP_MARGIN_M,
  NAV_STOP_MARGIN_MAX_M,
  nav_red_accel_cap,
  nav_red_comfort_speed_ms,
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


def test_far_urban_cruise_does_not_force_brake():
  """60 km/h @ 150 m red: comfort curve still above ego → a_cap = 0 (no grind)."""
  v_ego = 60.0 / 3.6  # ≈ 16.67
  light_d = 150.0
  margin = 3.0
  remaining = light_d - margin
  v_comfort = nav_red_comfort_speed_ms(light_d, margin)
  assert v_comfort > v_ego
  assert abs(v_comfort - (2.0 * 1.5 * remaining) ** 0.5) < 1e-6
  assert nav_red_accel_cap(v_ego, light_d, margin) == 0.0
  # Speed ceiling also still above cruise so min() would not pull MAX down.
  assert nav_red_speed_ms(light_d, v_ego, margin) >= v_ego - 1e-6


def test_kinematic_brake_when_above_comfort_curve():
  # remaining=50 → comfort √(2*1.5*50) ≈ 12.25; ego 14.1 must brake.
  a_on = nav_red_accel_cap(14.1, 54.5, 4.5)
  assert a_on <= -1.5
  assert -3.6 <= a_on <= -1.4
  # Clearly above: harder.
  assert nav_red_accel_cap(18.0, 54.5, 4.5) < -2.5
  # Far + already slow: still under comfort → no forced brake.
  assert nav_red_accel_cap(5.0, 100.0, 4.5) == 0.0
  # Past the stop point: hold brake.
  assert nav_red_accel_cap(1.0, 3.0, 4.5) <= -1.5


def test_force_stop_in_final_meters():
  assert nav_red_force_stop(0.4, 4.5, 4.5) is True   # remaining 0
  assert nav_red_force_stop(1.0, 7.5, 4.5) is True   # remaining 3.0 ≤ 3.5 m near zone
  assert nav_red_force_stop(5.0, 20.0, 4.5) is False  # far, still approaching
  assert nav_red_force_stop(2.0, 0.0, 4.5) is True    # no usable light range
  assert nav_red_force_stop(3.0, 8.5, 4.5) is False  # remaining 4.0 > 3.5

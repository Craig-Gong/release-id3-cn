from openpilot.sunnypilot.nav.protocol import (
  NAV_STOP_MARGIN_M,
  NAV_STOP_MARGIN_MAX_M,
  nav_red_accel_cap,
  nav_red_accel_raw,
  nav_red_comfort_speed_ms,
  nav_red_force_stop,
  nav_red_remaining_m,
  nav_red_speed_ms,
  nav_stop_margin_m,
)


def test_nav_margin_uses_slider_with_floor_and_cap():
  assert nav_stop_margin_m(None) == NAV_STOP_MARGIN_M == 3.0
  assert nav_stop_margin_m(0.0) == 3.0
  assert nav_stop_margin_m(4.5) == 4.5
  assert nav_stop_margin_m(3.0) == 3.0
  # Slider and vision share the same 10 m cap (lamp→line bias is separate).
  assert nav_stop_margin_m(8.5) == 8.5
  assert nav_stop_margin_m(10.0) == NAV_STOP_MARGIN_MAX_M == 10.0
  assert nav_stop_margin_m(12.0) == 10.0


def test_already_inside_margin_is_hard_stop():
  # remaining = light − margin − 2 m line bias.
  assert nav_red_speed_ms(2.0, 13.9, 3.0) == 0.0
  assert nav_red_speed_ms(0.0, 13.9, 3.0) == 0.0
  assert nav_red_speed_ms(3.0, 13.9, 3.0) == 0.0
  assert nav_red_speed_ms(4.0, 13.9, 4.5) == 0.0
  assert nav_red_speed_ms(5.0, 13.9, 3.0) == 0.0  # 5−3−2 = 0


def test_far_light_has_approach_speed():
  v = nav_red_speed_ms(80.0, 0.0, 3.0)
  assert v > 10.0  # √(2*2*75) ≈ 17.3


def test_far_urban_cruise_coasts_or_holds():
  """60 km/h @ 150 m: light floor / coast / early main — no grind / cut-in bait."""
  v_ego = 60.0 / 3.6  # ≈ 16.67
  light_d = 150.0
  margin = 3.0
  v_comfort = nav_red_comfort_speed_ms(light_d, margin)
  assert v_comfort > v_ego
  a = nav_red_accel_cap(v_ego, light_d, margin)
  # rem≈145 → a_req≈0.96 → main tracks −a_req (tiers raised FAR/COAST).
  assert a >= -1.10
  assert a <= 0.0
  assert a < 0.0  # never far-hold at 0 while stop_for_light
  # Speed ceiling still above cruise so min() would not pull MAX down.
  assert nav_red_speed_ms(light_d, v_ego, margin) >= v_ego - 1e-6
  # ~145 m remaining → a_req ≈ 0.96 → main ≈ −0.96.
  a145 = nav_red_accel_raw(v_ego, 145.0)
  assert -1.05 <= a145 <= -0.90
  # ~120 m → main tracks −a_req (≈ −1.16).
  a120 = nav_red_accel_raw(v_ego, 120.0)
  assert -1.30 <= a120 <= -1.05
  # ~100 m → main / harder (≈ −1.39).
  a100 = nav_red_accel_raw(v_ego, 100.0)
  assert -1.50 <= a100 <= -1.10


def test_main_and_hard_tiers():
  # remaining=50, v=14.1 → a_req ≈ 2.0 → hard / main floor.
  a_on = nav_red_accel_raw(14.1, 50.0)
  assert a_on <= -1.5
  assert -3.6 <= a_on <= -1.4
  # Clearly urgent.
  assert nav_red_accel_raw(18.0, 50.0) < -2.5
  # Far + already slow: light floor (not 0).
  assert nav_red_accel_raw(5.0, 95.5) == -0.75
  # Past the stop point: hold brake.
  assert nav_red_accel_cap(1.0, 3.0, 4.5) <= -1.5


def test_jerk_slew_avoids_single_frame_kick():
  """Crossing into main/hard must not jump floor → −1.5 in one 50 ms frame."""
  v = 16.67
  # Far light floor / coast.
  a0 = nav_red_accel_cap(v, 160.0, 3.0, prev_a=None)
  assert a0 >= -0.90
  assert a0 < 0.0
  # Suddenly much closer raw would be hard, but slew caps the step.
  raw_close = nav_red_accel_raw(v, 40.0)
  assert raw_close < -1.5
  a1 = nav_red_accel_cap(v, 43.0, 3.0, prev_a=a0, dt=0.05)
  # Max step ≈ 1.25 * 0.05 = 0.0625 — stay near previous.
  assert abs(a1 - a0) <= 0.065 + 1e-6
  # After enough frames, approach raw.
  a = a1
  for _ in range(80):
    a = nav_red_accel_cap(v, 43.0, 3.0, prev_a=a, dt=0.05)
  assert a <= -1.4
  assert abs(a - raw_close) < 0.15


def test_remaining_fuses_radar_bumper():
  # Lead short of light → stop at bumper − gap (line bias does not apply).
  rem = nav_red_remaining_m(100.0, 3.0, lead_d_rel=40.0, stop_gap=3.5)
  assert abs(rem - (40.0 - 3.5)) < 1e-6
  # Lead at/behind light → keep light remaining (margin + 2 m bias).
  rem2 = nav_red_remaining_m(50.0, 3.0, lead_d_rel=49.5, stop_gap=3.5)
  assert abs(rem2 - 45.0) < 1e-6
  # No lead: light − margin − 2.
  assert abs(nav_red_remaining_m(80.0, 4.0) - 74.0) < 1e-6


def test_force_stop_in_final_meters():
  # remaining = light − margin − 2.
  assert nav_red_force_stop(0.4, 4.5, 4.5) is True   # rem −2 → past
  assert nav_red_force_stop(1.0, 7.5, 4.5) is True   # rem 1.0 ≤ 3.5
  assert nav_red_force_stop(5.0, 20.0, 4.5) is False  # rem 13.5, still approaching
  assert nav_red_force_stop(2.0, 0.0, 4.5) is True    # no usable light range
  assert nav_red_force_stop(3.0, 8.5, 4.5) is True    # rem 2.0 ≤ 3.5 (was False pre-bias)
  assert nav_red_force_stop(3.0, 12.0, 4.5) is False  # rem 5.5 > 3.5

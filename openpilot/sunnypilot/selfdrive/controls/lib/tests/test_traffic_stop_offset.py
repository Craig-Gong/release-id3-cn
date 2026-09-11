from types import SimpleNamespace

from opendbc.car.interfaces import ACCEL_MIN
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.traffic_stop_offset import (
  TrafficStopOffset,
  _sanitize_offset_m,
  soft_release_remaining,
)


def _build(distance):
  c = TrafficStopOffset.__new__(TrafficStopOffset)
  c.frame = 0
  c.distance = float(distance)
  c._filtered_stop = None
  c._engaged = False
  return c


def _model_msg(stop_distance, end_velocity):
  x = [0.0] * (ModelConstants.IDX_N - 1) + [stop_distance]
  v = [0.0] * (ModelConstants.IDX_N - 1) + [end_velocity]
  return SimpleNamespace(position=SimpleNamespace(x=x), velocity=SimpleNamespace(x=v))


def _adjust(c, a_target=-0.1, should_stop=False, v_ego=8.0, stop_distance=12.0, end_velocity=0.0,
            stop_light=True, has_lead=False, right_blinker=False, lead_d_rel=None,
            nav_red=False, steering_angle_deg=0.0):
  return c.adjust(
    a_target, should_stop, v_ego, _model_msg(stop_distance, end_velocity),
    stop_light=stop_light, has_lead=has_lead, right_blinker=right_blinker,
    lead_d_rel=lead_d_rel, nav_red=nav_red, steering_angle_deg=steering_angle_deg,
  )


def test_zero_offset_is_a_no_op():
  assert _adjust(_build(0), a_target=-0.2) == (-0.2, False)


def test_lead_short_of_stop_is_a_no_op():
  assert _adjust(_build(3), a_target=-0.2, has_lead=True, lead_d_rel=8.0, stop_distance=20.0) == (-0.2, False)


def test_phantom_lead_past_stop_still_offsets():
  a_target, should_stop = _adjust(
    _build(3), a_target=0.0, v_ego=8.0, stop_distance=4.0, end_velocity=0.5,
    has_lead=True, lead_d_rel=6.0,
  )
  assert a_target < 0.0
  assert a_target >= ACCEL_MIN
  assert should_stop is True


def test_past_intended_stop_hard_stops():
  a_target, should_stop = _adjust(
    _build(10), a_target=0.0, v_ego=3.0, stop_distance=6.0, end_velocity=0.4,
  )
  assert should_stop is True
  assert a_target < 0.0


def test_short_trajectory_still_offsets():
  c = _build(3)
  model = SimpleNamespace(
    position=SimpleNamespace(x=[0.0, 4.0, 8.0, 12.0]),
    velocity=SimpleNamespace(x=[8.0, 4.0, 1.0, 0.0]),
  )
  a_target, should_stop = c.adjust(
    0.0, False, 8.0, model, stop_light=True, has_lead=False, right_blinker=False,
  )
  assert a_target < 0.0
  assert should_stop is False


def test_lead_without_distance_stays_conservative():
  assert _adjust(_build(3), a_target=-0.2, has_lead=True, lead_d_rel=None) == (-0.2, False)


def test_right_blinker_is_a_no_op():
  assert _adjust(_build(3), a_target=-0.2, right_blinker=True) == (-0.2, False)


def test_nav_red_skips_vision_offset():
  c = _build(3)
  a_target, should_stop = c.adjust(
    -0.2, False, 8.0, _model_msg(12.0, 0.0),
    stop_light=True, has_lead=False, right_blinker=False, nav_red=True,
  )
  assert (a_target, should_stop) == (-0.2, False)


def test_no_stop_light_is_a_no_op():
  assert _adjust(_build(3), a_target=-0.2, stop_light=False) == (-0.2, False)


def test_stop_sign_plan_is_untouched():
  assert _adjust(_build(3), a_target=-0.2, end_velocity=5.0) == (-0.2, False)


def test_soft_should_stop_plan_still_offsets():
  a_target, should_stop = _adjust(_build(3), a_target=0.0, v_ego=8.0, stop_distance=12.0, end_velocity=1.5)
  assert a_target < 0.0
  assert should_stop is False


def test_deepens_braking_short_of_model_stop():
  a_target, should_stop = _adjust(_build(3), a_target=0.0, v_ego=8.0, stop_distance=12.0)
  assert a_target < 0.0
  assert a_target >= ACCEL_MIN
  assert should_stop is False


def test_holds_when_already_short_of_offset():
  a_target, should_stop = _adjust(_build(3), a_target=0.0, v_ego=0.2, stop_distance=4.0)
  assert should_stop is True


def test_does_not_hold_when_model_stop_is_still_far():
  a_target, should_stop = _adjust(_build(3), a_target=0.0, v_ego=0.2, stop_distance=12.0)
  assert should_stop is False


def test_sanitize_keeps_half_meter_steps():
  assert _sanitize_offset_m(3) == 3.0
  assert _sanitize_offset_m("3.5") == 3.5
  assert _sanitize_offset_m(3.2) == 3.0
  assert _sanitize_offset_m(3.3) == 3.5
  assert _sanitize_offset_m(-1) == 0.0
  assert _sanitize_offset_m(9) == 9.0
  assert _sanitize_offset_m(12) == 10.0
  assert _sanitize_offset_m("nope") == 3.0


def test_soft_release_far_fades_offset():
  # Far: almost no offset yet
  rem = soft_release_remaining(77.0, 80.0, release_distance=50.0)
  assert rem > 77.0
  assert rem < 80.0
  # Near release: full hard
  assert soft_release_remaining(40.0, 43.0, release_distance=50.0) == 40.0


def test_far_stop_brakes_gentler_than_hard_offset():
  c_far = _build(3)
  a_soft, _ = _adjust(c_far, a_target=0.0, v_ego=14.0, stop_distance=80.0)
  # Bypass soft release by using a near stop with same offset geometry
  c_near = _build(3)
  a_hard, _ = _adjust(c_near, a_target=0.0, v_ego=14.0, stop_distance=12.0)
  # Far soft remaining ≈79.7 → weaker |a| than near remaining=9
  assert a_soft < 0.0
  assert a_hard < a_soft  # more negative when near


def test_steer_blocks_new_entry_then_allows_once_engaged():
  c = _build(3)
  assert _adjust(c, a_target=-0.2, steering_angle_deg=55.0) == (-0.2, False)
  a_target, _ = _adjust(c, a_target=0.0, steering_angle_deg=0.0, stop_distance=12.0)
  assert a_target < 0.0
  # Stay engaged through a later big steer
  a2, _ = _adjust(c, a_target=0.0, steering_angle_deg=60.0, stop_distance=12.0)
  assert a2 < 0.0


def test_noisy_near_jump_is_rate_limited():
  c = _build(3)
  _adjust(c, a_target=0.0, v_ego=0.0, stop_distance=20.0)
  assert c._filtered_stop == 20.0
  _adjust(c, a_target=0.0, v_ego=0.0, stop_distance=18.0)
  # v=0 → max close 0.5 m/frame
  assert c._filtered_stop == 19.5


def test_large_near_replan_snaps():
  c = _build(3)
  _adjust(c, a_target=0.0, v_ego=8.0, stop_distance=40.0)
  _adjust(c, a_target=0.0, v_ego=8.0, stop_distance=20.0)
  assert c._filtered_stop == 20.0

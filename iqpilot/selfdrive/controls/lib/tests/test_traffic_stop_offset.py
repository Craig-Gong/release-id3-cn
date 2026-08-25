from types import SimpleNamespace

from iqdbc.car.interfaces import ACCEL_MIN
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.iqpilot.selfdrive.controls.lib.traffic_stop_offset import (
  TrafficStopOffset,
  _sanitize_offset_m,
)


def _build(distance):
  c = TrafficStopOffset.__new__(TrafficStopOffset)
  c.frame = 0
  c.distance = float(distance)
  return c


def _model_msg(stop_distance, end_velocity):
  x = [0.0] * (ModelConstants.IDX_N - 1) + [stop_distance]
  v = [0.0] * (ModelConstants.IDX_N - 1) + [end_velocity]
  return SimpleNamespace(position=SimpleNamespace(x=x), velocity=SimpleNamespace(x=v))


def _adjust(c, a_target=-0.1, should_stop=False, v_ego=8.0, stop_distance=12.0, end_velocity=0.0,
            stop_light=True, has_lead=False, right_blinker=False, nav_red=False):
  return c.adjust(
    a_target, should_stop, v_ego, _model_msg(stop_distance, end_velocity),
    stop_light=stop_light, has_lead=has_lead, right_blinker=right_blinker, nav_red=nav_red,
  )


def test_zero_offset_is_a_no_op():
  assert _adjust(_build(0), a_target=-0.2) == (-0.2, False)


def test_lead_is_a_no_op():
  assert _adjust(_build(3), a_target=-0.2, has_lead=True) == (-0.2, False)


def test_right_blinker_is_a_no_op():
  assert _adjust(_build(3), a_target=-0.2, right_blinker=True) == (-0.2, False)


def test_nav_red_is_a_no_op():
  assert _adjust(_build(3), a_target=-0.2, nav_red=True) == (-0.2, False)


def test_no_stop_light_is_a_no_op():
  assert _adjust(_build(3), a_target=-0.2, stop_light=False) == (-0.2, False)


def test_stop_sign_plan_is_untouched():
  assert _adjust(_build(3), a_target=-0.2, end_velocity=5.0) == (-0.2, False)


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
  assert _sanitize_offset_m(9) == 6.0
  assert _sanitize_offset_m("nope") == 3.0

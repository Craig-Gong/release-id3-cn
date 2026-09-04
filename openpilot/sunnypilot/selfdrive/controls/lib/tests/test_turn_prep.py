from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.turn_prep import (
  STAGE_APPROACH,
  STAGE_OFF,
  STAGE_TURN_IN,
  TURN_IN_MS,
  UrbanTurnPrep,
)


class _Params:
  def __init__(self, desire=True, turn_value=24.8548):  # ~40 km/h
    self._desire = desire
    self._turn_value = turn_value

  def get(self, key, return_default=False):
    if key == "LaneTurnValue":
      return self._turn_value
    return None

  def get_bool(self, key):
    if key == "LaneTurnDesire":
      return bool(self._desire)
    return False


def _prep(desire=True, turn_value=24.8548) -> UrbanTurnPrep:
  helper = UrbanTurnPrep(params=_Params(desire, turn_value))
  helper.read_params()
  helper._refresh_tick = 1
  return helper


def _update(helper, v_kph, **kwargs):
  defaults = dict(
    enabled=True,
    left_blinker=True,
    right_blinker=False,
    gas_pressed=False,
    steering_angle_deg=0.0,
    posted_limit_ms=0.0,
    lane_change_state=0,
  )
  defaults.update(kwargs)
  return helper.update(v_ego=v_kph * CV.KPH_TO_MS, **defaults)


def test_no_blinker_does_nothing():
  helper = _prep()
  assert _update(helper, 55.0, left_blinker=False, right_blinker=False) is None
  assert helper.stage == STAGE_OFF


def test_approach_from_urban_blinker():
  helper = _prep()
  v = _update(helper, 55.0)
  assert v is not None
  assert helper.stage == STAGE_APPROACH
  assert abs(v - helper._approach_target()) < 1e-6


def test_does_not_enter_at_or_below_gate():
  helper = _prep()
  assert _update(helper, 40.0) is None
  assert helper.stage == STAGE_OFF


def test_turn_in_when_path_matches():
  helper = _prep()
  _update(helper, 55.0)
  xs = [0, 10, 20, 30, 40]
  ys = [0, 1.0, 2.5, 3.0, 3.2]
  v = _update(helper, 35.0, path_x=xs, path_y=ys)
  assert helper.stage == STAGE_TURN_IN
  assert abs(v - TURN_IN_MS) < 1e-6


def test_big_model_skips_turn_in_cap():
  helper = _prep()
  _update(helper, 55.0)
  xs = [0, 10, 20, 30, 40]
  ys = [0, 1.0, 2.5, 3.0, 3.2]
  v = _update(helper, 35.0, path_x=xs, path_y=ys, big=True)
  assert helper.stage == STAGE_TURN_IN
  assert v is None


def test_gas_releases_cap():
  helper = _prep()
  _update(helper, 55.0)
  assert _update(helper, 55.0, gas_pressed=True) is None
  assert helper.stage == STAGE_OFF


def test_disabled_lane_turn():
  helper = _prep(desire=False)
  assert _update(helper, 55.0) is None

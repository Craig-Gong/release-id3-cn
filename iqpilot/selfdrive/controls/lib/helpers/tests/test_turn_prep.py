from openpilot.common.constants import CV
from openpilot.iqpilot.selfdrive.controls.lib.helpers.turn_prep import (
  APPROACH_BELOW_GATE_MS,
  ENTER_ABOVE_GATE_MS,
  HIGHWAY_LIMIT_MS,
  LC_STARTING,
  MANEUVER_TURN,
  PHASE_HIGHWAY_COMMIT,
  PHASE_TURN_ACTIVE,
  STAGE_APPROACH,
  STAGE_OFF,
  STAGE_TURN_IN,
  TURN_IN_MS,
  TURN_LEFT,
  TURN_TRIGGER_MPS,
  URBAN_V_MAX_MS,
  UrbanTurnPrep,
)

G = 40.0 * CV.KPH_TO_MS


class _Params:
  def __init__(self, desire=None, turn_value=None):
    self._desire = desire
    self._turn_value = turn_value

  def get(self, key, return_default=False):
    if key == "IQLaneTurnDesire":
      return self._desire
    if key == "IQLaneTurnValue":
      return self._turn_value
    return None

  def get_bool(self, key):
    if key == "IQLaneTurnDesire":
      return bool(self._desire)
    return False


def _prep(desire=None, turn_value=None) -> UrbanTurnPrep:
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
  assert abs(v - (G - APPROACH_BELOW_GATE_MS)) < 1e-6


def test_does_not_enter_at_or_below_gate():
  helper = _prep()
  assert _update(helper, 40.0) is None
  assert helper.stage == STAGE_OFF
  just_above = (G + ENTER_ABOVE_GATE_MS) / CV.KPH_TO_MS
  assert _update(helper, just_above - 0.2) is None


def test_highway_speed_skipped():
  helper = _prep()
  assert _update(helper, URBAN_V_MAX_MS / CV.KPH_TO_MS + 2.0) is None


def test_posted_highway_limit_skipped():
  helper = _prep()
  assert _update(helper, 55.0, posted_limit_ms=HIGHWAY_LIMIT_MS) is None


def test_gas_releases():
  helper = _prep()
  assert _update(helper, 55.0) is not None
  assert _update(helper, 55.0, gas_pressed=True) is None
  assert helper.stage == STAGE_OFF


def test_blinker_off_releases():
  helper = _prep()
  assert _update(helper, 55.0) is not None
  assert _update(helper, 55.0, left_blinker=False, right_blinker=False) is None


def test_lane_change_in_progress_skipped():
  helper = _prep()
  assert _update(helper, 55.0, lane_change_state=LC_STARTING) is None


def test_nav_highway_commit_skipped():
  helper = _prep()
  assert _update(helper, 55.0, nav_phase=PHASE_HIGHWAY_COMMIT) is None


def test_turn_planning_off_skipped():
  helper = _prep(desire=False)
  assert _update(helper, 55.0) is None


def test_hold_approach_until_path_curves():
  helper = _prep()
  assert _update(helper, 55.0) is not None
  v = _update(helper, 38.0, path_x=[10.0, 20.0, 30.0, 40.0], path_y=[0.0, 0.1, 0.2, 0.2])
  assert helper.stage == STAGE_APPROACH
  assert abs(v - (G - APPROACH_BELOW_GATE_MS)) < 1e-6


def test_turn_in_when_path_matches_blinker():
  helper = _prep()
  assert _update(helper, 55.0) is not None
  v = _update(
    helper, 38.0,
    path_x=[10.0, 20.0, 30.0, 40.0],
    path_y=[0.5, 1.5, 2.8, 3.5],
  )
  assert helper.stage == STAGE_TURN_IN
  assert abs(v - TURN_IN_MS) < 1e-6


def test_turn_in_from_near_nav_turn():
  helper = _prep()
  assert _update(helper, 55.0) is not None
  v = _update(
    helper, 38.0,
    nav_maneuver_type=MANEUVER_TURN,
    nav_maneuver_dir=TURN_LEFT,
    nav_turn_dist_m=50.0,
  )
  assert helper.stage == STAGE_TURN_IN
  assert abs(v - TURN_IN_MS) < 1e-6
  # Unknown path is not "straight" — keep turn-in until the blinker cancels.
  v2 = _update(
    helper, 32.0,
    nav_maneuver_type=MANEUVER_TURN,
    nav_maneuver_dir=TURN_LEFT,
    nav_turn_dist_m=40.0,
  )
  assert v2 == TURN_IN_MS
  helper = _prep()
  assert _update(helper, 55.0) is not None
  v = _update(
    helper, 38.0,
    nav_maneuver_type=MANEUVER_TURN,
    nav_maneuver_dir=TURN_LEFT,
    nav_turn_dist_m=50.0,
  )
  assert helper.stage == STAGE_TURN_IN
  assert abs(v - TURN_IN_MS) < 1e-6


def test_turn_in_releases_when_straight_again():
  helper = _prep()
  assert _update(helper, 55.0) is not None
  assert _update(
    helper, 38.0,
    path_x=[10.0, 20.0, 30.0, 40.0],
    path_y=[0.5, 1.5, 2.8, 3.5],
  ) == TURN_IN_MS
  assert _update(
    helper, 28.0,
    steering_angle_deg=4.0,
    path_x=[10.0, 20.0, 30.0, 40.0],
    path_y=[0.0, 0.1, 0.2, 0.2],
  ) is None
  assert helper.stage == STAGE_OFF


def test_approach_tracks_higher_gate():
  helper = _prep(turn_value=28.0)
  v = _update(helper, 55.0)
  gate = min(TURN_TRIGGER_MPS, 28.0 * CV.MPH_TO_MS)
  assert abs(v - (gate - APPROACH_BELOW_GATE_MS)) < 1e-3


def test_iqlink_nav_approach_without_blinker():
  helper = _prep()
  v = _update(
    helper, 55.0,
    left_blinker=False,
    right_blinker=False,
    iqlink_on=True,
    nav_send_turn=True,
    nav_phase=PHASE_TURN_ACTIVE,
    nav_maneuver_dir=TURN_LEFT,
    nav_turn_dist_m=100.0,
  )
  assert v is not None
  assert helper.stage == STAGE_APPROACH


def test_iqlink_nav_turn_in_without_blinker():
  helper = _prep()
  assert _update(
    helper, 55.0,
    left_blinker=False,
    right_blinker=False,
    iqlink_on=True,
    nav_send_turn=True,
    nav_phase=PHASE_TURN_ACTIVE,
    nav_maneuver_dir=TURN_LEFT,
    nav_turn_dist_m=60.0,
  ) is not None
  v = _update(
    helper, 38.0,
    left_blinker=False,
    right_blinker=False,
    iqlink_on=True,
    nav_send_turn=True,
    nav_phase=PHASE_TURN_ACTIVE,
    nav_maneuver_dir=TURN_LEFT,
    nav_turn_dist_m=50.0,
  )
  assert helper.stage == STAGE_TURN_IN
  assert abs(v - TURN_IN_MS) < 1e-6


def test_iqlink_nav_prep_skipped_when_iqlink_off():
  helper = _prep()
  assert _update(
    helper, 55.0,
    left_blinker=False,
    right_blinker=False,
    iqlink_on=False,
    nav_send_turn=True,
    nav_phase=PHASE_TURN_ACTIVE,
    nav_maneuver_dir=TURN_LEFT,
    nav_turn_dist_m=60.0,
  ) is None

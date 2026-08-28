from openpilot.common.constants import CV
from openpilot.iqpilot.selfdrive.controls.lib.helpers.turn_prep import (
  TURN_LEFT,
  eval_nav_turn_desire,
)


def _cs(v_kph: float, *, left=False, right=False, bsl=False, bsr=False):
  return dict(
    v_ego_mps=v_kph * CV.KPH_TO_MS,
    left_blinker=left,
    right_blinker=right,
    left_blindspot=bsl,
    right_blindspot=bsr,
  )


def test_nav_turn_blocked_far_and_fast():
  d = eval_nav_turn_desire(direction_raw=TURN_LEFT, turn_dist_m=120.0, **_cs(50.0))
  assert d == 0


def test_nav_turn_near_and_slow():
  d = eval_nav_turn_desire(direction_raw=TURN_LEFT, turn_dist_m=50.0, **_cs(40.0))
  assert d == TURN_LEFT


def test_nav_turn_blinker_confirms_when_fast():
  d = eval_nav_turn_desire(direction_raw=TURN_LEFT, turn_dist_m=120.0, **_cs(55.0, left=True))
  assert d == TURN_LEFT


def test_nav_turn_bsm_blocks():
  d = eval_nav_turn_desire(direction_raw=TURN_LEFT, turn_dist_m=50.0, **_cs(40.0, bsl=True))
  assert d == 0

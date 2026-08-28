from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.iqpilot.selfdrive.controls.lib.helpers.nav_lane_guide import NavLaneGuide, _TextHold


class _Params:
  def get_bool(self, key: str) -> bool:
    return True

  def get(self, key: str):
    return None


def _nav(**kw):
  base = dict(
    active=True,
    shouldSendTurnDesire=False,
    shouldSendLaneChangeDesire=False,
    maneuverPhase=0,
    laneRecommend="none",
    roadSpeedLimit=100.0 * CV.KPH_TO_MS,
    secondNextManeuverValid=False,
    secondNextManeuverType="none",
    secondNextManeuverDirection="none",
    secondNextManeuverDistance=0.0,
  )
  base.update(kw)
  return SimpleNamespace(**base)


def test_text_hold_debounce():
  h = _TextHold(hold_s=0.05)
  now = 100.0
  assert h.filter("left", now) == ""
  assert h.filter("left", now + 0.06) == "left"


def test_lane_recommend_hint_after_hold():
  g = NavLaneGuide(_Params())
  g._lane_hold = _TextHold(hold_s=0.0)
  nav = _nav(laneRecommend="left")
  cap = g.update(nav, engaged=True, iqlink_on=True, v_ego=25.0,
                 posted_limit_ms=100 * CV.KPH_TO_MS, slc_enabled=True)
  assert g.hint == "靠左车道"
  assert cap is not None and cap < 25.0


def test_second_next_hint():
  g = NavLaneGuide(_Params())
  nav = _nav(
    secondNextManeuverValid=True,
    secondNextManeuverType="turn",
    secondNextManeuverDirection="left",
    secondNextManeuverDistance=400.0,
  )
  cap = g.update(nav, engaged=True, iqlink_on=True, v_ego=30.0,
                 posted_limit_ms=100 * CV.KPH_TO_MS, slc_enabled=True)
  assert g.hint == "前方左转"
  assert cap is not None and cap < 30.0


def test_skips_when_send_turn():
  g = NavLaneGuide(_Params())
  g._lane_hold = _TextHold(hold_s=0.0)
  nav = _nav(
    shouldSendTurnDesire=True,
    maneuverPhase=2,
    laneRecommend="left",
    secondNextManeuverValid=True,
    secondNextManeuverType="turn",
    secondNextManeuverDirection="left",
    secondNextManeuverDistance=200.0,
  )
  cap = g.update(nav, engaged=True, iqlink_on=True, v_ego=30.0,
                 posted_limit_ms=100 * CV.KPH_TO_MS, slc_enabled=True)
  assert g.hint == ""
  assert cap is None

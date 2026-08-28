from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.iqpilot.selfdrive.controls.lib.helpers.nav_auto_blinker import (
  NavAutoBlinker,
  arm_distance_m,
  DEBOUNCE_FRAMES,
  is_urban_context,
  urban_red_left_hold,
  urban_rtor_red_hold,
)


class _Params:
  def __init__(self, data: dict):
    self._data = data

  def get_bool(self, key: str) -> bool:
    return bool(self._data.get(key, False))


def _nav(**kw):
  base = dict(
    active=True,
    shouldSendTurnDesire=True,
    shouldSendLaneChangeDesire=False,
    maneuverPhase=2,
    turnDesireDirection="left",
    laneChangeDesireDirection="none",
    nextManeuverType="turn",
    nextManeuverDistance=60.0,
    roadSpeedLimit=50.0 * CV.KPH_TO_MS,
    trafficLight="none",
    trafficLightDistM=0.0,
    trafficLightRemainS=0.0,
    leftTurnPending=False,
    rightTurnPending=False,
  )
  base.update(kw)
  return SimpleNamespace(**base)


def _cs(v_kph=40.0, gear="drive", **kw):
  base = dict(
    vEgo=v_kph * CV.KPH_TO_MS,
    leftBlinker=False,
    rightBlinker=False,
    gearShifter=SimpleNamespace(name=gear),
    leftBlindspot=False,
    rightBlindspot=False,
  )
  base.update(kw)
  return SimpleNamespace(**base)


def _run(b, nav, cs):
  p = _Params({"IqlinkExclusive": True})
  for _ in range(DEBOUNCE_FRAMES + 2):
    left, right = b.update(nav, cs, engaged=True, params=p)
  return left, right


def test_arm_distance_clamped():
  assert arm_distance_m(10.0) == 50.0
  assert arm_distance_m(20.0) == 75.0
  assert arm_distance_m(40.0) == 120.0


def test_is_urban_context():
  assert is_urban_context(50 * CV.KPH_TO_MS, 40 * CV.KPH_TO_MS)
  assert not is_urban_context(80 * CV.KPH_TO_MS, 40 * CV.KPH_TO_MS)
  assert not is_urban_context(50 * CV.KPH_TO_MS, 70 * CV.KPH_TO_MS)


def test_debounce_before_blink():
  b = NavAutoBlinker(_Params({"IQNavAutoBlinker": True}))
  b.read_params()
  # 40 km/h → arm_distance_m = 50 m (D_MIN); default 60 m is outside the arm window.
  nav = _nav(nextManeuverDistance=45.0)
  cs = _cs()
  for i in range(DEBOUNCE_FRAMES - 1):
    left, right = b.update(nav, cs, engaged=True, params=_Params({"IqlinkExclusive": True}))
    assert not left and not right
  left, right = b.update(nav, cs, engaged=True, params=_Params({"IqlinkExclusive": True}))
  assert left and not right


def test_highway_fork_blinks():
  b = NavAutoBlinker(_Params({"IQNavAutoBlinker": True}))
  nav = _nav(
    shouldSendTurnDesire=False,
    shouldSendLaneChangeDesire=True,
    maneuverPhase=4,
    turnDesireDirection="none",
    laneChangeDesireDirection="right",
    nextManeuverType="fork",
    nextManeuverDistance=80.0,
    roadSpeedLimit=100.0 * CV.KPH_TO_MS,
  )
  left, right = _run(b, nav, _cs(v_kph=90.0))
  assert not left and right


def test_highway_fork_too_far_no_blink():
  b = NavAutoBlinker(_Params({"IQNavAutoBlinker": True}))
  nav = _nav(
    shouldSendTurnDesire=False,
    shouldSendLaneChangeDesire=True,
    maneuverPhase=4,
    laneChangeDesireDirection="right",
    nextManeuverType="fork",
    nextManeuverDistance=200.0,
    roadSpeedLimit=100.0 * CV.KPH_TO_MS,
  )
  left, right = _run(b, nav, _cs(v_kph=90.0))
  assert not left and not right


def test_too_far_no_blink():
  b = NavAutoBlinker(_Params({"IQNavAutoBlinker": True}))
  nav = _nav(nextManeuverDistance=130.0)
  left, right = _run(b, nav, _cs())
  assert not left and not right


def test_opposite_blinker_blocks():
  b = NavAutoBlinker(_Params({"IQNavAutoBlinker": True}))
  nav = _nav(nextManeuverDistance=45.0)
  cs = _cs()
  cs.rightBlinker = True
  left, right = _run(b, nav, cs)
  assert not left and not right


def test_highway_skips_urban_red_hold():
  b = NavAutoBlinker(_Params({"IQNavAutoBlinker": True}))
  nav = _nav(
    turnDesireDirection="left",
    nextManeuverDistance=80.0,
    roadSpeedLimit=100.0 * CV.KPH_TO_MS,
    trafficLight="red",
    leftTurnPending=True,
  )
  cs = _cs(v_kph=90.0)
  left, right = _run(b, nav, cs)
  assert left and not right


def test_urban_red_left_holds_until_near():
  nav = _nav(
    trafficLight="red",
    leftTurnPending=True,
    nextManeuverDistance=100.0,
  )
  cs = _cs(v_kph=30.0)
  assert urban_red_left_hold(nav, cs, side="left")
  near = _nav(
    trafficLight="red",
    leftTurnPending=True,
    nextManeuverDistance=70.0,
  )
  assert not urban_red_left_hold(near, cs, side="left")
  b = NavAutoBlinker(_Params({"IQNavAutoBlinker": True}))
  left, right = _run(b, nav, cs)
  assert not left and not right


def test_urban_red_left_arms_when_stopped():
  b = NavAutoBlinker(_Params({"IQNavAutoBlinker": True}))
  nav = _nav(
    trafficLight="red",
    leftTurnPending=True,
    nextManeuverDistance=45.0,
  )
  cs = _cs(v_kph=0.5)
  left, right = _run(b, nav, cs)
  assert left and not right


def test_urban_rtor_holds_until_light_close():
  nav = _nav(
    turnDesireDirection="right",
    trafficLight="red",
    rightTurnPending=True,
    nextManeuverDistance=80.0,
    trafficLightDistM=100.0,
  )
  cs = _cs(v_kph=25.0)
  assert urban_rtor_red_hold(nav, cs, side="right")
  b = NavAutoBlinker(_Params({"IQNavAutoBlinker": True}))
  left, right = _run(b, nav, cs)
  assert not left and not right


def test_urban_rtor_blinks_when_light_near():
  b = NavAutoBlinker(_Params({"IQNavAutoBlinker": True}))
  nav = _nav(
    turnDesireDirection="right",
    trafficLight="red",
    rightTurnPending=True,
    nextManeuverDistance=45.0,
    trafficLightDistM=40.0,
  )
  left, right = _run(b, nav, _cs(v_kph=25.0))
  assert not left and right


def test_bsm_blocks_same_side():
  b = NavAutoBlinker(_Params({"IQNavAutoBlinker": True}))
  nav = _nav(nextManeuverDistance=45.0)
  cs = _cs(leftBlindspot=True)
  left, right = _run(b, nav, cs)
  assert not left and not right


def test_blocks_lc_while_nav_turn():
  from types import SimpleNamespace
  from openpilot.iqpilot.selfdrive.controls.lib.helpers.nav_auto_blinker import nav_auto_blinker_blocks_lc

  p = _Params({"IQNavAutoBlinker": True})
  nav = SimpleNamespace(active=True, shouldSendTurnDesire=True, shouldSendLaneChangeDesire=False)
  assert nav_auto_blinker_blocks_lc(nav, p)
  nav2 = SimpleNamespace(active=True, shouldSendTurnDesire=False, shouldSendLaneChangeDesire=False)
  assert not nav_auto_blinker_blocks_lc(nav2, p)

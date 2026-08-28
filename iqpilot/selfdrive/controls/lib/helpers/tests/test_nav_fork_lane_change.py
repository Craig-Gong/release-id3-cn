from types import SimpleNamespace
from unittest.mock import patch

from cereal import log

from openpilot.common.constants import CV
from openpilot.iqpilot.selfdrive.controls.lib.helpers.lane_change import (
  FORK_LC_PROMOTE_M,
  NavForkLaneChangeController,
)


class _FakeParams:
  def __init__(self, data: dict):
    self._data = data

  def get_bool(self, key: str) -> bool:
    return bool(self._data.get(key, False))


def _nav(**kw):
  base = dict(
    active=True,
    shouldSendLaneChangeDesire=True,
    shouldSendTurnDesire=False,
    maneuverPhase=4,
    nextManeuverType=4,
    laneChangeDesireDirection=2,
    laneRecommend="right",
    nextManeuverDistance=FORK_LC_PROMOTE_M + 50.0,
    roadSpeedLimit=100.0 * CV.KPH_TO_MS,
  )
  base.update(kw)
  return SimpleNamespace(**base)


def _cs(v_kph=80.0, **kw):
  base = dict(
    vEgo=v_kph * CV.KPH_TO_MS,
    leftBlindspot=False,
    rightBlindspot=False,
  )
  base.update(kw)
  return SimpleNamespace(**base)


def _ctrl(params: dict, enable_bsm: bool = True) -> NavForkLaneChangeController:
  with patch(
    "openpilot.iqpilot.selfdrive.controls.lib.helpers.lane_change.Params",
    return_value=_FakeParams(params),
  ):
    return NavForkLaneChangeController(enable_bsm)


def _update(ctrl: NavForkLaneChangeController, nav, cs, params: dict):
  with patch(
    "openpilot.iqpilot.selfdrive.controls.lib.helpers.lane_change.Params",
    return_value=_FakeParams(params),
  ):
    ctrl.update_params()
    ctrl.update(nav, cs)


def test_fork_alc_disabled():
  ctrl = _ctrl({"IQNavHighwayAlc": False, "IqlinkEnabled": True})
  _update(ctrl, _nav(), _cs(), {"IQNavHighwayAlc": False, "IqlinkEnabled": True})
  assert not ctrl.active


def test_fork_alc_signal_active():
  params = {"IQNavHighwayAlc": True, "IqlinkEnabled": True}
  ctrl = _ctrl(params)
  _update(ctrl, _nav(), _cs(), params)
  assert ctrl.active
  assert ctrl.direction == log.LaneChangeDirection.right
  assert not ctrl.auto_allowed


def test_fork_alc_direct_auto_allowed():
  params = {
    "IQNavHighwayAlc": True,
    "IQNavHighwayAlcDirect": True,
    "IqlinkEnabled": True,
  }
  ctrl = _ctrl(params)
  _update(ctrl, _nav(), _cs(), params)
  assert ctrl.active
  assert ctrl.auto_allowed


def test_fork_alc_bsm_blocks_direct():
  params = {
    "IQNavHighwayAlc": True,
    "IQNavHighwayAlcDirect": True,
    "IqlinkEnabled": True,
  }
  ctrl = _ctrl(params, enable_bsm=True)
  _update(ctrl, _nav(), _cs(rightBlindspot=True), params)
  assert ctrl.active
  assert not ctrl.auto_allowed


def test_fork_alc_latches_after_signal_drops():
  params = {"IQNavHighwayAlc": True, "IqlinkEnabled": True}
  ctrl = _ctrl(params)
  _update(ctrl, _nav(), _cs(), params)
  assert ctrl.latched
  _update(
    ctrl,
    _nav(shouldSendLaneChangeDesire=False),
    _cs(),
    params,
  )
  assert ctrl.active
  assert ctrl.direction == log.LaneChangeDirection.right


def test_fork_alc_latch_clears_when_lc_off():
  params = {"IQNavHighwayAlc": True, "IqlinkEnabled": True}
  ctrl = _ctrl(params)
  _update(ctrl, _nav(), _cs(), params)
  ctrl.note_lane_change_state(log.LaneChangeState.off)
  _update(ctrl, _nav(shouldSendLaneChangeDesire=False), _cs(), params)
  assert not ctrl.latched
  assert not ctrl.active


def test_fork_alc_blocks_inside_promote_window():
  params = {"IQNavHighwayAlc": True, "IqlinkEnabled": True}
  ctrl = _ctrl(params)
  _update(ctrl, _nav(nextManeuverDistance=100.0), _cs(), params)
  assert not ctrl.active


def test_fork_alc_blocks_below_speed_gate():
  params = {"IQNavHighwayAlc": True, "IqlinkEnabled": True}
  ctrl = _ctrl(params)
  _update(ctrl, _nav(), _cs(v_kph=40.0), params)
  assert not ctrl.active

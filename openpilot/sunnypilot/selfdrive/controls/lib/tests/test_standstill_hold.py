from openpilot.sunnypilot.selfdrive.controls.lib.helpers.standstill_hold import StandstillHold

_STANDSTILL_HOLD_RELEASE_S = 1.0
_DT_MDL = 0.05


def test_hold_blocks_brief_model_go():
  h = StandstillHold()
  should_stop, a_target = h.apply(True, -0.4, 0.0, standstill=True, gas=False, model_stop=True)
  assert should_stop
  assert h.hold

  should_stop, a_target = h.apply(False, 0.8, 0.0, standstill=True, gas=False, model_stop=True)
  assert should_stop
  assert a_target <= 0.0
  assert h.hold


def test_hold_releases_after_stable_go():
  h = StandstillHold()
  h.apply(True, -0.4, 0.0, standstill=True, gas=False, model_stop=True)
  for _ in range(int(_STANDSTILL_HOLD_RELEASE_S / _DT_MDL)):
    should_stop, a_target = h.apply(False, 0.8, 0.0, standstill=True, gas=False, model_stop=False)
  assert should_stop is False
  assert a_target == 0.8
  assert h.hold is False


def test_gas_releases_immediately():
  h = StandstillHold()
  h.apply(True, -0.4, 0.0, standstill=True, gas=False, model_stop=True)
  should_stop, a_target = h.apply(True, -0.4, 0.0, standstill=True, gas=True, model_stop=True)
  assert should_stop is True
  assert a_target == -0.4
  assert h.hold is False


def test_does_not_rearm_on_sticky_model_stop():
  h = StandstillHold()
  h.apply(True, -0.4, 0.0, standstill=True, gas=False, model_stop=True)
  for _ in range(int(_STANDSTILL_HOLD_RELEASE_S / _DT_MDL)):
    h.apply(False, 0.8, 0.0, standstill=True, gas=False, model_stop=False)
  should_stop, a_target = h.apply(False, 0.8, 0.0, standstill=True, gas=False, model_stop=True)
  assert should_stop is False
  assert a_target == 0.8

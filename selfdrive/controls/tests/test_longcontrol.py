from types import SimpleNamespace

from cereal import custom
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState, long_control_state_trans


def _cp(*, starting_state=False, v_ego_starting=0.5):
  return SimpleNamespace(startingState=starting_state, vEgoStarting=v_ego_starting)


def _trans(CP, CP_IQ, active, state, *, v_ego=0.0, should_stop=False, brake_pressed=False, cruise_standstill=False):
  return long_control_state_trans(CP, CP_IQ, active, state, v_ego, should_stop, brake_pressed, cruise_standstill)


class TestLongControlStateTransition:

  def test_stay_stopped(self):
    CP = _cp()
    CP_IQ = custom.IQCarParams.new_message()
    active = True
    current_state = LongCtrlState.stopping
    next_state = _trans(CP, CP_IQ, active, current_state, should_stop=True)
    assert next_state == LongCtrlState.stopping
    next_state = _trans(CP, CP_IQ, active, current_state, brake_pressed=True)
    assert next_state == LongCtrlState.stopping
    next_state = _trans(CP, CP_IQ, active, current_state, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    next_state = _trans(CP, CP_IQ, active, current_state)
    assert next_state == LongCtrlState.pid
    next_state = _trans(CP, CP_IQ, False, current_state)
    assert next_state == LongCtrlState.off

  def test_meb_starting_then_pid(self):
    CP = _cp(starting_state=True, v_ego_starting=0.5)
    CP_IQ = custom.IQCarParams.new_message()
    state = LongCtrlState.stopping
    state = _trans(CP, CP_IQ, True, state, v_ego=0.0)
    assert state == LongCtrlState.starting
    state = _trans(CP, CP_IQ, True, state, v_ego=0.2)
    assert state == LongCtrlState.starting
    state = _trans(CP, CP_IQ, True, state, v_ego=0.6)
    assert state == LongCtrlState.pid


def test_engage():
  CP = _cp()
  CP_IQ = custom.IQCarParams.new_message()
  active = True
  current_state = LongCtrlState.off
  next_state = _trans(CP, CP_IQ, active, current_state, should_stop=True)
  assert next_state == LongCtrlState.stopping
  next_state = _trans(CP, CP_IQ, active, current_state, brake_pressed=True)
  assert next_state == LongCtrlState.stopping
  next_state = _trans(CP, CP_IQ, active, current_state, cruise_standstill=True)
  assert next_state == LongCtrlState.stopping
  next_state = _trans(CP, CP_IQ, active, current_state)
  assert next_state == LongCtrlState.pid


def test_engage_meb_uses_starting():
  CP = _cp(starting_state=True)
  CP_IQ = custom.IQCarParams.new_message()
  next_state = _trans(CP, CP_IQ, True, LongCtrlState.off)
  assert next_state == LongCtrlState.starting

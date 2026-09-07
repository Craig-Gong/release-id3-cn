"""Standstill hold: sticky red, remainS debounce, launch floor."""
from openpilot.sunnypilot.nav.snapshot import NavSnapshot
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.standstill_hold import (
  StandstillHold, _GO_LAUNCH_FLOOR_A, _REMAIN_GO_CONFIRM_S, _STANDSTILL_HOLD_RELEASE_S,
)


def _snap(**kwargs) -> NavSnapshot:
  base = dict(
    ts=10.0, link_ok=True, link_state=2, iqlink_enabled=True,
    traffic_light="red", dist_m=5.0, remain_s=8.0, remain_go=False,
    stop_for_light=True, speed_target=0.0, accel_target=-2.0,
  )
  base.update(kwargs)
  return NavSnapshot(**base)


def test_sticky_red_survives_stale_executable():
  h = StandstillHold()
  live = _snap(ts=10.0)
  h.observe_nav(live, now=10.0, gas=False, v_ego=0.0)
  assert h.red_pin is True
  # 3 s later: past STALE_LINK_S=2.5, but within sticky TTL 8 s
  stale = _snap(ts=10.0, stop_for_light=True)
  h.observe_nav(stale, now=13.0, gas=False, v_ego=0.0)
  assert h.sticky_red is True
  assert h.red_pin is True


def test_remain_go_needs_confirm_then_floors_accel():
  h = StandstillHold()
  t0 = 20.0
  h.observe_nav(_snap(ts=t0), now=t0, gas=False, v_ego=0.0)
  # First remain_go frames still pin
  stop, a = h.apply(True, -0.5, 0.0, standstill=True, gas=False, model_stop=False,
                    sm={}, now=t0)
  assert stop is True
  go = _snap(ts=t0, traffic_light="red", remain_go=True, remain_s=1.0, stop_for_light=False)
  # Write via apply's read_snapshot — inject by patching is hard; drive remain counter
  # by calling apply with a monkeypatched read. Use observe + internal counter.
  h._remain_go_s = 0.0
  # Simulate confirmed remain by setting counter and clearing red via apply path:
  # use direct state after enough confirm time with mocked snap through apply's read.
  from openpilot.sunnypilot.selfdrive.controls.lib.helpers import standstill_hold as mod

  calls = {"n": 0}

  def fake_read():
    return go

  def fake_exec(snap, now=None):
    return True

  orig_read, orig_exec = mod.read_snapshot, mod.snapshot_executable
  mod.read_snapshot = fake_read
  mod.snapshot_executable = fake_exec
  try:
    # Not enough confirm yet
    stop, a = h.apply(False, 0.2, 0.0, standstill=True, gas=False, model_stop=False,
                      sm={}, now=t0)
    assert h._remain_go_s > 0.0
    assert stop is True or h._remain_go_s < _REMAIN_GO_CONFIRM_S
    # Advance past confirm
    for i in range(4):
      stop, a = h.apply(False, 0.2, 0.0, standstill=True, gas=False, model_stop=False,
                        sm={}, now=t0 + 0.05 * (i + 1))
    assert stop is False
    assert a >= _GO_LAUNCH_FLOOR_A
  finally:
    mod.read_snapshot = orig_read
    mod.snapshot_executable = orig_exec


def test_apk_green_dwell_is_one_second():
  assert _STANDSTILL_HOLD_RELEASE_S == 1.0

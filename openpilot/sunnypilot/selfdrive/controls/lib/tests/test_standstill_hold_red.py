"""Standstill hold: sticky red, remainS debounce, head-car green+radar."""
from types import SimpleNamespace

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


def _radar_sm(*, d_rel=None, v_lead=0.0):
  """Minimal sm: radarState present; optional in-queue lead."""
  if d_rel is None:
    lead = SimpleNamespace(present=False, status=False, dRel=0.0, vLead=0.0)
  else:
    lead = SimpleNamespace(present=True, status=True, dRel=float(d_rel), vLead=float(v_lead))
  return {"radarState": SimpleNamespace(leadOne=lead)}


def test_sticky_red_survives_stale_executable():
  h = StandstillHold()
  live = _snap(ts=10.0)
  h.observe_nav(live, now=10.0, gas=False, v_ego=0.0)
  assert h.red_pin is True
  # HMAC / link_ok dropped; sticky TTL still holds the red pin.
  stale = _snap(ts=10.0, link_ok=False, stop_for_light=True)
  h.observe_nav(stale, now=12.0, gas=False, v_ego=0.0)
  assert h.sticky_red is True
  assert h.red_pin is True


def _vision_phantom_sm(*, x=8.0, v=0.0, prob=0.9):
  """Radar clear + vision lead — classic light phantom."""
  sm = _radar_sm()
  sm["modelV2"] = SimpleNamespace(
    leadsV3=[SimpleNamespace(prob=prob, x=[float(x)], v=[float(v)])],
  )
  return sm


def test_vision_phantom_does_not_block_head_green():
  """mmWave clear + vision-only lead still counts as head car on green."""
  from openpilot.sunnypilot.selfdrive.controls.lib.helpers import standstill_hold as mod
  from openpilot.sunnypilot.selfdrive.controls.lib.helpers.green_follow_lead import (
    is_nav_head_car, read_follow_lead, read_nav_queue_lead,
  )

  sm = _vision_phantom_sm()
  assert read_follow_lead(sm).present is True
  assert read_nav_queue_lead(sm).present is False
  assert is_nav_head_car(sm) is True

  h = StandstillHold()
  t0 = 50.0
  green = _snap(
    ts=t0, traffic_light="green", remain_s=0.0, remain_go=False,
    stop_for_light=False, dist_m=20.0,
  )
  orig_read, orig_exec = mod.read_snapshot, mod.snapshot_executable
  mod.read_snapshot = lambda: green
  mod.snapshot_executable = lambda snap, now=None: True
  try:
    h.observe_nav(_snap(ts=t0 - 1.0), now=t0 - 1.0, gas=False, v_ego=0.0)
    for i in range(int(_STANDSTILL_HOLD_RELEASE_S / 0.05) + 2):
      stop, a = h.apply(False, 0.2, 0.0, standstill=True, gas=False, model_stop=False,
                        sm=sm, now=t0 + 0.05 * i)
    assert stop is False
    assert a >= _GO_LAUNCH_FLOOR_A
  finally:
    mod.read_snapshot = orig_read
    mod.snapshot_executable = orig_exec


def test_head_remain_go_on_red_does_not_launch():
  """Head car + remainS==1 while still red must keep the brake."""
  from openpilot.sunnypilot.selfdrive.controls.lib.helpers import standstill_hold as mod

  h = StandstillHold()
  t0 = 20.0
  red_go = _snap(ts=t0, traffic_light="red", remain_go=True, remain_s=1.0, stop_for_light=True)
  sm = _radar_sm()  # mmWave clear, no lead
  orig_read, orig_exec = mod.read_snapshot, mod.snapshot_executable
  mod.read_snapshot = lambda: red_go
  mod.snapshot_executable = lambda snap, now=None: True
  try:
    for i in range(6):
      stop, a = h.apply(True, -0.5, 0.0, standstill=True, gas=False, model_stop=False,
                        sm=sm, now=t0 + 0.05 * i)
    assert stop is True
    assert a <= 0.0
  finally:
    mod.read_snapshot = orig_read
    mod.snapshot_executable = orig_exec


def test_head_green_needs_radar_and_dwell():
  """Head car launches only after green + mmWave clear + ~1 s dwell."""
  from openpilot.sunnypilot.selfdrive.controls.lib.helpers import standstill_hold as mod

  h = StandstillHold()
  t0 = 30.0
  green = _snap(
    ts=t0, traffic_light="green", remain_s=0.0, remain_go=False,
    stop_for_light=False, dist_m=20.0,
  )
  sm = _radar_sm()
  orig_read, orig_exec = mod.read_snapshot, mod.snapshot_executable
  mod.read_snapshot = lambda: green
  mod.snapshot_executable = lambda snap, now=None: True
  try:
    h.observe_nav(_snap(ts=t0 - 1.0), now=t0 - 1.0, gas=False, v_ego=0.0)
    # Early frames still dwell
    stop, a = h.apply(True, -0.5, 0.0, standstill=True, gas=False, model_stop=False,
                      sm=sm, now=t0)
    assert stop is True
    for i in range(int(_STANDSTILL_HOLD_RELEASE_S / 0.05) + 2):
      stop, a = h.apply(False, 0.2, 0.0, standstill=True, gas=False, model_stop=False,
                        sm=sm, now=t0 + 0.05 * (i + 1))
    assert stop is False
    assert a >= _GO_LAUNCH_FLOOR_A
  finally:
    mod.read_snapshot = orig_read
    mod.snapshot_executable = orig_exec


def test_head_green_blocked_without_radar_state():
  """No radarState → head car must not treat nose as clear."""
  from openpilot.sunnypilot.selfdrive.controls.lib.helpers import standstill_hold as mod

  h = StandstillHold()
  t0 = 40.0
  green = _snap(ts=t0, traffic_light="green", stop_for_light=False, remain_go=False)
  orig_read, orig_exec = mod.read_snapshot, mod.snapshot_executable
  mod.read_snapshot = lambda: green
  mod.snapshot_executable = lambda snap, now=None: True
  try:
    for i in range(30):
      stop, a = h.apply(True, -0.5, 0.0, standstill=True, gas=False, model_stop=False,
                        sm={}, now=t0 + 0.05 * i)
    assert stop is True
  finally:
    mod.read_snapshot = orig_read
    mod.snapshot_executable = orig_exec


def test_apk_green_dwell_is_one_second():
  assert _STANDSTILL_HOLD_RELEASE_S == 1.0
  assert _REMAIN_GO_CONFIRM_S == 0.15


def test_nav_go_latch_blocks_vision_hitch_while_creeping():
  """Green release then model shouldStop must not tap the brake under ~2 m/s."""
  from openpilot.sunnypilot.selfdrive.controls.lib.helpers import standstill_hold as mod

  h = StandstillHold()
  green = _snap(
    ts=30.0, traffic_light="green", remain_s=0.0, remain_go=False,
    stop_for_light=False, dist_m=20.0,
  )
  orig_read, orig_exec = mod.read_snapshot, mod.snapshot_executable
  mod.read_snapshot = lambda: green
  mod.snapshot_executable = lambda snap, now=None: True
  try:
    h._nav_go_latched = True
    h.hold_released = True
    h.red_pin = False
    # Still stopped: floor launch, ignore vision should_stop / negative a.
    stop, a = h.apply(True, -1.5, 0.0, standstill=True, gas=False, model_stop=True,
                      sm=_radar_sm(), now=30.0)
    assert stop is False
    assert a >= _GO_LAUNCH_FLOOR_A
    assert h._nav_go_latched is True
    # Creeping past standstill threshold — old code cleared the latch here.
    stop, a = h.apply(True, -1.8, 0.6, standstill=False, gas=False, model_stop=True,
                      sm=_radar_sm(), now=30.1)
    assert stop is False
    assert a >= 0.0
    assert h._nav_go_latched is True
    # Rolling out clears via reset at _RELEASE_V_EGO.
    stop, a = h.apply(False, 0.4, 2.5, standstill=False, gas=False, model_stop=False,
                      sm=_radar_sm(), now=30.2)
    assert h._nav_go_latched is False
  finally:
    mod.read_snapshot = orig_read
    mod.snapshot_executable = orig_exec

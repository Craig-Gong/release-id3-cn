"""Standstill hold: sticky red, remainS debounce, head-car green+radar+camera."""
from contextlib import contextmanager
from types import SimpleNamespace

from openpilot.sunnypilot.nav.snapshot import NavSnapshot
from openpilot.sunnypilot.selfdrive.controls.lib.helpers import standstill_hold as mod
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.standstill_hold import (
  StandstillHold, _CAMERA_GO_S, _GO_LAUNCH_FLOOR_A, _REMAIN_GO_CONFIRM_S,
  _STANDSTILL_HOLD_RELEASE_S, _STICKY_RED_TTL_S,
)

_DT = 0.05
# Frames for camera clear + green dwell, with margin.
_GO_FRAMES = int((_CAMERA_GO_S + _STANDSTILL_HOLD_RELEASE_S) / _DT) + 3


def _snap(**kwargs) -> NavSnapshot:
  base = dict(
    ts=10.0, link_ok=True, link_state=2, iqlink_enabled=True,
    traffic_light="red", dist_m=0.0, remain_s=8.0, remain_go=False,
    stop_for_light=True, speed_target=0.0, accel_target=-2.0, light_ts=10.0,
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


def _vision_phantom_sm(*, x=8.0, v=0.0, prob=0.9):
  """Radar clear + vision lead — classic light phantom."""
  sm = _radar_sm()
  sm["modelV2"] = SimpleNamespace(
    leadsV3=[SimpleNamespace(prob=prob, x=[float(x)], v=[float(v)])],
  )
  return sm


@contextmanager
def _nav(snap, live=True):
  orig_read, orig_exec = mod.read_snapshot, mod.light_executable
  mod.read_snapshot = lambda: snap
  mod.light_executable = lambda s, now=None: live
  try:
    yield
  finally:
    mod.read_snapshot = orig_read
    mod.light_executable = orig_exec


def _run(h, frames, t0, *, sm, model_stop=False, standstill=True, v_ego=0.0, a=0.2, stop=False):
  out = (stop, a)
  for i in range(frames):
    out = h.apply(stop, a, v_ego, standstill=standstill, gas=False, model_stop=model_stop,
                  sm=sm, now=t0 + _DT * i)
  return out


def test_sticky_red_survives_stale_light():
  h = StandstillHold()
  h.observe_nav(_snap(ts=10.0, light_ts=10.0), now=10.0, gas=False, v_ego=0.0)
  assert h.red_pin is True
  # Gaode stopped refreshing (light_ts old): sticky still pins briefly.
  stale = _snap(ts=11.5, light_ts=8.0)
  h.observe_nav(stale, now=11.5, gas=False, v_ego=0.0)
  assert h.sticky_red is True
  assert h.red_pin is True
  h.observe_nav(stale, now=10.0 + _STICKY_RED_TTL_S + 0.1, gas=False, v_ego=0.0)
  assert h.red_pin is False


def test_red_dark_at_end_of_countdown_clears_pin_now():
  """Gaode sends status 0 at red→green: countdown ≤3 s then dark = changed."""
  h = StandstillHold()
  h.observe_nav(_snap(ts=10.0, light_ts=10.0, remain_s=2.0), now=10.0, gas=False, v_ego=0.0)
  assert h.red_pin is True
  dark = _snap(ts=10.2, light_ts=0.0, traffic_light="none", stop_for_light=False, remain_s=0.0)
  h.observe_nav(dark, now=10.2, gas=False, v_ego=0.0)
  assert h.sticky_red is False
  assert h.red_pin is False


def test_red_dark_mid_countdown_keeps_sticky():
  h = StandstillHold()
  h.observe_nav(_snap(ts=10.0, light_ts=10.0, remain_s=25.0), now=10.0, gas=False, v_ego=0.0)
  dark = _snap(ts=10.2, light_ts=0.0, traffic_light="none", stop_for_light=False, remain_s=0.0)
  h.observe_nav(dark, now=10.2, gas=False, v_ego=0.0)
  assert h.red_pin is True


def test_vision_phantom_does_not_block_head_green():
  """mmWave clear + vision-only lead still counts as head car on green."""
  from openpilot.sunnypilot.selfdrive.controls.lib.helpers.green_follow_lead import (
    is_nav_head_car, read_follow_lead, read_nav_queue_lead,
  )

  sm = _vision_phantom_sm()
  assert read_follow_lead(sm).present is True
  assert read_nav_queue_lead(sm).present is False
  assert is_nav_head_car(sm) is True

  h = StandstillHold()
  green = _snap(ts=50.0, traffic_light="green", remain_s=0.0, stop_for_light=False)
  with _nav(green):
    stop, a = _run(h, _GO_FRAMES, 50.0, sm=sm)
  assert stop is False
  assert a >= _GO_LAUNCH_FLOOR_A


def test_head_remain_go_on_red_does_not_launch():
  """Head car + remainS==1 while still red must keep the brake."""
  h = StandstillHold()
  red_go = _snap(ts=20.0, remain_go=True, remain_s=1.0)
  with _nav(red_go):
    stop, a = _run(h, 6, 20.0, sm=_radar_sm(), a=-0.5, stop=True)
  assert stop is True
  assert a <= 0.0


def test_head_green_needs_radar_and_dwell():
  """Head car launches only after green + mmWave clear + camera clear + ~1 s dwell."""
  h = StandstillHold()
  green = _snap(ts=30.0, traffic_light="green", remain_s=0.0, stop_for_light=False)
  with _nav(green):
    stop, _ = _run(h, 3, 30.0, sm=_radar_sm())
    assert stop is True
    stop, a = _run(h, _GO_FRAMES, 30.2, sm=_radar_sm())
  assert stop is False
  assert a >= _GO_LAUNCH_FLOOR_A


def test_green_does_not_launch_while_camera_sees_stop():
  """Wrong-direction / stale green: model still stopping → stay pinned."""
  h = StandstillHold()
  green = _snap(ts=35.0, traffic_light="green", remain_s=0.0, stop_for_light=False)
  with _nav(green):
    stop, a = _run(h, _GO_FRAMES * 2, 35.0, sm=_radar_sm(), model_stop=True)
  assert stop is True
  assert a <= -1.0
  assert h._nav_go_latched is False


def test_head_green_blocked_without_radar_state():
  """No radarState → head car must not treat nose as clear."""
  h = StandstillHold()
  green = _snap(ts=40.0, traffic_light="green", stop_for_light=False)
  with _nav(green):
    stop, _ = _run(h, _GO_FRAMES, 40.0, sm={}, a=-0.5, stop=True)
  assert stop is True


def test_queue_remain_go_needs_camera_clear():
  h = StandstillHold()
  red_go = _snap(ts=45.0, remain_go=True, remain_s=1.0)
  sm = _radar_sm(d_rel=10.0, v_lead=2.0)
  with _nav(red_go):
    stop, _ = _run(h, 10, 45.0, sm=sm, model_stop=True, a=-0.5, stop=True)
    assert stop is True
    assert h._nav_go_latched is False


def test_apk_green_dwell_is_one_second():
  assert _STANDSTILL_HOLD_RELEASE_S == 1.0
  assert _REMAIN_GO_CONFIRM_S == 0.15


def test_nav_go_latch_ignores_one_frame_hitch():
  """Green release then a brief model shouldStop must not tap the brake."""
  h = StandstillHold()
  green = _snap(ts=30.0, traffic_light="green", remain_s=0.0, stop_for_light=False)
  with _nav(green):
    h._nav_go_latched = True
    h.hold_released = True
    stop, a = h.apply(True, -1.5, 0.0, standstill=True, gas=False, model_stop=True,
                      sm=_radar_sm(), now=30.0)
    assert stop is False
    assert a >= _GO_LAUNCH_FLOOR_A
    stop, a = h.apply(True, -1.8, 0.6, standstill=False, gas=False, model_stop=True,
                      sm=_radar_sm(), now=30.05)
    assert stop is False
    assert a >= 0.0
    assert h._nav_go_latched is True
    h.apply(False, 0.4, 2.5, standstill=False, gas=False, model_stop=False,
            sm=_radar_sm(), now=30.1)
    assert h._nav_go_latched is False


def test_sustained_model_stop_cancels_nav_latch():
  h = StandstillHold()
  green = _snap(ts=31.0, traffic_light="green", remain_s=0.0, stop_for_light=False)
  with _nav(green):
    h._nav_go_latched = True
    h.hold_released = True
    stop, a = _run(h, 8, 31.0, sm=_radar_sm(), model_stop=True, standstill=False,
                   v_ego=0.8, a=-1.2, stop=True)
  assert h._nav_go_latched is False
  assert stop is True
  assert a <= -1.2


def test_rolling_red_pin_passes_through_to_camera():
  """Rolling: nav red alone does not brake; upstream camera plan is kept."""
  h = StandstillHold()
  red = _snap(ts=60.0)
  with _nav(red):
    stop, a = h.apply(False, 0.3, 1.2, standstill=False, gas=False, model_stop=False,
                      sm=_radar_sm(), now=60.0)
    assert h.red_pin is True
    assert stop is False
    assert a == 0.3
    stop, a = h.apply(True, -1.4, 1.0, standstill=False, gas=False, model_stop=True,
                      sm=_radar_sm(), now=60.05)
  assert stop is True
  assert a == -1.4


def test_at_rest_red_pin_nails_should_stop():
  """Stopped on red: should_stop + a≤−1 until confirmed green."""
  h = StandstillHold()
  with _nav(_snap(ts=70.0)):
    stop, a = h.apply(True, -0.5, 0.0, standstill=True, gas=False, model_stop=False,
                      sm=_radar_sm(), now=70.0)
  assert h.red_pin is True
  assert stop is True
  assert a <= -1.0


def test_lead_departed_with_camera_stop_does_not_launch():
  h = StandstillHold()
  off = _snap(ts=75.0, iqlink_enabled=False, link_ok=False, traffic_light="none", stop_for_light=False)
  orig = mod.radar_lead_departed
  mod.radar_lead_departed = lambda sm: True
  try:
    with _nav(off, live=False):
      stop, a = h.apply(True, 0.3, 0.0, standstill=True, gas=False, model_stop=True,
                        sm=_radar_sm(), now=75.0)
  finally:
    mod.radar_lead_departed = orig
  assert stop is True
  assert a <= -1.0
  assert h._nav_go_latched is False


def test_vision_pin_holds_at_rest_without_iqlink():
  """IQ-link off: model_stop rising edge pins standstill with a≤−1 (no creep)."""
  h = StandstillHold()
  off = _snap(ts=80.0, iqlink_enabled=False, link_ok=False, traffic_light="", stop_for_light=False)
  with _nav(off, live=False):
    stop, a = h.apply(True, 0.2, 0.0, standstill=True, gas=False, model_stop=True,
                      sm=_radar_sm(), now=80.0)
    assert h.vision_pin is True
    assert stop is True
    assert a <= -1.0
    stop, a = h.apply(False, 0.4, 0.0, standstill=True, gas=False, model_stop=False,
                      sm=_radar_sm(), now=80.5)
  assert h.vision_pin is True
  assert stop is True
  assert a <= -1.0

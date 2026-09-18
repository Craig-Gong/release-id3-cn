"""CTM parity: e2e stays on for no-lead nav red; offset not skipped by light color."""
from pathlib import Path
from types import SimpleNamespace

from openpilot.sunnypilot.nav.snapshot import NavSnapshot
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.standstill_hold import (
  StandstillHold, _GO_LAUNCH_FLOOR_A, _STANDSTILL_HOLD_RELEASE_S,
)

_HELPERS = Path(__file__).resolve().parents[1]
_PLANNER = _HELPERS / "longitudinal_planner.py"
_OFFSET = _HELPERS / "helpers" / "traffic_stop_offset.py"


def _snap(**kwargs) -> NavSnapshot:
  base = dict(
    ts=10.0, link_ok=True, link_state=2, iqlink_enabled=True,
    traffic_light="red", dist_m=40.0, remain_s=8.0, remain_go=False,
    stop_for_light=True, speed_target=0.0, accel_target=-2.0,
  )
  base.update(kwargs)
  return NavSnapshot(**base)


def _radar_sm(*, present=False, d_rel=20.0):
  lead = SimpleNamespace(present=present, status=present, dRel=float(d_rel), vLead=0.0)
  return {"radarState": SimpleNamespace(leadOne=lead)}


def test_is_e2e_source_allows_nav_red():
  """Regression: red_pin / stop_for_light must not force is_e2e=False."""
  src = _PLANNER.read_text()
  assert 'if getattr(self.standstill_hold, "red_pin", False):\n      return False' not in src
  assert "snap.stop_for_light:\n        return False" not in src
  assert "do NOT disable e2e on nav/sticky red" in src
  assert "read_nav_queue_lead" in src  # radar-first; phantoms must not kill e2e
  assert "radar_state_readable" in src


def test_skip_vision_only_on_latch_not_light_color():
  src = _PLANNER.read_text()
  assert "nav_owns_light" not in src
  assert 'skip_vision_stop = bool(getattr(self.standstill_hold, "_nav_go_latched", False))' in src
  assert "Safety net only" in src
  assert "radar-first so vision phantoms" in src


def test_offset_module_still_gates_on_nav_red_flag():
  """adjust(..., nav_red=True) must no-op; planner only sets that for green latch."""
  src = _OFFSET.read_text()
  assert "nav_red" in src
  assert "if self.distance <= 0. or not stop_light or right_blinker or nav_red:" in src


def test_at_rest_red_pin_still_nails_hold():
  from openpilot.sunnypilot.selfdrive.controls.lib.helpers import standstill_hold as mod

  h = StandstillHold()
  t0 = 90.0
  red = _snap(ts=t0, dist_m=5.0)
  orig_read, orig_exec = mod.read_snapshot, mod.snapshot_executable
  mod.read_snapshot = lambda: red
  mod.snapshot_executable = lambda snap, now=None: True
  try:
    stop, a = h.apply(True, -0.2, 0.0, standstill=True, gas=False, model_stop=False,
                      sm=_radar_sm(), now=t0)
    assert h.red_pin is True
    assert stop is True
    assert a <= -1.0
  finally:
    mod.read_snapshot = orig_read
    mod.snapshot_executable = orig_exec


def test_confirmed_green_releases_after_dwell():
  from openpilot.sunnypilot.selfdrive.controls.lib.helpers import standstill_hold as mod

  h = StandstillHold()
  t0 = 100.0
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
                        sm=_radar_sm(), now=t0 + 0.05 * i)
    assert stop is False
    assert a >= _GO_LAUNCH_FLOOR_A
  finally:
    mod.read_snapshot = orig_read
    mod.snapshot_executable = orig_exec

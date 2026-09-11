"""Queue behind stopped lead + far nav red: lead owns, not nav hard-stop."""
from openpilot.sunnypilot.nav.snapshot import NavSnapshot
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.green_follow_lead import (
  apply_stopped_lead_gap,
  lead_owns_nav_stop,
)
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.standstill_hold import StandstillHold


class _Lead:
  def __init__(self, present, d_rel, v_lead):
    self.present = present
    self.status = present
    self.dRel = d_rel
    self.vLead = v_lead


def _sm(d_rel=8.0, v_lead=0.0, present=True):
  return {"radarState": type("R", (), {"leadOne": _Lead(present, d_rel, v_lead)})()}


def test_far_red_with_stopped_lead_owns():
  snap = NavSnapshot(stop_for_light=True, dist_m=201.0, traffic_light="red")
  assert lead_owns_nav_stop(_sm(d_rel=8.0, v_lead=0.0), snap) is True


def test_far_red_with_rolling_lead_still_owns():
  # Launch must not flip ownership back to far red when the bumper starts.
  snap = NavSnapshot(stop_for_light=True, dist_m=180.0, traffic_light="red")
  assert lead_owns_nav_stop(_sm(d_rel=8.0, v_lead=1.0), snap) is True
  assert lead_owns_nav_stop(_sm(d_rel=8.0, v_lead=3.0), snap) is True


def test_head_car_far_red_does_not_own():
  snap = NavSnapshot(stop_for_light=True, dist_m=201.0, traffic_light="red")
  assert lead_owns_nav_stop(_sm(present=False), snap) is False


def test_lead_past_light_does_not_own():
  # Track beyond the light point — do not treat as queue bumper.
  snap = NavSnapshot(stop_for_light=True, dist_m=15.0, traffic_light="red")
  assert lead_owns_nav_stop(_sm(d_rel=20.0, v_lead=0.0), snap) is False


def test_gap_helper_runs_when_lead_owns_far_red(monkeypatch=None):
  snap = NavSnapshot(
    ts=10.0, link_ok=True, iqlink_enabled=True, stop_for_light=True,
    dist_m=201.0, traffic_light="red", accel_target=-2.0,
  )
  import openpilot.sunnypilot.selfdrive.controls.lib.helpers.green_follow_lead as mod
  orig = mod.read_snapshot if hasattr(mod, "read_snapshot") else None
  # apply_stopped_lead_gap imports read_snapshot locally; patch snapshot module.
  import openpilot.sunnypilot.nav.snapshot as snap_mod
  old = snap_mod.read_snapshot
  snap_mod.read_snapshot = lambda: snap
  snap_mod.snapshot_executable = lambda s, now=None: True
  try:
    # 6 m behind stopped lead: should pin (gap), not passthrough nav skip.
    a, stop = apply_stopped_lead_gap(_sm(d_rel=6.0, v_lead=0.0), v_ego=0.2, a_target=0.5,
                                     should_stop=False, red_pin=True)
    assert stop is True
    assert a < 0.0
  finally:
    snap_mod.read_snapshot = old


def test_standstill_clears_red_pin_when_lead_owns():
  h = StandstillHold()
  snap = NavSnapshot(
    ts=10.0, link_ok=True, link_state=2, iqlink_enabled=True,
    traffic_light="red", stop_for_light=True, dist_m=180.0,
  )
  sm = _sm(d_rel=7.0, v_lead=0.0)
  h.observe_nav(snap, 10.0, gas=False, v_ego=0.0, sm=sm)
  assert h.red_pin is False

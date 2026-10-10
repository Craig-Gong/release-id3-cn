"""Cautious Unprotected Turn — activate / exit / follow / gas.

Host-safe: stubs Params / numpy-heavy imports so Mac unittest can run.
"""
from __future__ import annotations

import os
import sys
import tempfile
import types
import unittest
from unittest import mock

# --- host stubs (before importing CUT) ---
_KPH_TO_MS = 1.0 / 3.6

if "openpilot.common.constants" not in sys.modules:
  _cv = types.ModuleType("openpilot.common.constants")
  _cv.CV = types.SimpleNamespace(KPH_TO_MS=_KPH_TO_MS, MS_TO_KPH=3.6)
  sys.modules["openpilot.common.constants"] = _cv

if "openpilot.common.params" not in sys.modules:
  _params = types.ModuleType("openpilot.common.params")

  class UnknownKeyName(Exception):
    pass

  class Params:
    def get_bool(self, key):
      return True

  _params.UnknownKeyName = UnknownKeyName
  _params.Params = Params
  sys.modules["openpilot.common.params"] = _params

if "openpilot.common.file_params" not in sys.modules:
  _fp = types.ModuleType("openpilot.common.file_params")
  _fp.read_file_param = lambda key, default=None: default
  sys.modules["openpilot.common.file_params"] = _fp

from openpilot.sunnypilot.nav.snapshot import NavSnapshot  # noqa: E402
from openpilot.sunnypilot.selfdrive.controls.lib.helpers import unprotected_turn as ut  # noqa: E402


class _ParamsOn:
  def get_bool(self, key):
    return True


class _ParamsOff:
  def get_bool(self, key):
    return False


def _assist(on=True) -> ut.UnprotectedTurnAssist:
  a = ut.UnprotectedTurnAssist.__new__(ut.UnprotectedTurnAssist)
  a._params = _ParamsOn() if on else _ParamsOff()
  a._last = ut.CutDecision()
  return a


def _left_snap(dist_m: float, *, remain_go=False, light="none") -> NavSnapshot:
  return NavSnapshot(
    send_turn=True,
    maneuver_dir="left",
    tbt_dist=float(dist_m),
    remain_go=bool(remain_go),
    traffic_light=str(light),
  )


def _dec(a: ut.UnprotectedTurnAssist, *, v_kph=20.0, standstill=False, gas=False,
         left=True, right=False, near_lead=False, snap=None, steer=0.0,
         path_x=None, path_y=None, lane_change_state=0, nav_go_latched=False,
         posted_kph=0.0, enabled=True):
  with mock.patch.object(ut, "write_cut_snapshot", lambda *args, **kwargs: None):
    return a.update(
      v_ego=float(v_kph) * _KPH_TO_MS,
      enabled=enabled,
      standstill=standstill,
      gas=gas,
      left_blinker=left,
      right_blinker=right,
      steering_angle_deg=steer,
      lane_change_state=lane_change_state,
      near_lead=near_lead,
      posted_limit_ms=float(posted_kph) * _KPH_TO_MS,
      path_x=path_x,
      path_y=path_y,
      snap=snap,
      nav_go_latched=nav_go_latched,
    )


class TestUnprotectedTurn(unittest.TestCase):
  def test_nav_left_near_caps(self):
    a = _assist()
    d = _dec(a, v_kph=30.0, snap=_left_snap(40.0))
    self.assertTrue(d.active)
    self.assertIsNotNone(d.v_cap_ms)
    self.assertAlmostEqual(d.v_cap_ms, ut.WAIT_CAP_MS, places=4)
    self.assertFalse(d.hold)
    self.assertTrue(d.hud)

  def test_nav_left_far_arms_without_cap(self):
    a = _assist()
    d = _dec(a, v_kph=30.0, snap=_left_snap(70.0))
    self.assertTrue(d.active)
    self.assertIsNone(d.v_cap_ms)
    self.assertFalse(d.hold)
    self.assertFalse(d.hud)

  def test_standstill_hold_near_corner(self):
    a = _assist()
    d = _dec(a, v_kph=0.0, standstill=True, snap=_left_snap(35.0))
    self.assertTrue(d.active)
    self.assertTrue(d.hold)
    self.assertTrue(d.hud)

  def test_creep_no_hold(self):
    a = _assist()
    d = _dec(a, v_kph=8.0, standstill=False, snap=_left_snap(35.0))
    self.assertTrue(d.active)
    self.assertIsNotNone(d.v_cap_ms)
    self.assertFalse(d.hold)

  def test_near_lead_exits(self):
    a = _assist()
    d = _dec(a, snap=_left_snap(35.0), near_lead=True)
    self.assertFalse(d.active)
    self.assertEqual(d.reason, "lead")

  def test_gas_exits(self):
    a = _assist()
    d = _dec(a, snap=_left_snap(35.0), gas=True, standstill=True)
    self.assertFalse(d.active)
    self.assertEqual(d.reason, "off")

  def test_nav_go_exits(self):
    a = _assist()
    d = _dec(a, snap=_left_snap(35.0, remain_go=True), standstill=True)
    self.assertFalse(d.active)
    self.assertEqual(d.reason, "nav_go")

  def test_green_light_exits(self):
    a = _assist()
    d = _dec(a, snap=_left_snap(35.0, light="green"), standstill=True)
    self.assertFalse(d.active)
    self.assertEqual(d.reason, "nav_go")

  def test_right_blinker_exits(self):
    a = _assist()
    d = _dec(a, snap=_left_snap(35.0), right=True)
    self.assertFalse(d.active)

  def test_highway_blocked(self):
    a = _assist()
    d = _dec(a, v_kph=40.0, left=True, snap=None, posted_kph=80.0, steer=25.0,
             path_x=[0, 15, 25], path_y=[0, 2.0, 2.5])
    self.assertFalse(d.active)
    self.assertEqual(d.reason, "highway")

  def test_weak_left_needs_turn_in(self):
    a = _assist()
    d = _dec(a, v_kph=18.0, left=True, snap=None, steer=5.0)
    self.assertFalse(d.active)
    self.assertEqual(d.reason, "no_intent")
    d2 = _dec(a, v_kph=18.0, left=True, snap=None, steer=25.0, standstill=True)
    self.assertTrue(d2.active)
    self.assertTrue(d2.hold)
    self.assertEqual(d2.reason, "weak_left")

  def test_param_off(self):
    a = _assist(on=False)
    d = _dec(a, snap=_left_snap(35.0), standstill=True)
    self.assertFalse(d.active)

  def test_fast_exits(self):
    a = _assist()
    d = _dec(a, v_kph=50.0, snap=_left_snap(40.0))
    self.assertFalse(d.active)
    self.assertEqual(d.reason, "fast")

  def test_shm_roundtrip(self):
    with tempfile.TemporaryDirectory() as td:
      path = os.path.join(td, "sp_cut.json")
      ut.write_cut_snapshot(ut.CutSnapshot(ts=100.0, active=True, hold=True, hud=True, reason="nav_left"), path)
      snap = ut.read_cut_snapshot(path, now=100.5)
      self.assertTrue(snap.active)
      self.assertTrue(snap.hold)
      stale = ut.read_cut_snapshot(path, now=103.0)
      self.assertFalse(stale.active)


if __name__ == "__main__":
  unittest.main()

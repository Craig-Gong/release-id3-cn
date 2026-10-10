"""Unit tests for gated MEB software blinker / nav auto-LC intent."""
from __future__ import annotations

import os
import tempfile
import unittest
from unittest import mock

from openpilot.sunnypilot.nav.snapshot import NavSnapshot
from openpilot.sunnypilot.selfdrive.controls.lib.helpers import nav_auto_blinker as nab


def _snap(**kwargs) -> NavSnapshot:
  base = dict(
    ts=100.0,
    link_ok=True,
    iqlink_enabled=True,
    maneuver="turn",
    maneuver_dir="left",
    tbt_dist=50.0,
    send_turn=True,
    road_limit_kph=50.0,
  )
  base.update(kwargs)
  return NavSnapshot(**base)


class TestNavAutoBlinker(unittest.TestCase):
  def setUp(self):
    self._tmpdir = tempfile.TemporaryDirectory()
    self._patcher = mock.patch.object(nab, "FORCE_BLINKER_PARAM", "MebForceBlinker")
    self._patcher.start()
    self._fp = mock.patch("openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_auto_blinker.read_file_param")
    self._wp = mock.patch("openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_auto_blinker.write_file_param")
    self.read = self._fp.start()
    self.write = self._wp.start()
    self._store = {"MebForceBlinker": 0, "NavAutoBlinker": False, "NavAutoLaneChange": False}

    def _r(key, default=None):
      return self._store.get(key, default)

    def _w(key, value):
      self._store[key] = value

    self.read.side_effect = _r
    self.write.side_effect = _w
    nab._force_until_mono = 0.0
    nab._force_side = 0

  def tearDown(self):
    self._fp.stop()
    self._wp.stop()
    self._patcher.stop()
    self._tmpdir.cleanup()

  def test_force_left_then_expire(self):
    self._store["MebForceBlinker"] = 1
    r = nab.evaluate_blinker_request(snap=None, v_ego_mps=0.0, now=10.0)
    self.assertEqual(r.source, "force")
    self.assertTrue(r.left)
    r2 = nab.evaluate_blinker_request(snap=None, v_ego_mps=0.0, now=10.0 + nab.FORCE_BLINKER_HOLD_S + 0.1)
    self.assertEqual(r2.source, "none")
    self.assertEqual(self._store["MebForceBlinker"], 0)

  def test_stalk_yields(self):
    self._store["MebForceBlinker"] = 1
    r = nab.evaluate_blinker_request(
      snap=None, v_ego_mps=0.0, left_blinker_active=True, now=10.0,
    )
    self.assertEqual(r.source, "yield_stalk")
    self.assertFalse(r.left)

  def test_nav_turn_requires_enable(self):
    snap = _snap()
    r = nab.evaluate_blinker_request(snap=snap, v_ego_mps=10.0, now=100.0)
    self.assertEqual(r.source, "none")
    self._store["NavAutoBlinker"] = True
    r2 = nab.evaluate_blinker_request(snap=snap, v_ego_mps=10.0, now=100.0)
    self.assertEqual(r2.source, "turn")
    self.assertTrue(r2.left)

  def test_nav_turn_not_at_highway_speed(self):
    self._store["NavAutoBlinker"] = True
    snap = _snap()
    r = nab.evaluate_blinker_request(snap=snap, v_ego_mps=70 / 3.6, now=100.0)
    self.assertEqual(r.source, "none")

  def test_nav_lc_gates(self):
    self._store["NavAutoLaneChange"] = True
    snap = _snap(
      maneuver="fork", maneuver_dir="right", send_turn=False,
      tbt_dist=120.0, road_limit_kph=100.0,
    )
    # too slow
    r = nab.evaluate_blinker_request(snap=snap, v_ego_mps=30 / 3.6, now=100.0)
    self.assertEqual(r.source, "none")
    # ok
    r2 = nab.evaluate_blinker_request(snap=snap, v_ego_mps=80 / 3.6, now=100.0)
    self.assertEqual(r2.source, "lc")
    self.assertTrue(r2.right)
    # BSM blocks
    r3 = nab.evaluate_blinker_request(
      snap=snap, v_ego_mps=80 / 3.6, right_blindspot=True, now=100.0,
    )
    self.assertEqual(r3.source, "none")
    # urban road limit — no auto LC
    urban = _snap(
      maneuver="fork", maneuver_dir="left", send_turn=False,
      tbt_dist=120.0, road_limit_kph=50.0,
    )
    r4 = nab.evaluate_blinker_request(snap=urban, v_ego_mps=50 / 3.6, now=100.0)
    self.assertEqual(r4.source, "none")

  def test_intersection_turn_blocks_lc_path(self):
    self._store["NavAutoLaneChange"] = True
    self._store["NavAutoBlinker"] = False
    snap = _snap(maneuver="turn", send_turn=True, road_limit_kph=80.0, tbt_dist=80.0)
    r = nab.evaluate_blinker_request(snap=snap, v_ego_mps=50 / 3.6, now=100.0)
    self.assertEqual(r.source, "none")


class TestMebCreateBlinker(unittest.TestCase):
  def test_create_blinker_left(self):
    try:
      from opendbc.can import CANPacker
      from opendbc.car.volkswagen import mebcan
      packer = CANPacker("vw_meb_generated")
    except Exception:
      self.skipTest("opendbc / vw_meb_generated not available in this env")
    stock = {s: 0 for s in [
      "EA_Texte", "ACF_Lampe_Hands_Off", "EA_Infotainment_Anf", "EA_Tueren_Anf",
      "EA_Innenraumlicht_Anf", "zFAS_Warnblinken", "STP_Primaeranz",
      "EA_Bremslichtblinken", "EA_Blinken", "EA_Unknown",
    ]}
    msg = mebcan.create_blinker_control(packer, 0, stock, {"EA_Funktionsstatus": 0}, True, False)
    addr = msg[0] if isinstance(msg, (tuple, list)) else getattr(msg, "address", None)
    self.assertEqual(addr, 0x1F0)


if __name__ == "__main__":
  unittest.main()

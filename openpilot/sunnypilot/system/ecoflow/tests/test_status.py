#!/usr/bin/env python3
import tempfile
import unittest
from pathlib import Path

from openpilot.sunnypilot.system.ecoflow.status import (
  EcoflowStatus, dc12v_from_telemetry, ecoflow_dc_label, read_status, write_status,
)


class TestDcFromTelemetry(unittest.TestCase):
  def test_cfg_open(self):
    self.assertTrue(dc12v_from_telemetry({"cfg_dc12v_out_open": 1}))
    self.assertFalse(dc12v_from_telemetry({"cfg_dc12v_out_open": 0}))

  def test_flow_info_fallback(self):
    self.assertTrue(dc12v_from_telemetry({"flow_info_12v": 14}))
    self.assertFalse(dc12v_from_telemetry({"flow_info_12v": 4}))
    self.assertIsNone(dc12v_from_telemetry({}))
    self.assertIsNone(dc12v_from_telemetry(None))


class TestEcoflowDcLabel(unittest.TestCase):
  def test_disabled(self):
    snap = EcoflowStatus(ts=10.0, enabled=True, mqtt=True, dc12v=True)
    self.assertEqual(ecoflow_dc_label(enabled=False, snap=snap, now=10.0), "12V 未启用")

  def test_unknown_when_stale_or_missing(self):
    stale = EcoflowStatus(ts=10.0, enabled=True, mqtt=True, dc12v=True)
    self.assertEqual(ecoflow_dc_label(enabled=True, snap=stale, now=20.0), "12V 未知")
    empty = EcoflowStatus()
    self.assertEqual(ecoflow_dc_label(enabled=True, snap=empty, now=10.0), "12V 未知")

  def test_on_off(self):
    on = EcoflowStatus(ts=10.0, enabled=True, mqtt=True, dc12v=True)
    off = EcoflowStatus(ts=10.0, enabled=True, mqtt=True, dc12v=False)
    self.assertEqual(ecoflow_dc_label(enabled=True, snap=on, now=10.1), "12V 开")
    self.assertEqual(ecoflow_dc_label(enabled=True, snap=off, now=10.1), "12V 关")


class TestStatusShm(unittest.TestCase):
  def test_roundtrip(self):
    with tempfile.TemporaryDirectory() as td:
      path = str(Path(td) / "sp_ecoflow.json")
      write_status(enabled=True, mqtt=True, dc12v=True, kl15=True, want_on=True, path=path, now=12.5)
      snap = read_status(path)
      self.assertTrue(snap.enabled)
      self.assertTrue(snap.mqtt)
      self.assertTrue(snap.dc12v)
      self.assertTrue(snap.kl15)
      self.assertTrue(snap.want_on)
      self.assertEqual(snap.ts, 12.5)

  def test_missing_file(self):
    snap = read_status("/tmp/sp_ecoflow_missing_no_such_file.json")
    self.assertFalse(snap.enabled)
    self.assertIsNone(snap.dc12v)


if __name__ == "__main__":
  unittest.main()

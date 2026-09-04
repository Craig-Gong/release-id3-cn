#!/usr/bin/env python3
import unittest

from openpilot.sunnypilot.nav.snapshot import STALE_LINK_S, NavSnapshot, snapshot_executable


class TestSnapshotExecutable(unittest.TestCase):
  def test_fresh_packet_executes(self):
    snap = NavSnapshot(ts=10.0, link_ok=True, link_state=2, iqlink_enabled=True)
    self.assertTrue(snapshot_executable(snap, now=10.0 + STALE_LINK_S))

  def test_stale_packet_does_not_execute_even_if_link_state_connected(self):
    snap = NavSnapshot(ts=10.0, link_ok=True, link_state=2, iqlink_enabled=True)
    self.assertFalse(snapshot_executable(snap, now=10.0 + STALE_LINK_S + 0.01))

  def test_link_down_keeps_hud_but_does_not_execute(self):
    snap = NavSnapshot(ts=10.0, link_ok=False, link_state=1, iqlink_enabled=True)
    self.assertFalse(snapshot_executable(snap, now=10.1))

  def test_disabled_does_not_execute(self):
    snap = NavSnapshot(ts=10.0, link_ok=True, link_state=2, iqlink_enabled=False)
    self.assertFalse(snapshot_executable(snap, now=10.1))


if __name__ == "__main__":
  unittest.main()

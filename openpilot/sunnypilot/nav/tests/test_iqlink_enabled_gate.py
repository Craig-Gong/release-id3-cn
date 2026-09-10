"""Regression: IqlinkEnabled gates control, not the ability to stay link_ok."""
from openpilot.sunnypilot.nav.snapshot import NavSnapshot, snapshot_executable


def test_disabled_toggle_blocks_execution_even_when_link_fresh():
  snap = NavSnapshot(
    ts=10.0, link_ok=True, link_state=2, iqlink_enabled=False,
    road_limit_kph=50.0, stop_for_light=True, send_turn=True,
  )
  assert not snapshot_executable(snap, now=10.1)


def test_enabled_and_fresh_is_executable():
  snap = NavSnapshot(
    ts=10.0, link_ok=True, link_state=2, iqlink_enabled=True,
    road_limit_kph=50.0,
  )
  assert snapshot_executable(snap, now=10.1)

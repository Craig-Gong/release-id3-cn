"""Park/reverse / stale BLE must not follow leftover IQlink speedTarget."""
from types import SimpleNamespace

from openpilot.iqpilot.selfdrive.controls.lib.longitudinal_planner import (
  nav_long_blocked,
  nav_long_blocked_by_gear,
)


def test_park_blocks_nav_long():
  assert nav_long_blocked_by_gear(SimpleNamespace(name="park")) is True


def test_reverse_blocks_nav_long():
  assert nav_long_blocked_by_gear(SimpleNamespace(name="reverse")) is True


def test_drive_allows_nav_long():
  assert nav_long_blocked_by_gear(SimpleNamespace(name="drive")) is False


def test_stale_ble_blocks_nav_long_in_drive():
  assert nav_long_blocked(SimpleNamespace(name="drive"), link_warn=True) is True


def test_fresh_ble_allows_nav_long_in_drive():
  assert nav_long_blocked(SimpleNamespace(name="drive"), link_warn=False) is False

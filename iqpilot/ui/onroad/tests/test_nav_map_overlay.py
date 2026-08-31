from types import SimpleNamespace

from iqpilot.ui.onroad.nav_map_utils import iqnav_dest_coords


def test_iqnav_dest_coords_from_goal_not_zero():
  nav = SimpleNamespace(destinationLatitude=32.10, destinationLongitude=118.80)
  lat, lon, ok = iqnav_dest_coords(nav)
  assert ok
  assert abs(lat - 32.10) < 1e-6
  assert abs(lon - 118.80) < 1e-6


def test_iqnav_dest_coords_rejects_missing_pin():
  nav = SimpleNamespace(destinationLatitude=0.0, destinationLongitude=0.0)
  _, _, ok = iqnav_dest_coords(nav)
  assert ok is False

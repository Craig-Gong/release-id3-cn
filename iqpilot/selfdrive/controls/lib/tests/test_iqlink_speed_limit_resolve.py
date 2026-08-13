"""Gaode (Iqlink) road limit must win HUD resolve when mapd OSM is empty."""
from types import SimpleNamespace

from openpilot.iqpilot.selfdrive.controls.lib.speed_limit_controller import (
  IQSpeedLimitResolver,
  POLICY_MAP_DATA_ONLY,
  POLICY_MAP_DATA_PRIORITY,
)


class _FakeSM:
  def __init__(self, alive, **msgs):
    self.alive = alive
    self._msgs = msgs

  def __getitem__(self, key):
    return self._msgs[key]


def test_iqlink_preferred_over_empty_map():
  r = IQSpeedLimitResolver()
  r.map_speed_limit = 0.0
  lim, src = r.resolve(0.0, 0.0, {"slc_policy": POLICY_MAP_DATA_PRIORITY}, iqlink_limit=22.22)
  assert src == "Iqlink"
  assert abs(lim - 22.22) < 1e-6


def test_iqlink_preferred_over_map_data():
  r = IQSpeedLimitResolver()
  r.map_speed_limit = 16.67  # 60 kph OSM
  lim, src = r.resolve(0.0, 0.0, {"slc_policy": POLICY_MAP_DATA_PRIORITY}, iqlink_limit=27.78)  # 100
  assert src == "Iqlink"
  assert abs(lim - 27.78) < 1e-6


def test_iqlink_in_map_data_only_policy():
  r = IQSpeedLimitResolver()
  lim, src = r.resolve(0.0, 0.0, {"slc_policy": POLICY_MAP_DATA_ONLY}, iqlink_limit=16.67)
  assert src == "Iqlink"
  assert abs(lim - 16.67) < 1e-6


def test_below_min_ignored():
  r = IQSpeedLimitResolver()
  lim, src = r.resolve(0.0, 0.0, {"slc_policy": POLICY_MAP_DATA_PRIORITY}, iqlink_limit=5.0)
  assert src == "None"
  assert lim == 0.0


def test_update_iqlink_nav_from_target_speed():
  r = IQSpeedLimitResolver()
  # Clear shm so fallback path is exercised
  try:
    open("/dev/shm/iqlink_road_speed_ms", "w", encoding="utf-8").write("0")
  except OSError:
    pass
  sm = _FakeSM(
    {"iqNavState": True},
    iqNavState=SimpleNamespace(
      active=True,
      targetSpeedValid=True,
      navSpeedTargetActive=False,
      longitudinalProvider="route",
      targetSpeed=22.22,
    ),
  )
  r.update_iqlink_nav(sm)
  assert abs(r.iqlink_speed_limit - 22.22) < 1e-6
  lim, src = r.resolve(0.0, 0.0, {"slc_policy": POLICY_MAP_DATA_PRIORITY})
  assert src == "Iqlink"


def test_update_iqlink_keeps_last_while_tbt_cap():
  r = IQSpeedLimitResolver()
  r.iqlink_speed_limit = 27.78
  try:
    open("/dev/shm/iqlink_road_speed_ms", "w", encoding="utf-8").write("0")
  except OSError:
    pass
  sm = _FakeSM(
    {"iqNavState": True},
    iqNavState=SimpleNamespace(
      active=True,
      targetSpeedValid=True,
      navSpeedTargetActive=True,  # TBT/red capped
      longitudinalProvider="route",
      targetSpeed=8.33,
    ),
  )
  r.update_iqlink_nav(sm)
  assert abs(r.iqlink_speed_limit - 27.78) < 1e-6


def test_update_iqlink_shm_raw_beats_capped_target():
  r = IQSpeedLimitResolver()
  try:
    open("/dev/shm/iqlink_road_speed_ms", "w", encoding="utf-8").write("16.6667")
  except OSError:
    return  # skip on hosts without /dev/shm
  sm = _FakeSM(
    {"iqNavState": True},
    iqNavState=SimpleNamespace(
      active=True,
      targetSpeedValid=True,
      navSpeedTargetActive=True,
      longitudinalProvider="route",
      targetSpeed=13.333,  # green-wave capped
    ),
  )
  r.update_iqlink_nav(sm)
  assert abs(r.iqlink_speed_limit - 16.6667) < 1e-3
  lim, src = r.resolve(0.0, 0.0, {"slc_policy": POLICY_MAP_DATA_PRIORITY})
  assert src == "Iqlink"
  assert abs(lim * 3.6 - 60.0) < 0.1

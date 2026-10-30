from openpilot.sunnypilot.nav.road_class import (
  MOTORWAY_LIMIT_FLOOR_KPH, motorway_context, urban_nav_band,
)


def test_motorway_cate_zero():
  assert motorway_context(0) is True
  assert urban_nav_band(60, roadcate=0) is False


def test_sticky_motorway_ignored_below_floor():
  """After leaving freeway, sticky ROADCATE=0 + city limit must not block RTOR."""
  assert MOTORWAY_LIMIT_FLOOR_KPH == 50.0
  assert urban_nav_band(40, roadcate=0) is True
  assert urban_nav_band(50, roadcate=0) is False


def test_city_surface_cate():
  assert motorway_context(8) is False
  assert urban_nav_band(60, roadcate=8) is True


def test_unknown_and_express_leave_limit_only():
  assert motorway_context(-1) is None
  assert motorway_context(6) is None  # 城市快速路
  assert urban_nav_band(60) is True
  assert urban_nav_band(80) is False

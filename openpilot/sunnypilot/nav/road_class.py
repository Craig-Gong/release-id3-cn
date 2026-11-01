"""Gaode ROADCATE / ROAD_TYPE → soft motorway vs urban hints.

AutoSDK RoadClass (broadcast ROADCATE) — verified against GBL RoadClass:
  0 Freeway  1 National  2 Province  3 County  4 Rural  5 In-county
  6 CitySpeedway  7 Main  8 Secondary  9 Common  10 NonNavi  (−1 NULL)

Never replace posted limit alone — only bias promote / Nav LC / CUT when the
hint is decisive. Missing / unknown cate → None (legacy limit-only behavior).

Sticky APK roadcate=0 after leaving the freeway is guarded by
MOTORWAY_LIMIT_FLOOR_KPH: below that floor we ignore motorway cate so city
RTOR / lc promote / CUT are not permanently suppressed.
"""
from __future__ import annotations

# Strong motorway. National (1) left unknown — mixes trunk + city arterials.
_MOTORWAY_ROADCATE = frozenset({0})
# Clearly not a controlled-access motorway approach.
_CITY_SURFACE_ROADCATE = frozenset({3, 4, 5, 7, 8, 9, 10})
# Posted limit below this → do not trust motorway cate (city after ramp /
# sticky Partner field). Matches Nav LC HWY_EXIT_LIMIT_MIN_KPH.
MOTORWAY_LIMIT_FLOOR_KPH = 50.0


def motorway_context(roadcate: int = -1, road_type: int = -1) -> bool | None:
  """True = freeway; False = city surface; None = unknown / undecided.

  road_type is accepted for forward-compat but not decisive alone (SDK enum
  varies across amapauto builds).
  """
  del road_type  # reserved; Partner may send ROAD_TYPE but values are not stable
  cate = int(roadcate)
  if cate in _MOTORWAY_ROADCATE:
    return True
  if cate in _CITY_SURFACE_ROADCATE:
    return False
  # City express (6) / national (1) / provincial (2): limit still decides.
  return None


def urban_nav_band(
  road_kph: float,
  *,
  roadcate: int = -1,
  road_type: int = -1,
  promote_max_kph: float = 70.0,
) -> bool:
  """lc_* may promote to send_turn / urban RTOR / CUT fork.

  Motorway cate with a dropped posted limit (e.g. 100→60 at ramp) stays
  *outside* the urban band so Nav LC keeps the fork/exit path — but only
  while limit is still ≥ MOTORWAY_LIMIT_FLOOR_KPH.
  """
  road = float(road_kph)
  if road >= float(promote_max_kph):
    return False
  if (
    motorway_context(roadcate, road_type) is True
    and road >= MOTORWAY_LIMIT_FLOOR_KPH
  ):
    return False
  return True

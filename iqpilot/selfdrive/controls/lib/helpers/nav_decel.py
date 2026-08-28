"""Shared distance-based speed caps for nav longitudinal helpers."""

from __future__ import annotations

import math


def approach_speed_ms(
  distance_m: float,
  decel_ms2: float,
  *,
  floor_ms: float = 0.0,
  cap_ms: float = 0.0,
) -> float:
  """Comfort speed upper bound at distance_m with constant decel toward floor_ms."""
  d = max(0.0, float(distance_m))
  floor_ms = max(0.0, float(floor_ms))
  decel_ms2 = max(0.05, float(decel_ms2))
  if d <= 0.05:
    return floor_ms
  v = math.sqrt(2.0 * decel_ms2 * d + floor_ms * floor_ms)
  if cap_ms > 0.0:
    v = min(v, float(cap_ms))
  return max(v, floor_ms)

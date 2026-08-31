"""Debounce Gaode nRoadLimitSpeed before HUD / SLC adoption.

Gaode BLE updates can flutter one step (e.g. 60↔80) without a real zone change.
Camera VZE already uses a 1.5 s hold; map limits adopt faster on decreases (real
zones) and slower on large jumps (glitches).
"""

from __future__ import annotations

# Seconds a new reading must stay stable before replacing the confirmed limit.
_HOLD_NORMAL_S = 1.0
_HOLD_LARGE_JUMP_S = 1.5
_HOLD_DECREASE_S = 0.6
_HOLD_FIRST_S = 0.5

_LARGE_JUMP_KPH = 40.0


class IqlinkRoadLimitHold:
  def __init__(self) -> None:
    self._confirmed_kph = 0.0
    self._pending_kph = -1.0
    self._pending_since = 0.0

  @property
  def confirmed_kph(self) -> float:
    return self._confirmed_kph

  def reset(self) -> None:
    self._confirmed_kph = 0.0
    self._pending_kph = -1.0
    self._pending_since = 0.0

  def _hold_seconds(self, raw_kph: float) -> float:
    if self._confirmed_kph <= 0.0:
      return _HOLD_FIRST_S
    delta = raw_kph - self._confirmed_kph
    if delta <= -1.0:
      return _HOLD_DECREASE_S
    if abs(delta) >= _LARGE_JUMP_KPH:
      return _HOLD_LARGE_JUMP_S
    return _HOLD_NORMAL_S

  def filter_kph(self, raw_kph: float, now: float) -> float:
    """Return debounced road limit (km/h). Invalid/zero raw keeps last confirmed."""
    raw_kph = max(0.0, float(raw_kph))
    if raw_kph <= 0.0:
      return self._confirmed_kph

    if abs(raw_kph - self._confirmed_kph) < 0.5:
      self._pending_kph = -1.0
      return self._confirmed_kph

    hold_s = self._hold_seconds(raw_kph)
    if self._pending_kph != raw_kph:
      self._pending_kph = raw_kph
      self._pending_since = now
      return self._confirmed_kph

    if now - self._pending_since >= hold_s:
      self._confirmed_kph = raw_kph
      self._pending_kph = -1.0
    return self._confirmed_kph

"""Camera TSR (VZE) glitch filter for MEB.

Highway flashes of 40 and urban flashes of 120 need 1.5 s of a stable
reading before they replace the last accepted limit.
"""
from __future__ import annotations

import time

from opendbc.car.common.conversions import Conversions as CV

NOT_SET = 0
VZE_HOLD_S = 1.5
VZE_JUMP_KPH = 40
VZE_HIGHWAY_EGO_KPH = 90
VZE_HIGHWAY_FALSE_LOW_KPH = 50
VZE_CITY_EGO_KPH = 55
VZE_CITY_FALSE_HIGH_KPH = 100
SANITY_CHECK_DIFF_PERCENT_LOWER = 50
SPEED_LIMIT_UNLIMITED_VZE_KPH = int(round(144 * CV.MS_TO_KPH))


class VzeSpeedLimitHold:
  def __init__(self):
    self.last_kph = NOT_SET
    self._pending_kph = NOT_SET
    self._pending_since = 0.0
    self._holding = False

  def _needs_hold(self, new_kph: float, v_ego_kph: float) -> bool:
    last = self.last_kph
    if last != NOT_SET:
      if abs(new_kph - last) >= VZE_JUMP_KPH:
        return True
      if last > 0 and (100.0 * new_kph / last) < SANITY_CHECK_DIFF_PERCENT_LOWER:
        return True
    if v_ego_kph >= VZE_HIGHWAY_EGO_KPH and new_kph <= VZE_HIGHWAY_FALSE_LOW_KPH:
      return True
    if v_ego_kph <= VZE_CITY_EGO_KPH and new_kph >= VZE_CITY_FALSE_HIGH_KPH:
      return True
    return False

  def update(self, new_kph: float, v_ego_ms: float, now: float | None = None) -> float:
    if now is None:
      now = time.monotonic()

    if new_kph <= 0 or new_kph > SPEED_LIMIT_UNLIMITED_VZE_KPH:
      self._pending_kph = NOT_SET
      self._holding = False
      return float(self.last_kph)

    v_ego_kph = float(v_ego_ms) * CV.MS_TO_KPH
    if not self._needs_hold(new_kph, v_ego_kph):
      self.last_kph = new_kph
      self._pending_kph = NOT_SET
      self._holding = False
      return float(self.last_kph)

    if self._pending_kph != new_kph:
      self._pending_kph = new_kph
      self._pending_since = now
      self._holding = True
      return float(self.last_kph)

    if now - self._pending_since >= VZE_HOLD_S:
      self.last_kph = new_kph
      self._pending_kph = NOT_SET
      self._holding = False
      return float(self.last_kph)

    self._holding = True
    return float(self.last_kph)

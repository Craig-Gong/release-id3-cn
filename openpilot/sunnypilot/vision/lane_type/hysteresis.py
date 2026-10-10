"""Per-side solid/dashed hysteresis for lane-change gating.

Solid locks after SOLID_ENTER consecutive solid samples.
Unlocks only after DASHED_EXIT consecutive dashed samples (or unknown clears).
Unknown / stale never blocks (fail-open).
"""
from __future__ import annotations

from openpilot.sunnypilot.vision.lane_type.snapshot import (
  LINE_DASHED, LINE_SOLID, LINE_UNKNOWN, LaneTypeSnapshot,
)

SOLID_ENTER = 3
DASHED_EXIT = 5


class SideHysteresis:
  def __init__(self):
    self.locked_solid = False
    self._solid_streak = 0
    self._dashed_streak = 0

  def update(self, line: int) -> bool:
    """Return True when this side should block lane change (confident solid)."""
    if line == LINE_SOLID:
      self._solid_streak += 1
      self._dashed_streak = 0
      if self._solid_streak >= SOLID_ENTER:
        self.locked_solid = True
    elif line == LINE_DASHED:
      self._dashed_streak += 1
      self._solid_streak = 0
      if self._dashed_streak >= DASHED_EXIT:
        self.locked_solid = False
    else:
      # Unknown sample: freeze streaks (no new lock, no unlock). Full-snap
      # invalid/stale is handled by LaneTypeGate.reset() → fail-open.
      self._solid_streak = 0
      self._dashed_streak = 0
    return self.locked_solid

  def reset(self) -> None:
    self.locked_solid = False
    self._solid_streak = 0
    self._dashed_streak = 0


class LaneTypeGate:
  """Consumes shm snapshots; exposes left/right solid blocks for DesireHelper."""

  def __init__(self):
    self.left = SideHysteresis()
    self.right = SideHysteresis()
    self.enabled = False
    self._last_snap = LaneTypeSnapshot()

  def reset(self) -> None:
    self.left.reset()
    self.right.reset()
    self._last_snap = LaneTypeSnapshot()

  def update(self, snap: LaneTypeSnapshot, *, enabled: bool) -> None:
    self.enabled = bool(enabled)
    if not self.enabled or not snap.valid:
      self.reset()
      return
    self._last_snap = snap
    self.left.update(int(snap.left))
    self.right.update(int(snap.right))

  def blocks(self, direction_left: bool) -> bool:
    if not self.enabled:
      return False
    return self.left.locked_solid if direction_left else self.right.locked_solid

  def blocked_left(self) -> bool:
    return self.blocks(True)

  def blocked_right(self) -> bool:
    return self.blocks(False)

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class MarqueeState:
  text: str = ""
  offset: float = 0.0
  pause: float = 0.0


def advance_horizontal_marquee(
  state: MarqueeState,
  text: str,
  text_w: float,
  view_w: float,
  dt: float,
  *,
  speed: float = 32.0,
  pause_s: float = 1.5,
  gap: float = 28.0,
) -> float:
  """Return left scroll offset for clipped single-line marquee text."""
  dt = max(0.0, min(dt, 0.1))
  if text != state.text:
    state.text = text
    state.offset = 0.0
    state.pause = pause_s

  overflow = text_w - view_w
  if overflow <= 0:
    state.offset = 0.0
    state.pause = 0.0
    return 0.0

  if state.pause > 0:
    state.pause = max(0.0, state.pause - dt)
    return 0.0

  state.offset += speed * dt
  if state.offset >= overflow + gap:
    state.offset = 0.0
    state.pause = pause_s
  return state.offset

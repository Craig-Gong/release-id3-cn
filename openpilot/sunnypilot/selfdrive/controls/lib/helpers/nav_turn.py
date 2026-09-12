"""IQ-link intersection turn gates (shm snapshot). No auto blinker / NavExit ALC.

Toast / send_turn / nav-led prep: ≤150 m.
Lateral turn desire (no stalk) + turn-in 20 cap: ≤120 m and vEgo < 45 km/h,
or same-side blinker confirms farther/faster. Same-side BSM blocks.
Product: steer the turn without auto EA_02; driver may still stalk for indicators.
"""
from __future__ import annotations

from openpilot.common.constants import CV
from openpilot.sunnypilot.nav.protocol import NAV_LATERAL_TURN_M, TURN_DESIRE_WINDOW_M
from openpilot.sunnypilot.nav.snapshot import NavSnapshot, snapshot_executable

TURN_TRIGGER_MPS = 45.0 * CV.KPH_TO_MS
# Desire + turn-in near the corner (modeld only pulses on rising edge).
NAV_NEAR_TURN_M = NAV_LATERAL_TURN_M
# Inside this, desire_helper keep-pulses faster so the rising edge hits the corner.
NAV_CORNER_PULSE_M = 50.0
NAV_CORNER_PULSE_S = 0.40
NAV_APPROACH_PULSE_S = 0.70
NAV_DEFAULT_PULSE_S = 1.0


def _dir(value) -> str:
  token = str(value or "none").strip().lower()
  return token if token in ("left", "right") else "none"


def nav_intersection_turn(snap: NavSnapshot) -> bool:
  """Toast / nav-led / desire gate. Gaode urban intersections are often lc_*
  with send_turn promoted; highway forks still need A1 + turn_prep speed caps."""
  return bool(snap.send_turn and _dir(snap.maneuver_dir) != "none")


def eval_nav_turn_desire(
  *,
  direction: str,
  turn_dist_m: float,
  v_ego_mps: float,
  left_blinker: bool,
  right_blinker: bool,
  left_blindspot: bool,
  right_blindspot: bool,
) -> str:
  d = _dir(direction)
  if d == "none":
    return "none"
  if d == "left" and left_blindspot:
    return "none"
  if d == "right" and right_blindspot:
    return "none"
  near_exec = 0.0 < float(turn_dist_m) <= NAV_NEAR_TURN_M
  slow_enough = float(v_ego_mps) < TURN_TRIGGER_MPS
  blinker_ok = (
    (d == "left" and left_blinker and not right_blinker)
    or (d == "right" and right_blinker and not left_blinker)
  )
  if blinker_ok or (near_exec and slow_enough):
    return d
  return "none"


def nav_turn_keep_pulse_s(turn_dist_m: float) -> float:
  """Faster rising-edge pulses as the corner approaches (modeld edge-triggers)."""
  d = float(turn_dist_m or 0.0)
  if 0.0 < d <= NAV_CORNER_PULSE_M:
    return NAV_CORNER_PULSE_S
  if 0.0 < d <= NAV_NEAR_TURN_M:
    return NAV_APPROACH_PULSE_S
  return NAV_DEFAULT_PULSE_S


def nav_led_approach(snap: NavSnapshot, *, now: float | None = None) -> bool:
  """IQ-link ON: longitudinal approach without a blinker (150 m toast window)."""
  if not snapshot_executable(snap, now=now):
    return False
  if not nav_intersection_turn(snap):
    return False
  return 0.0 < float(snap.tbt_dist) <= TURN_DESIRE_WINDOW_M


def nav_near_matching_turn(*, left: bool, right: bool, snap: NavSnapshot) -> bool:
  if not nav_intersection_turn(snap):
    return False
  if not (0.0 < float(snap.tbt_dist) <= NAV_NEAR_TURN_M):
    return False
  d = _dir(snap.maneuver_dir)
  if left and d == "left":
    return True
  if right and d == "right":
    return True
  return False


def nav_long_blocked(gear) -> bool:
  """P/R: keep HUD, do not execute leftover nav speed / red."""
  name = str(getattr(gear, "name", gear) or "").split(".")[-1].lower()
  return name in ("park", "reverse")


def snapshot_long_ok(snap: NavSnapshot, gear=None, *, now: float | None = None) -> bool:
  if gear is not None and nav_long_blocked(gear):
    return False
  return snapshot_executable(snap, now=now)

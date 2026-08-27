"""Gating for navExitLeft/Right HUD. Import-light: no cereal/selfdrived."""

# Keep in sync with lane_change.NAV_EXIT_COMMIT_DISTANCE and cereal enums:
# IQNavState.ManeuverType.exit @2; CarState.GearShifter.park @1 reverse @4
NAV_EXIT_COMMIT_DISTANCE = 500.0
NAV_EXIT_ALERT_MIN_REMAIN_M = 200.0
NAV_EXIT_ALERT_DEDUP_S = 8.0
_MANEUVER_EXIT = 2
_GEAR_PARK = 1
_GEAR_REVERSE = 4


def _raw(v) -> int:
  return int(getattr(v, "raw", v) or 0)


def nav_exit_alert_allowed(nav_state, CS, now_mono: float, last_t: float, last_dir: int) -> tuple[bool, int]:
  if not getattr(nav_state, "active", False):
    return False, 0
  if not getattr(nav_state, "nextManeuverValid", False):
    return False, 0

  type_raw = _raw(getattr(nav_state, "nextManeuverType", 0))
  if type_raw != _MANEUVER_EXIT:
    return False, 0

  dist = float(getattr(nav_state, "nextManeuverDistance", 0.0) or 0.0)
  if not (0.0 < dist <= NAV_EXIT_COMMIT_DISTANCE):
    return False, 0

  remain_v = getattr(nav_state, "distanceRemaining", None)
  # 0 / missing = unset on the wire; only a positive remain near dest suppresses.
  remain = float("inf") if remain_v is None else float(remain_v)
  if 0.0 < remain <= NAV_EXIT_ALERT_MIN_REMAIN_M:
    return False, 0

  gear = getattr(CS, "gearShifter", None) if CS is not None else None
  if gear is not None:
    gear_raw = _raw(gear)
    if gear_raw in (_GEAR_REVERSE, _GEAR_PARK):
      return False, 0

  dir_raw = _raw(getattr(nav_state, "nextManeuverDirection", 0))
  if dir_raw not in (1, 2):
    return False, 0

  if dir_raw == last_dir and (now_mono - last_t) < NAV_EXIT_ALERT_DEDUP_S:
    return False, 0

  return True, dir_raw

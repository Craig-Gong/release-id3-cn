"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos

Engagement tiers for the speed-assist feature. A tier is persisted as an integer
under the "IQSpeedAssistMode" param; the ordinal IS the stored value and must remain
stable (0..3), ordered by how much the tier is allowed to intervene.
"""
from enum import IntEnum

STORE_KEY = "IQSpeedAssistMode"

# none -> just display the limit -> highlight overspeed -> move the set speed
SpeedAssistTier = IntEnum("SpeedAssistTier", "DISABLED ADVISORY ALERTING ACTUATING", start=0)

DEFAULT_TIER = SpeedAssistTier.ADVISORY


def actuates_speed(tier) -> bool:
  """Only the top tier is permitted to drive the cruise set speed."""
  return int(tier) == SpeedAssistTier.ACTUATING


def set_speed_follows_limit(params) -> bool:
  """Whether HUD MAX should track the accepted speed limit.

  Explicit `SLCSetSpeedToLimit` always wins. Otherwise, with IQ-link off and
  Control selected, treat it as on so camera/map limits can move set speed.
  IQ-link stays on the existing default (off) so nav set-speed is unchanged.
  """
  try:
    if params.get_bool("SLCSetSpeedToLimit"):
      return True
    if params.get_bool("IqlinkEnabled"):
      return False
    raw = params.get("IQSpeedAssistMode", return_default=True)
    if raw is None:
      return False
    if isinstance(raw, bytes):
      raw = raw.decode("utf-8")
    return actuates_speed(int(raw))
  except (TypeError, ValueError, AttributeError):
    return False

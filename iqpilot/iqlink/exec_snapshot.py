"""Keep last nav-long envelope when a later packet drops the road limit to 0."""
from __future__ import annotations

from typing import Any

EXEC_LONG_KEYS = (
  "targetSpeed",
  "targetSpeedValid",
  "speedTarget",
  "accelTarget",
  "valid",
  "longitudinalEngaged",
  "longitudinalProvider",
  "longitudinalState",
  "navSpeedTargetActive",
  "roadSpeedLimit",
  "roadSpeedLimitValid",
)


def preserve_exec_long(fields: dict[str, Any], prev: dict[str, Any] | None) -> dict[str, Any]:
  """Do not drop nav long when a dest/TBT packet arrives with speed 0."""
  if fields.get("longitudinalEngaged"):
    return fields
  if not prev or not prev.get("longitudinalEngaged"):
    return fields
  merged = dict(fields)
  for key in EXEC_LONG_KEYS:
    if key in prev:
      merged[key] = prev[key]
  return merged

"""Green / remainS release gate when following a lead at a traffic light.

Head car (no close lead): unchanged — remainS==1 immediate, green ~1 s dwell.
Follow car: navigation may say go, but hold until the lead moves or a timeout.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

# Planner tick (openpilot.common.realtime.DT_MDL).
_DT_MDL = 0.05

# Within this range we treat the scene as queue follow (not head car).
LEAD_QUEUE_M = 12.0
LEAD_MIN_D_M = 0.5

# Lead must be crawling before we release (stable window).
LEAD_GO_SPEED_MPS = 0.4
LEAD_GO_CONFIRM_S = 0.5

# Or distance opening vs last sample while nav says go.
LEAD_GAP_M = 0.3

# Do not wait forever behind a stuck lead / bad radar lock.
FOLLOW_TIMEOUT_S = 4.0

# Scheme 2: softer starting / E2E launch while still in the queue envelope.
FOLLOW_LEAD_START_ACCEL = 1.1
FOLLOW_LEAD_LAUNCH_V_EGO = 2.0


@dataclass(frozen=True)
class LeadSnapshot:
  present: bool
  d_rel: float
  v_lead: float


def _sm_get(sm: Any, key: str) -> Any:
  if isinstance(sm, dict):
    return sm.get(key)
  return sm[key]


def read_follow_lead(sm: Any) -> LeadSnapshot:
  """Radar-first with vision fallback for close stopped leads."""
  d_rel = 0.0
  v_lead = 0.0
  radar_ok = False
  try:
    lead = _sm_get(sm, "radarState").leadOne
    if bool(getattr(lead, "status", False)):
      d_rel = float(getattr(lead, "dRel", 0.0) or 0.0)
      v_lead = float(getattr(lead, "vLead", 0.0) or 0.0)
      radar_ok = LEAD_MIN_D_M < d_rel <= LEAD_QUEUE_M
  except Exception:
    radar_ok = False

  if radar_ok:
    return LeadSnapshot(True, d_rel, v_lead)

  try:
    model = _sm_get(sm, "modelV2")
    ml = model.leadsV3[0]
    if float(ml.prob) > 0.5:
      d_rel = float(ml.x[0])
      v_lead = float(ml.v[0])
      if LEAD_MIN_D_M < d_rel <= LEAD_QUEUE_M:
        return LeadSnapshot(True, d_rel, v_lead)
  except Exception:
    pass

  return LeadSnapshot(False, 0.0, 0.0)


def follow_lead_present(sm: Any) -> bool:
  return read_follow_lead(sm).present


def follow_lead_soft_launch(sm: Any, v_ego: float) -> bool:
  """Suppress E2E launch boost and high starting floor when queued behind a lead."""
  if v_ego > FOLLOW_LEAD_LAUNCH_V_EGO:
    return False
  return follow_lead_present(sm)


class GreenFollowLeadGate:
  """Stateful gate: nav_go is necessary but not sufficient when a close lead exists."""

  def __init__(self) -> None:
    self._nav_go_since: float | None = None
    self._lead_moving_s = 0.0
    self._drel_prev: float | None = None

  def reset(self) -> None:
    self._nav_go_since = None
    self._lead_moving_s = 0.0
    self._drel_prev = None

  def may_release(self, *, now: float, nav_go: bool, sm: Any) -> bool:
    if not nav_go:
      self._nav_go_since = None
      self._lead_moving_s = 0.0
      self._drel_prev = None
      return False

    lead = read_follow_lead(sm)
    if not lead.present:
      self.reset()
      return True

    if self._nav_go_since is None:
      self._nav_go_since = now

    if lead.v_lead >= LEAD_GO_SPEED_MPS:
      self._lead_moving_s += _DT_MDL
    else:
      self._lead_moving_s = 0.0

    gap_opening = False
    if self._drel_prev is not None and lead.d_rel > self._drel_prev + LEAD_GAP_M:
      gap_opening = True
    self._drel_prev = lead.d_rel

    if self._lead_moving_s >= LEAD_GO_CONFIRM_S:
      self.reset()
      return True
    if gap_opening:
      self.reset()
      return True
    if self._nav_go_since is not None and (now - self._nav_go_since) >= FOLLOW_TIMEOUT_S:
      self.reset()
      return True
    return False

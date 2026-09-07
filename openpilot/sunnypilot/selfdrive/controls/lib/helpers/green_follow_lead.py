"""Green / remainS==1 must not launch into a close stopped lead.

Head car (no close lead): remainS==1 is immediate after a short flicker
filter; APK green dwell is owned by StandstillHold (~1 s). Follow car: wait
for radar/vision lead motion or an opening gap. Close stopped queue (≤8 m)
has no timeout so a false go cannot dump into the bumper; 8–12 m still times
out in 4 s (false lock).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

_DT_MDL = 0.05

LEAD_QUEUE_M = 20.0
LEAD_CLOSE_M = 8.0
LEAD_MIN_D_M = 0.5
# Congestion: lead often creeps <0.4 before we used to release.
LEAD_GO_SPEED_MPS = 0.25
LEAD_GO_CONFIRM_S = 0.05
LEAD_GAP_M = 0.3
FOLLOW_TIMEOUT_S = 4.0
FOLLOW_LEAD_START_ACCEL = 1.5
FOLLOW_LEAD_LAUNCH_V_EGO = 2.5
VISION_LEAD_PROB = 0.5

# Settle ~3.5 m behind a stopped lead (LongitudinalTuning.stop_distance).
# Hold 3.5–5.0 m (no creep). Only creep when clearly farther than 5.0 m.
STOPPED_LEAD_V_MPS = 0.5
STOPPED_LEAD_GAP_M = 3.5
STOPPED_LEAD_CREEP_M = 5.0
STOPPED_LEAD_SOFT_M = 8.0
STOPPED_LEAD_HARD_A = -2.5
STOPPED_LEAD_SOFT_A = -1.5
STOPPED_LEAD_HOLD_A = -1.2
STOPPED_LEAD_CLOSE_A = 0.25
STOPPED_LEAD_CLOSE_V_MAX = 0.8
RADAR_TO_CAMERA_M = 1.52


@dataclass(frozen=True)
class LeadSnapshot:
  present: bool
  d_rel: float
  v_lead: float
  close_queue: bool


def _sm_get(sm: Any, key: str) -> Any:
  if isinstance(sm, dict):
    return sm.get(key)
  return sm[key]


def radar_lead_present(lead: Any) -> bool:
  try:
    return bool(getattr(lead, "present", False) or getattr(lead, "status", False))
  except Exception:
    return False


def _from_radar(sm: Any) -> LeadSnapshot | None:
  try:
    lead = _sm_get(sm, "radarState").leadOne
    if not radar_lead_present(lead):
      return None
    d_rel = float(getattr(lead, "dRel", 0.0) or 0.0)
    v_lead = float(getattr(lead, "vLead", 0.0) or 0.0)
    if not (LEAD_MIN_D_M < d_rel <= LEAD_QUEUE_M):
      return None
    return LeadSnapshot(True, d_rel, v_lead, d_rel <= LEAD_CLOSE_M)
  except Exception:
    return None


def _from_vision(sm: Any) -> LeadSnapshot | None:
  try:
    ml = _sm_get(sm, "modelV2").leadsV3[0]
    if float(ml.prob) <= VISION_LEAD_PROB:
      return None
    # leadsV3.x is camera-frame; align with radarState / bumper gap.
    d_rel = float(ml.x[0]) - RADAR_TO_CAMERA_M
    v_lead = float(ml.v[0])
    if not (LEAD_MIN_D_M < d_rel <= LEAD_QUEUE_M):
      return None
    return LeadSnapshot(True, d_rel, v_lead, d_rel <= LEAD_CLOSE_M)
  except Exception:
    return None


def read_follow_lead(sm: Any) -> LeadSnapshot:
  radar = _from_radar(sm)
  if radar is not None:
    return radar
  vision = _from_vision(sm)
  if vision is not None:
    return vision
  return LeadSnapshot(False, 0.0, 0.0, False)


def follow_lead_present(sm: Any) -> bool:
  return read_follow_lead(sm).present


def follow_lead_soft_launch(sm: Any, v_ego: float) -> bool:
  if v_ego > FOLLOW_LEAD_LAUNCH_V_EGO:
    return False
  return follow_lead_present(sm)


def apply_stopped_lead_gap(sm: Any, v_ego: float, a_target: float, should_stop: bool,
                           *, red_pin: bool = False, model_stop: bool = False) -> tuple[float, bool]:
  """Keep ~3.5 m behind a stopped lead; only creep when clearly too far (>5.0 m)."""
  # Never creep into a red / yellow nav stop — that fights standstill hold.
  if red_pin:
    return float(a_target), bool(should_stop)
  try:
    from openpilot.sunnypilot.nav.snapshot import read_snapshot, snapshot_executable
    snap = read_snapshot()
    if snapshot_executable(snap) and snap.stop_for_light:
      return float(a_target), bool(should_stop)
  except Exception:
    pass

  lead = read_follow_lead(sm)
  if not lead.present or lead.v_lead >= STOPPED_LEAD_V_MPS:
    return float(a_target), bool(should_stop)

  d_rel = lead.d_rel
  if d_rel <= LEAD_MIN_D_M:
    return float(a_target), bool(should_stop)

  # Vision red: a track past the model stop point is usually phantom /
  # cross-traffic past the line — do not creep toward it over the line.
  if model_stop:
    try:
      x_end = float(_sm_get(sm, "modelV2").position.x[-1])
      if d_rel >= max(x_end - 1.0, 0.0):
        return float(a_target), bool(should_stop)
    except Exception:
      pass

  if d_rel < STOPPED_LEAD_GAP_M:
    should_stop = True
    brake = STOPPED_LEAD_HARD_A if v_ego > 0.15 else STOPPED_LEAD_HOLD_A
    a_target = min(float(a_target), brake)
    return float(a_target), bool(should_stop)

  if d_rel < STOPPED_LEAD_CREEP_M:
    # Near the settle gap: pin, do not nudge into the bumper.
    should_stop = True
    brake = STOPPED_LEAD_SOFT_A if v_ego > 0.15 else STOPPED_LEAD_HOLD_A
    a_target = min(float(a_target), brake)
    return float(a_target), bool(should_stop)

  if d_rel < STOPPED_LEAD_SOFT_M:
    # Clearly too far behind a stopped bumper: very gentle close only.
    if v_ego > STOPPED_LEAD_CLOSE_V_MAX:
      a_target = min(float(a_target), STOPPED_LEAD_SOFT_A)
    else:
      should_stop = False
      a_target = min(max(float(a_target), STOPPED_LEAD_CLOSE_A * 0.5), STOPPED_LEAD_CLOSE_A)
    return float(a_target), bool(should_stop)

  return float(a_target), bool(should_stop)


class GreenFollowLeadGate:
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
      self.reset()
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

    # Close stopped bumper: wait for real motion. No 4 s timeout.
    if lead.close_queue and lead.v_lead < LEAD_GO_SPEED_MPS:
      return False

    if self._nav_go_since is not None and (now - self._nav_go_since) >= FOLLOW_TIMEOUT_S:
      self.reset()
      return True
    return False

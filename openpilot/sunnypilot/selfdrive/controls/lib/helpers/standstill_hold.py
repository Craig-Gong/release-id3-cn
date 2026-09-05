"""Hold after a vision stop, or IQ-link red, until a stable go.

Vision-only: ~1 s dwell after the model stops asking to stop.
IQ-link: remainS==1 is immediate only if GreenFollowLeadGate agrees;
APK green still dwells ~1 s, then the same lead gate. After a nav go,
sticky vision-stop does not re-arm until the car moves.
"""
from __future__ import annotations

from openpilot.sunnypilot.selfdrive.controls.lib.helpers.green_follow_lead import (
  FOLLOW_LEAD_LAUNCH_V_EGO,
  FOLLOW_LEAD_START_ACCEL,
  LEAD_GO_SPEED_MPS,
  STOPPED_LEAD_GAP_M,
  GreenFollowLeadGate,
  follow_lead_soft_launch,
  read_follow_lead,
)
from openpilot.sunnypilot.nav.snapshot import NavSnapshot, read_snapshot, snapshot_executable

_STANDSTILL_HOLD_RELEASE_S = 1.0
_STANDSTILL_HOLD_LEAD_RELEASE_S = 0.15
_DT_MDL = 0.05
_RELEASE_V_EGO = 2.0
_STANDSTILL_V = 0.3


class StandstillHold:
  def __init__(self):
    self.hold = False
    self.hold_s = 0.0
    self.hold_released = False
    self._follow = GreenFollowLeadGate()
    self._nav_go_latched = False

  def reset(self) -> None:
    self.hold = False
    self.hold_s = 0.0
    self.hold_released = False
    self._follow.reset()
    self._nav_go_latched = False

  def apply(self, should_stop: bool, a_target: float, v_ego: float, *,
            standstill: bool, gas: bool, model_stop: bool,
            sm=None, now: float | None = None) -> tuple[bool, float]:
    if gas or v_ego > _RELEASE_V_EGO:
      self.reset()
      return should_stop, a_target

    snap = read_snapshot() if sm is not None else NavSnapshot()
    nav_live = snapshot_executable(snap, now=now)
    follow_sm = sm if sm is not None else {}
    clock = float(now) if now is not None else 0.0
    lead = read_follow_lead(follow_sm)
    lead_rolling = bool(lead.present and lead.v_lead >= LEAD_GO_SPEED_MPS)
    closing_gap = bool(
      lead.present and lead.v_lead < LEAD_GO_SPEED_MPS and lead.d_rel > STOPPED_LEAD_GAP_M
    )

    at_rest = standstill or v_ego <= _STANDSTILL_V
    if not at_rest:
      self.hold_released = False
      self._nav_go_latched = False
      if self.hold:
        self.hold = False
        self.hold_s = 0.0
      return should_stop, a_target

    nav_red = bool(nav_live and snap.stop_for_light)
    remain_go = bool(nav_live and snap.remain_go)
    apk_green = bool(nav_live and snap.apk_green)
    nav_go = remain_go or apk_green
    follow_ok = self._follow.may_release(now=clock, nav_go=nav_go, sm=follow_sm)

    if nav_red and not (remain_go and follow_ok):
      self.hold = True
      self.hold_s = 0.0
      self.hold_released = False
      return True, min(float(a_target), 0.0)

    # Congestion: lead already rolling, or we are intentionally closing a
    # too-large gap behind a stopped bumper — do not keep the 1 s pin.
    if (lead_rolling or closing_gap) and not nav_red:
      release_s = _STANDSTILL_HOLD_LEAD_RELEASE_S if lead_rolling else 0.0
      if self.hold:
        self.hold_s += _DT_MDL
        if self.hold_s < release_s:
          return True, min(float(a_target), 0.0)
        self.hold = False
        self.hold_s = 0.0
        self.hold_released = True
      return should_stop, float(a_target)

    if remain_go and follow_ok:
      self.hold = False
      self.hold_s = 0.0
      self.hold_released = True
      self._nav_go_latched = True
      return False, float(a_target)

    if apk_green:
      if not follow_ok:
        self.hold = True
        return True, min(float(a_target), 0.0)
      if not self.hold_released:
        if not self.hold:
          self.hold = True
          self.hold_s = 0.0
        self.hold_s += _DT_MDL
        if self.hold_s < _STANDSTILL_HOLD_RELEASE_S:
          return True, min(float(a_target), 0.0)
        self.hold = False
        self.hold_released = True
        self._nav_go_latched = True
        return False, float(a_target)

    if self._nav_go_latched:
      # Green already released: do not re-pin on sticky vision stop.
      return should_stop, a_target

    arm = should_stop if self.hold_released else (should_stop or model_stop)
    if arm:
      self.hold = True
      self.hold_s = 0.0
    elif self.hold:
      self.hold_s += _DT_MDL
      if self.hold_s >= _STANDSTILL_HOLD_RELEASE_S:
        self.hold = False
        self.hold_released = True
    if self.hold:
      return True, min(float(a_target), 0.0)
    return should_stop, a_target


def apply_follow_launch(sm, v_ego: float, a_target: float) -> float:
  if v_ego > FOLLOW_LEAD_LAUNCH_V_EGO:
    return float(a_target)
  lead = read_follow_lead(sm)
  if not lead.present:
    return float(a_target)
  # Lead already rolling: do not keep the queued-launch cap.
  if lead.v_lead >= LEAD_GO_SPEED_MPS:
    return float(a_target)
  # Closing a too-large gap behind a still-stopped lead: allow the creep
  # accel from apply_stopped_lead_gap instead of the 1.x queue floor only.
  if lead.d_rel > STOPPED_LEAD_GAP_M:
    return float(a_target)
  if follow_lead_soft_launch(sm, v_ego):
    return min(float(a_target), FOLLOW_LEAD_START_ACCEL)
  return float(a_target)

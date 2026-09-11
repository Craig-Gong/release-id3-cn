"""Hold after a vision stop, or IQ-link red, until a stable go.

Vision-only: ~1 s dwell after the model stops asking to stop.
IQ-link: remainS==1 is immediate after a short flicker filter if
GreenFollowLeadGate agrees; APK green dwells ~1 s, then the same lead
gate. Sticky red keeps pinning briefly when BLE drops executable.
After a nav go, sticky vision-stop does not re-arm until the car moves.
"""
from __future__ import annotations

from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_stop_safety import radar_lead_departed
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.green_follow_lead import (
  FOLLOW_LEAD_GO_FLOOR_A,
  FOLLOW_LEAD_LAUNCH_V_EGO,
  FOLLOW_LEAD_START_ACCEL,
  LEAD_GO_SPEED_MPS,
  STOPPED_LEAD_CREEP_M,
  STOPPED_LEAD_GAP_M,
  GreenFollowLeadGate,
  follow_lead_soft_launch,
  lead_owns_nav_stop,
  read_follow_lead,
)
from openpilot.sunnypilot.nav.snapshot import NavSnapshot, read_snapshot, snapshot_executable
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_turn import nav_long_blocked, snapshot_long_ok

_STANDSTILL_HOLD_RELEASE_S = 1.0
_STANDSTILL_HOLD_LEAD_RELEASE_S = 0.15
_REMAIN_GO_CONFIRM_S = 0.15  # 3 frames @ 50 ms — filter single-packet false go
_STICKY_RED_TTL_S = 8.0
# MEB needs clearly positive accel for ANFAHREN; 0.4 felt like release-without-go.
_GO_LAUNCH_FLOOR_A = 0.9
_DT_MDL = 0.05
_RELEASE_V_EGO = 2.0
_STANDSTILL_V = 0.3


def left_arrow_red(snap: NavSnapshot) -> bool:
  """Left-turn TBT + red: keep pin even if approach curve still has speed."""
  return (
    str(snap.traffic_light or "").strip().lower() == "red"
    and bool(snap.send_turn)
    and str(snap.maneuver_dir or "") == "left"
    and not bool(snap.remain_go)
  )


def _right_turn_pending(snap: NavSnapshot) -> bool:
  # Match protocol RTOR: near right turn must not sticky-arm a red pin.
  return (
    str(snap.maneuver or "") == "turn"
    and str(snap.maneuver_dir or "") == "right"
    and 0.0 < float(snap.tbt_dist or 0.0) <= 150.0
  )


class StandstillHold:
  def __init__(self):
    self.hold = False
    self.hold_s = 0.0
    self.hold_released = False
    self._follow = GreenFollowLeadGate()
    self._nav_go_latched = False
    self.sticky_red = False
    self._sticky_until = 0.0
    self._remain_go_s = 0.0
    self.red_pin = False

  def reset(self) -> None:
    self.hold = False
    self.hold_s = 0.0
    self.hold_released = False
    self._follow.reset()
    self._nav_go_latched = False
    self.sticky_red = False
    self._sticky_until = 0.0
    self._remain_go_s = 0.0
    self.red_pin = False

  def _clear_sticky(self) -> None:
    self.sticky_red = False
    self._sticky_until = 0.0

  def observe_nav(self, snap: NavSnapshot, now: float, *, gas: bool, v_ego: float,
                  gear=None, sm=None) -> None:
    """Arm / expire sticky red. Speed-limit stale rules stay separate."""
    if gas or v_ego > _RELEASE_V_EGO or not snap.iqlink_enabled or nav_long_blocked(gear):
      self._clear_sticky()
      self.red_pin = False
      self._remain_go_s = 0.0
      return

    # Queue behind a stopped lead short of the light: follow lead, not nav pin.
    if sm is not None and lead_owns_nav_stop(sm, snap):
      self._clear_sticky()
      self.red_pin = False
      self._remain_go_s = 0.0
      return

    live = snapshot_executable(snap, now=now)
    live_red = bool(live and snap.stop_for_light)
    live_left = bool(live and left_arrow_red(snap))
    # Never sticky-arm across an active RTOR exemption.
    if (live_red or live_left) and not _right_turn_pending(snap):
      self.sticky_red = True
      self._sticky_until = float(now) + _STICKY_RED_TTL_S
    elif self.sticky_red and float(now) > self._sticky_until:
      self._clear_sticky()

    self.red_pin = bool(
      (live_red or live_left or self.sticky_red) and not _right_turn_pending(snap)
    )

  def apply(self, should_stop: bool, a_target: float, v_ego: float, *,
            standstill: bool, gas: bool, model_stop: bool,
            sm=None, now: float | None = None) -> tuple[bool, float]:
    if gas or v_ego > _RELEASE_V_EGO:
      self.reset()
      return should_stop, a_target

    clock = float(now) if now is not None else 0.0
    snap = read_snapshot() if sm is not None else NavSnapshot()
    try:
      gear = sm['carState'].gearShifter if sm is not None else None
    except Exception:
      gear = None
    self.observe_nav(snap, clock, gas=False, v_ego=v_ego, gear=gear, sm=sm)

    nav_live = snapshot_long_ok(snap, gear, now=clock)
    follow_sm = sm if sm is not None else {}
    lead = read_follow_lead(follow_sm)
    lead_rolling = bool(lead.present and lead.v_lead >= LEAD_GO_SPEED_MPS)
    closing_gap = bool(
      lead.present and lead.v_lead < LEAD_GO_SPEED_MPS and lead.d_rel > STOPPED_LEAD_CREEP_M
    )

    # remainS==1 must be live; require ~3 frames to filter a single false packet.
    if nav_live and snap.remain_go:
      self._remain_go_s += _DT_MDL
    else:
      self._remain_go_s = 0.0
    remain_go = self._remain_go_s >= _REMAIN_GO_CONFIRM_S

    apk_green = bool(nav_live and snap.apk_green)
    nav_go = remain_go or apk_green
    follow_ok = self._follow.may_release(now=clock, nav_go=nav_go, sm=follow_sm)

    # Explicit go clears sticky so a fresh green is not re-pinned.
    if remain_go and follow_ok:
      self._clear_sticky()
      self.red_pin = False
      self.hold = False
      self.hold_s = 0.0
      self.hold_released = True
      self._nav_go_latched = True
      return False, max(float(a_target), _GO_LAUNCH_FLOOR_A)

    # Nav / sticky red (no confirmed go): pin including while creeping.
    if self.red_pin and not (remain_go and follow_ok):
      self.hold = True
      self.hold_s = 0.0
      self.hold_released = False
      self._nav_go_latched = False
      return True, min(float(a_target), -1.0)

    at_rest = standstill or v_ego <= _STANDSTILL_V
    if not at_rest:
      self.hold_released = False
      self._nav_go_latched = False
      if self.hold:
        self.hold = False
        self.hold_s = 0.0
      return should_stop, a_target

    # Radar, not the nav bar: lead has left the settle gap and is really moving.
    # Do not wait for 8 m or a missing green packet. still honor a live red pin.
    if radar_lead_departed(follow_sm) and not self.red_pin:
      self.hold = False
      self.hold_s = 0.0
      self.hold_released = True
      self._nav_go_latched = True
      a_out = float(a_target)
      if v_ego <= _STANDSTILL_V + 0.5:
        a_out = max(a_out, _GO_LAUNCH_FLOOR_A)
      return False, a_out

    # Congestion: lead already rolling, or closing a too-large gap.
    if lead_rolling or closing_gap:
      # Nose-to-bumper only. 8 m was "a short lead start" and left ego sitting.
      if lead.present and lead.d_rel < STOPPED_LEAD_GAP_M:
        self.hold = True
        self.hold_released = False
        return True, min(float(a_target), 0.0)
      release_s = _STANDSTILL_HOLD_LEAD_RELEASE_S if lead_rolling else 0.0
      if self.hold:
        self.hold_s += _DT_MDL
        if self.hold_s < release_s:
          return True, min(float(a_target), 0.0)
        self.hold = False
        self.hold_s = 0.0
        self.hold_released = True
      a_out = float(a_target)
      # Lead rolling with a usable gap. Under CREEP: radar noise is not a go —
      # leave accel to gap brake, do not add a positive floor.
      if lead_rolling and lead.d_rel >= STOPPED_LEAD_CREEP_M:
        a_out = max(a_out, _GO_LAUNCH_FLOOR_A)
      return should_stop, a_out

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
        self._clear_sticky()
        self.red_pin = False
        return False, max(float(a_target), _GO_LAUNCH_FLOOR_A)

    if self._nav_go_latched:
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
  # Past settle gap and lead actually rolling (~3 km/h): follow, don't wait for 5 m.
  if lead.d_rel >= STOPPED_LEAD_GAP_M and lead.v_lead >= 0.8:
    return max(float(a_target), FOLLOW_LEAD_GO_FLOOR_A)
  # Critically closed: never add a positive floor. Stopped lead also forbids accel.
  if lead.d_rel < STOPPED_LEAD_CREEP_M:
    if lead.v_lead < LEAD_GO_SPEED_MPS:
      return min(float(a_target), 0.0)
    return float(a_target)
  if lead.v_lead >= LEAD_GO_SPEED_MPS:
    return max(float(a_target), FOLLOW_LEAD_GO_FLOOR_A)
  if follow_lead_soft_launch(sm, v_ego):
    return min(float(a_target), FOLLOW_LEAD_START_ACCEL)
  return float(a_target)

"""Congestion follow takeoff: soft floor, accel slew, lead-decel anticipate."""
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_follow_comfort import (
  ACCEL_JERK_ONSET, FOLLOW_FLOOR_SOFT_A, LAUNCH_CONFIRM_S, LAUNCH_CONFIRM_V,
  AccelOnsetLimiter, FollowGoConfirm, FollowLaunchController,
  compute_anticipate_cap, compute_follow_floor,
)
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.standstill_hold import (
  apply_follow_launch,
)


DT = 0.05


def _sm(d_rel=6.0, v_lead=0.8, a_lead=0.0, present=True):
  lead = SimpleNamespace(
    present=present, status=present, dRel=float(d_rel),
    vLead=float(v_lead), aLeadK=float(a_lead),
  )
  return {
    "radarState": SimpleNamespace(leadOne=lead),
    "modelV2": SimpleNamespace(leadsV3=[]),
  }


def test_floor_needs_confirm_and_matches_lead_accel():
  # Not confirmed → no floor.
  assert compute_follow_floor(0.5, 6.0, 0.8, 0.5, confirmed=False) is None
  # Confirmed, lead accelerating, gap opening → soft floor below the old 1.2 stamp.
  a = compute_follow_floor(0.5, 6.0, 0.8, 0.5, confirmed=True)
  assert a is not None
  assert 0.3 <= a <= FOLLOW_FLOOR_SOFT_A
  # Lead decelerating → kill floor (phase 3 path will also cap).
  assert compute_follow_floor(0.5, 6.0, 0.8, -0.5, confirmed=True) is None
  # Closing hard on the bumper → kill floor.
  assert compute_follow_floor(1.4, 6.0, 0.8, 0.0, confirmed=True) is None


def test_confirm_hysteresis():
  g = FollowGoConfirm()
  assert not g.update(0.3, DT)
  for _ in range(int(LAUNCH_CONFIRM_S / DT) + 1):
    g.update(LAUNCH_CONFIRM_V, DT)
  assert g.confirmed
  # Band between go-speed and confirm keeps confirmed.
  assert g.update(0.35, DT)
  # Lead stops → drop.
  assert not g.update(0.0, DT)
  assert not g.confirmed


def test_anticipate_coasts_when_lead_brakes():
  assert compute_anticipate_cap(3.0, 10.0, 3.0, 0.0) is None
  soft = compute_anticipate_cap(3.0, 10.0, 3.0, -0.5)
  assert soft is not None and soft <= 0.0
  hard = compute_anticipate_cap(3.0, 10.0, 3.0, -1.0)
  assert hard is not None and hard <= -0.3
  # Above congestion window: no anticipate.
  assert compute_anticipate_cap(10.0, 10.0, 10.0, -1.0) is None


def test_apply_follow_launch_no_longer_stamps_1_2():
  ctrl = FollowLaunchController()
  sm = _sm(d_rel=6.0, v_lead=0.8, a_lead=0.2)
  # Warm confirm while gap is opening (ego slower than lead).
  for _ in range(int(LAUNCH_CONFIRM_S / DT) + 2):
    apply_follow_launch(sm, 0.5, 0.0, dt=DT, controller=ctrl)
  a = apply_follow_launch(sm, 0.5, 0.0, dt=DT, controller=ctrl)
  assert 0.0 < a <= FOLLOW_FLOOR_SOFT_A
  assert a < 1.2


def test_apply_follow_launch_pause_then_restart_does_not_jab():
  """Lead go → stop → go: floor drops on pause; restart needs re-confirm."""
  ctrl = FollowLaunchController()
  # Lead rolling, gap opening.
  sm = _sm(d_rel=6.5, v_lead=0.9, a_lead=0.3)
  for _ in range(int(LAUNCH_CONFIRM_S / DT) + 2):
    apply_follow_launch(sm, 0.5, 0.0, dt=DT, controller=ctrl)
  a_go = apply_follow_launch(sm, 0.5, 0.0, dt=DT, controller=ctrl)
  assert a_go > 0.2
  # Lead pauses (and brakes).
  sm_stop = _sm(d_rel=6.2, v_lead=0.0, a_lead=-0.6)
  a_pause = apply_follow_launch(sm_stop, 0.6, a_go, dt=DT, controller=ctrl)
  assert a_pause <= 0.0
  # Restart: first frames must not instantly re-stamp a high floor.
  sm_go2 = _sm(d_rel=6.5, v_lead=0.55, a_lead=0.2)
  a_early = apply_follow_launch(sm_go2, 0.3, 0.0, dt=DT, controller=ctrl)
  assert a_early < 0.5


def test_close_stopped_lead_forbids_accel():
  ctrl = FollowLaunchController()
  sm = _sm(d_rel=4.0, v_lead=0.0, a_lead=0.0)
  a = apply_follow_launch(sm, 0.6, 0.8, dt=DT, controller=ctrl)
  assert a <= 0.0


def test_accel_onset_slews_and_bypasses_standstill_path():
  lim = AccelOnsetLimiter()
  lim.a_prev = 0.0
  # One frame of +1.2 would be the old jab; slew caps the step.
  out = lim.update(1.2, v_ego=1.0, dt=DT, bypass=False)
  assert out <= ACCEL_JERK_ONSET * DT + 1e-6
  assert out < 0.2
  # Bypass (standstill go) passes the MEB floor in one frame.
  lim2 = AccelOnsetLimiter()
  lim2.a_prev = -1.0
  assert lim2.update(0.9, v_ego=0.0, dt=DT, bypass=True) == 0.9


def test_accel_drop_is_free():
  lim = AccelOnsetLimiter()
  lim.a_prev = 0.8
  assert lim.update(0.0, v_ego=1.0, dt=DT, bypass=False) == 0.0

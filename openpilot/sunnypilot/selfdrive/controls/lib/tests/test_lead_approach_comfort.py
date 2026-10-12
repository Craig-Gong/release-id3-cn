"""Lead Approach Comfort: far deadband, lift/coast, follow accel cap."""
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_approach_comfort import (
  COAST_A, ENGAGE_REQ, FOLLOW_OPEN_JERK, LIFT_A, LOCK_FADE_S, LeadApproachComfort,
  approach_a_req, approach_accel_profile, follow_accel_cap,
)
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_follow_comfort import (
  compute_anticipate_cap,
)


DT = 0.05


def _sm(d_rel=40.0, v_lead=10.0, a_lead=0.0, present=True):
  lead = SimpleNamespace(
    present=present, status=present, dRel=float(d_rel),
    vLead=float(v_lead), aLeadK=float(a_lead),
  )
  return {
    "radarState": SimpleNamespace(leadOne=lead),
    "modelV2": SimpleNamespace(leadsV3=[]),
  }


def test_far_mild_close_does_not_prebrake():
  # 80→72 @60 m / 50→45 @40 m: long TTC, tiny a_req → leave to MPC.
  assert approach_accel_profile(22.2, 60.0, 20.0) is None
  assert approach_accel_profile(13.9, 40.0, 12.5) is None
  assert approach_accel_profile(20.0, 50.0, 16.0) is None


def test_engage_lift_is_zero_not_minus_point_four():
  # Closing enough to engage, but still in lift band → cap 0 (ease off only).
  a = approach_accel_profile(14.0, 31.0, 10.0)
  assert a is not None
  assert abs(a - LIFT_A) < 1e-6
  assert a == 0.0


def test_hard_close_tracks_physics():
  a = approach_accel_profile(20.0, 12.0, 5.0)
  assert a is not None
  assert a <= COAST_A
  assert a >= -3.5


def test_lock_fade_starts_soft():
  # Lift-band profile (0): first half-second softens a speculative MPC bite.
  lac = LeadApproachComfort()
  sm = _sm(d_rel=31.0, v_lead=10.0, a_lead=0.0)
  a0 = lac.apply(sm, 14.0, -2.5, dt=DT, gas=False, light_stop=False)
  assert a0 >= COAST_A - 1e-6
  # Still inside fade_w < 0.5 (~1 s).
  for _ in range(8):
    a_mid = lac.apply(sm, 14.0, -2.5, dt=DT)
  assert a_mid >= COAST_A - 1e-6


def test_follow_accel_cap_blocks_punch():
  lac = LeadApproachComfort()
  sm = _sm(d_rel=25.0, v_lead=22.0, a_lead=0.5)
  a = lac.apply(sm, 18.0, 2.0, dt=DT)
  assert a <= follow_accel_cap(18.0) + 1e-6
  assert lac.open_jerk == FOLLOW_OPEN_JERK
  # Raised cap vs old 0.62-ish at 18 m/s.
  assert follow_accel_cap(18.0) >= 0.75


def test_gas_and_light_skip_lac():
  lac = LeadApproachComfort()
  sm = _sm(d_rel=20.0, v_lead=10.0)
  assert lac.apply(sm, 15.0, 1.5, gas=True) == 1.5
  assert lac.apply(sm, 15.0, -2.0, light_stop=True) == -2.0


def test_urgent_cut_in_not_stuck_in_coast():
  lac = LeadApproachComfort()
  sm = _sm(d_rel=15.0, v_lead=5.0, a_lead=0.0)
  a = lac.apply(sm, 15.0, -3.0, dt=DT)
  assert a < COAST_A


def test_deep_profile_cut_in_not_held_at_coast():
  # TTC just above urgent (~3.1 s) but profile already −3.5: first frame must
  # not clamp to COAST (stop_safety + LAC used to delay ~1 s).
  lac = LeadApproachComfort()
  sm = _sm(d_rel=25.0, v_lead=12.0, a_lead=0.0)
  a = lac.apply(sm, 20.0, -3.5, dt=DT)
  assert a < COAST_A - 0.5
  assert a <= -2.0


def test_anticipate_mid_speed():
  soft = compute_anticipate_cap(15.0, 40.0, 15.0, -0.5)
  assert soft is not None and soft <= 0.0
  assert compute_anticipate_cap(15.0, 120.0, 15.0, -1.0) is None


def test_a_req_zero_when_opening():
  assert approach_a_req(10.0, 30.0, 12.0) == 0.0
  assert approach_accel_profile(10.0, 30.0, 12.0) is None


def test_engage_threshold_documented():
  assert ENGAGE_REQ == 0.45
  assert LIFT_A == 0.0

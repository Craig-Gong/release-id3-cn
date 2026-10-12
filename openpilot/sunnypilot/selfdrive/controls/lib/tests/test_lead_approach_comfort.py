"""Lead Approach Comfort: coast-first close, lock fade, follow accel cap."""
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_approach_comfort import (
  COAST_A, FOLLOW_OPEN_JERK, LIFT_A, LOCK_FADE_S, LeadApproachComfort,
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


def test_soft_close_is_lift_or_coast_not_hard_bite():
  # 20 m/s ego, 16 m/s lead, 50 m → mild a_req → lift/coast band.
  a = approach_accel_profile(20.0, 50.0, 16.0)
  assert a is not None
  assert LIFT_A - 1e-6 <= a <= 0.0 or COAST_A - 0.05 <= a <= LIFT_A + 0.05
  assert a > -1.5


def test_hard_close_tracks_physics():
  # Fast close on short slack → firm brake, still above hard floor.
  a = approach_accel_profile(20.0, 12.0, 5.0)
  assert a is not None
  assert a <= COAST_A
  assert a >= -3.5


def test_lock_fade_starts_soft():
  lac = LeadApproachComfort()
  sm = _sm(d_rel=35.0, v_lead=12.0, a_lead=0.0)
  # MPC already asked for a hard bite.
  a0 = lac.apply(sm, 18.0, -2.5, dt=DT, gas=False, light_stop=False)
  assert a0 >= COAST_A - 1e-6
  # After fade, may follow a deeper profile.
  for _ in range(int(LOCK_FADE_S / DT) + 2):
    a_end = lac.apply(sm, 18.0, -2.5, dt=DT)
  assert a_end <= a0 + 1e-6


def test_follow_accel_cap_blocks_punch():
  lac = LeadApproachComfort()
  sm = _sm(d_rel=25.0, v_lead=22.0, a_lead=0.5)  # opening
  a = lac.apply(sm, 18.0, 2.0, dt=DT)
  assert a <= follow_accel_cap(18.0) + 1e-6
  assert lac.open_jerk == FOLLOW_OPEN_JERK


def test_gas_and_light_skip_lac():
  lac = LeadApproachComfort()
  sm = _sm(d_rel=20.0, v_lead=10.0)
  assert lac.apply(sm, 15.0, 1.5, gas=True) == 1.5
  assert lac.apply(sm, 15.0, -2.0, light_stop=True) == -2.0


def test_urgent_cut_in_not_stuck_in_coast():
  lac = LeadApproachComfort()
  # TTC ~1.5 s at 10 m closing from 15 m → urgent.
  sm = _sm(d_rel=15.0, v_lead=5.0, a_lead=0.0)
  a = lac.apply(sm, 15.0, -3.0, dt=DT)
  assert a < COAST_A


def test_anticipate_mid_speed():
  # Was gated at 7 m/s; now active around 15 m/s / 40 m.
  soft = compute_anticipate_cap(15.0, 40.0, 15.0, -0.5)
  assert soft is not None and soft <= 0.0
  # Too far for speed → still None.
  assert compute_anticipate_cap(15.0, 120.0, 15.0, -1.0) is None


def test_a_req_zero_when_opening():
  assert approach_a_req(10.0, 30.0, 12.0) == 0.0
  assert approach_accel_profile(10.0, 30.0, 12.0) is None

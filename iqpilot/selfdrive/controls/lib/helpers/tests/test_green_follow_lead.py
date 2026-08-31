from types import SimpleNamespace

DT_MDL = 0.05

from iqpilot.selfdrive.controls.lib.helpers.green_follow_lead import (
  CLOSE_HOLD_M,
  FOLLOW_TIMEOUT_S,
  LEAD_GO_CONFIRM_S,
  LEAD_GO_SPEED_MPS,
  VISION_LEAD_PROB,
  GreenFollowLeadGate,
  follow_lead_present,
  follow_lead_soft_launch,
  read_follow_lead,
)


def _sm(*, lead_status=False, lead_present=None, d_rel=6.0, v_lead=0.0, vision_prob=0.0, vision_x=50.0):
  present = lead_status if lead_present is None else lead_present
  return {
    "radarState": SimpleNamespace(
      leadOne=SimpleNamespace(status=lead_status, present=present, dRel=d_rel, vLead=v_lead),
    ),
    "modelV2": SimpleNamespace(leadsV3=[SimpleNamespace(prob=vision_prob, x=[vision_x], v=[0.0])]),
  }


def test_no_lead_always_release():
  gate = GreenFollowLeadGate()
  sm = _sm()
  assert not follow_lead_present(sm)
  assert gate.may_release(now=0.0, nav_go=True, sm=sm)


def test_follow_lead_blocks_until_moving():
  gate = GreenFollowLeadGate()
  sm = _sm(lead_status=True, d_rel=5.0, v_lead=0.0)
  assert follow_lead_present(sm)
  assert not gate.may_release(now=0.0, nav_go=True, sm=sm)

  t = 0.0
  sm_move = _sm(lead_status=True, d_rel=5.2, v_lead=LEAD_GO_SPEED_MPS + 0.1)
  released = False
  for _ in range(int((LEAD_GO_CONFIRM_S + 0.1) / DT_MDL)):
    t += DT_MDL
    if gate.may_release(now=t, nav_go=True, sm=sm_move):
      released = True
      break
  assert released


def test_close_lead_timeout_does_not_release():
  gate = GreenFollowLeadGate()
  sm = _sm(lead_status=True, d_rel=4.0, v_lead=0.0)
  assert not gate.may_release(now=0.0, nav_go=True, sm=sm)
  assert not gate.may_release(now=FOLLOW_TIMEOUT_S + 0.1, nav_go=True, sm=sm)


def test_far_lead_timeout_releases():
  gate = GreenFollowLeadGate()
  sm = _sm(lead_status=True, d_rel=CLOSE_HOLD_M + 1.5, v_lead=0.0)
  assert not gate.may_release(now=0.0, nav_go=True, sm=sm)
  assert gate.may_release(now=FOLLOW_TIMEOUT_S + 0.1, nav_go=True, sm=sm)


def test_radar_present_flag_counts():
  snap = read_follow_lead(_sm(lead_status=False, lead_present=True, d_rel=5.0))
  assert snap.present
  assert snap.d_rel == 5.0


def test_vision_fallback_close_lead():
  snap = read_follow_lead(_sm(lead_status=False, vision_prob=0.8, vision_x=7.0))
  assert snap.present
  assert snap.d_rel == 7.0


def test_vision_fallback_accepts_borderline_prob():
  snap = read_follow_lead(_sm(lead_status=False, vision_prob=VISION_LEAD_PROB + 0.01, vision_x=6.0))
  assert snap.present


def test_soft_launch_only_when_queued():
  sm = _sm(lead_status=True, d_rel=5.0)
  assert follow_lead_soft_launch(sm, 0.0)
  assert not follow_lead_soft_launch(sm, 3.0)
  assert not follow_lead_soft_launch(_sm(), 0.0)

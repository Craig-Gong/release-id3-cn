from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.controls.lib.helpers.green_follow_lead import (
  FOLLOW_TIMEOUT_S,
  GreenFollowLeadGate,
  read_follow_lead,
)

DT_MDL = 0.05


def _sm(d_rel=0.0, v_lead=0.0, radar=True, vision=None):
  on = bool(radar and d_rel > 0)
  lead = SimpleNamespace(present=on, status=on, dRel=d_rel, vLead=v_lead)
  radar_state = SimpleNamespace(leadOne=lead)
  if vision is None:
    model = SimpleNamespace(leadsV3=[SimpleNamespace(prob=0.0, x=[0.0], v=[0.0])])
  else:
    d, v, p = vision
    model = SimpleNamespace(leadsV3=[SimpleNamespace(prob=p, x=[d], v=[v])])
  return {"radarState": radar_state, "modelV2": model}


def test_head_car_releases_immediately():
  g = GreenFollowLeadGate()
  assert g.may_release(now=1.0, nav_go=True, sm=_sm(0.0, 0.0, radar=False))


def test_close_stopped_lead_no_timeout():
  g = GreenFollowLeadGate()
  sm = _sm(5.0, 0.0)
  now = 10.0
  assert g.may_release(now=now, nav_go=True, sm=sm) is False
  ticks = int(FOLLOW_TIMEOUT_S / DT_MDL) + 5
  for i in range(ticks):
    assert g.may_release(now=now + i * DT_MDL, nav_go=True, sm=sm) is False


def test_close_lead_releases_when_moving():
  g = GreenFollowLeadGate()
  sm = _sm(5.0, 0.0)
  g.may_release(now=1.0, nav_go=True, sm=sm)
  moving = _sm(5.0, 0.5)
  released = False
  for i in range(int(0.5 / DT_MDL) + 2):
    if g.may_release(now=2.0 + i * DT_MDL, nav_go=True, sm=moving):
      released = True
      break
  assert released is True


def test_close_lead_releases_on_gap_opening():
  g = GreenFollowLeadGate()
  g.may_release(now=1.0, nav_go=True, sm=_sm(5.0, 0.0))
  assert g.may_release(now=1.1, nav_go=True, sm=_sm(5.4, 0.0)) is True


def test_far_queue_times_out():
  g = GreenFollowLeadGate()
  sm = _sm(10.0, 0.0)
  now = 5.0
  g.may_release(now=now, nav_go=True, sm=sm)
  assert g.may_release(now=now + FOLLOW_TIMEOUT_S, nav_go=True, sm=sm) is True


def test_vision_fallback_close_lead():
  sm = _sm(0.0, 0.0, radar=False, vision=(6.0, 0.0, 0.8))
  lead = read_follow_lead(sm)
  assert lead.present
  assert lead.close_queue


def test_radar_present_field_without_status():
  lead = SimpleNamespace(present=True, dRel=6.0, vLead=0.0)
  sm = {
    "radarState": SimpleNamespace(leadOne=lead),
    "modelV2": SimpleNamespace(leadsV3=[SimpleNamespace(prob=0.0, x=[0.0], v=[0.0])]),
  }
  got = read_follow_lead(sm)
  assert got.present
  assert got.close_queue

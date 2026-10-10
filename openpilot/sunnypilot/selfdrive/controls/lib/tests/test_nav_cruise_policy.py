"""NavCruisePolicy gates — camera / dest / dual-TBT / congestion / light."""
from __future__ import annotations

import math

from openpilot.common.constants import CV
from openpilot.sunnypilot.nav.hud_copy import APPROACH_DEST, CAMERA_AHEAD, INTERVAL_SPEED
from openpilot.sunnypilot.nav.snapshot import NavSnapshot
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_cruise_policy import (
  CONG_A_POS, DEST_A_POS, GREEN_STANDSTILL_A, NavCruisePolicy, SDI_INTERVAL_END,
  SDI_INTERVAL_START, SDI_POINT, apply_a_pos_cap,
)


def _live(**kwargs) -> NavSnapshot:
  base = dict(
    ts=100.0,
    link_ok=True,
    link_state=2,
    iqlink_enabled=True,
    road_limit_kph=50.0,
    go_dist_m=5000.0,
    go_time_s=600.0,
  )
  base.update(kwargs)
  return NavSnapshot(**base)


def _dec(pol: NavCruisePolicy, snap: NavSnapshot, *, v_kph=40.0, now=100.0,
         near_lead=False, standstill=False, gas=False, gear="drive"):
  # Keep HMAC freshness: snapshot_long_ok rejects age > STALE_LINK_S.
  snap.ts = float(now)
  return pol.update(
    snap, v_kph * CV.KPH_TO_MS,
    gear=gear, has_near_lead=near_lead, standstill=standstill,
    gas=gas, long_enabled=True, now=now,
  )


def test_camera_point_caps_at_limit():
  pol = NavCruisePolicy()
  snap = _live(sdi_type=SDI_POINT, sdi_dist_m=180.0, sdi_speed_kph=60.0, road_limit_kph=80.0)
  d = _dec(pol, snap, v_kph=70.0, now=100.0)
  assert d.v_cap_ms is not None
  assert abs(d.v_cap_ms - 60.0 * CV.KPH_TO_MS) < 1e-3
  assert d.toast == CAMERA_AHEAD
  assert "camera" in d.mode


def test_camera_uses_road_when_sdi_speed_missing():
  pol = NavCruisePolicy()
  snap = _live(sdi_type=SDI_POINT, sdi_dist_m=120.0, sdi_speed_kph=0.0, road_limit_kph=50.0)
  d = _dec(pol, snap, now=100.0)
  assert abs(d.v_cap_ms - 50.0 * CV.KPH_TO_MS) < 1e-3


def test_camera_ignores_far_and_whitelist_only_types():
  pol = NavCruisePolicy()
  far = _live(sdi_type=SDI_POINT, sdi_dist_m=400.0, sdi_speed_kph=60.0)
  assert _dec(pol, far, now=100.0).v_cap_ms is None
  junk = _live(sdi_type=1, sdi_dist_m=80.0, sdi_speed_kph=60.0)
  assert _dec(pol, junk, now=101.0).v_cap_ms is None


def test_interval_latches_until_end_or_timeout():
  pol = NavCruisePolicy()
  start = _live(sdi_type=SDI_INTERVAL_START, sdi_speed_kph=80.0, road_limit_kph=100.0)
  d0 = _dec(pol, start, now=200.0)
  assert abs(d0.v_cap_ms - 80.0 * CV.KPH_TO_MS) < 1e-3
  assert d0.toast == INTERVAL_SPEED

  mid = _live(sdi_type=-1, sdi_speed_kph=0.0, road_limit_kph=100.0)
  d1 = _dec(pol, mid, now=210.0)
  assert abs(d1.v_cap_ms - 80.0 * CV.KPH_TO_MS) < 1e-3

  end = _live(sdi_type=SDI_INTERVAL_END, road_limit_kph=100.0)
  d2 = _dec(pol, end, now=220.0)
  assert d2.v_cap_ms is None

  # Restart then timeout
  _dec(pol, start, now=300.0)
  d3 = _dec(pol, mid, now=300.0 + 181.0)
  assert d3.v_cap_ms is None


def test_dest_last_500m_dulls_accel_urban():
  pol = NavCruisePolicy()
  snap = _live(go_dist_m=300.0, road_limit_kph=40.0, maneuver="none")
  d = _dec(pol, snap, v_kph=35.0, now=100.0)
  assert d.a_pos_cap == DEST_A_POS
  assert abs(d.v_cap_ms - 40.0 * CV.KPH_TO_MS) < 1e-3
  assert d.toast == APPROACH_DEST


def test_dest_skipped_on_highway_without_hint():
  pol = NavCruisePolicy()
  snap = _live(go_dist_m=300.0, road_limit_kph=100.0, maneuver="none")
  d = _dec(pol, snap, v_kph=90.0, now=100.0)
  assert d.a_pos_cap is None
  assert "dest" not in d.mode


def test_dest_hint_parking_name_triggers():
  pol = NavCruisePolicy()
  snap = _live(go_dist_m=400.0, road_limit_kph=80.0, goal_name="万达停车场B2", maneuver="none")
  d = _dec(pol, snap, v_kph=50.0, now=100.0)
  assert d.a_pos_cap == DEST_A_POS


def test_near_lead_keeps_camera_skips_a_pos():
  pol = NavCruisePolicy()
  # Far destination so dest a_pos would fire without a lead; camera still caps.
  snap = _live(
    sdi_type=SDI_POINT, sdi_dist_m=150.0, sdi_speed_kph=60.0,
    go_dist_m=9000.0, road_limit_kph=80.0,
  )
  d = _dec(pol, snap, near_lead=True, now=100.0)
  assert abs(d.v_cap_ms - 60.0 * CV.KPH_TO_MS) < 1e-3
  assert d.a_pos_cap is None


def test_gas_bypasses_a_pos_not_v_cap():
  pol = NavCruisePolicy()
  snap = _live(go_dist_m=200.0, road_limit_kph=40.0)
  d = _dec(pol, snap, gas=True, now=100.0)
  assert d.a_pos_cap is None
  assert d.v_cap_ms is not None


def test_dual_tbt_soft_cap():
  pol = NavCruisePolicy()
  # next turn type 2 = right
  snap = _live(
    tbt_dist=80.0, tbt_dist_next=200.0, tbt_type_next=2,
    road_limit_kph=50.0, go_dist_m=8000.0,
  )
  d = _dec(pol, snap, now=100.0)
  assert d.v_cap_ms is not None
  expect = max(12.0, math.sqrt(2.0 * 0.9 * 200.0))
  expect = min(expect, 50.0 * CV.KPH_TO_MS)
  assert abs(d.v_cap_ms - expect) < 1e-3
  assert "dual_tbt" in d.mode


def test_dual_tbt_skipped_highway_and_arrive():
  pol = NavCruisePolicy()
  hw = _live(tbt_dist=80.0, tbt_dist_next=200.0, tbt_type_next=2, road_limit_kph=80.0)
  assert _dec(pol, hw, now=100.0).v_cap_ms is None or "dual_tbt" not in (_dec(pol, hw, now=100.0).mode)
  arr = _live(
    tbt_dist=80.0, tbt_dist_next=200.0, tbt_type_next=2,
    road_limit_kph=50.0, maneuver="arrive",
  )
  d = _dec(pol, arr, now=101.0)
  assert "dual_tbt" not in d.mode


def test_congestion_enter_exit():
  pol = NavCruisePolicy()
  # Seed sample
  s0 = _live(go_dist_m=5000.0, go_time_s=900.0, road_limit_kph=50.0)
  assert _dec(pol, s0, now=100.0).a_pos_cap is None
  # After 4 s: lost 20 m but ETA grew +20 s → congested
  s1 = _live(go_dist_m=4980.0, go_time_s=920.0, road_limit_kph=50.0)
  d1 = _dec(pol, s1, now=104.5)
  assert d1.a_pos_cap == CONG_A_POS
  # Recover: good progress, ETA shrinks
  s2 = _live(go_dist_m=4700.0, go_time_s=850.0, road_limit_kph=50.0)
  d2 = _dec(pol, s2, now=109.0)
  assert d2.a_pos_cap is None or "cong" not in d2.mode


def test_light_green_short_standstill():
  pol = NavCruisePolicy()
  snap = _live(
    traffic_light="green", remain_s=2.0, stop_for_light=False,
    go_dist_m=9000.0, road_limit_kph=50.0,
  )
  d = _dec(pol, snap, standstill=True, v_kph=0.0, now=100.0)
  assert d.a_pos_cap == GREEN_STANDSTILL_A


def test_red_light_a_pos_zero_without_lead():
  pol = NavCruisePolicy()
  snap = _live(
    traffic_light="red", stop_for_light=True,
    go_dist_m=9000.0, road_limit_kph=50.0,
  )
  d = _dec(pol, snap, now=100.0)
  assert d.a_pos_cap == 0.0


def test_stale_or_park_clears():
  pol = NavCruisePolicy()
  live = _live(sdi_type=SDI_POINT, sdi_dist_m=100.0, sdi_speed_kph=60.0)
  _dec(pol, live, now=100.0)
  # Force stale: _dec would refresh ts — call update directly.
  stale = _live(ts=1.0, sdi_type=SDI_POINT, sdi_dist_m=100.0, sdi_speed_kph=60.0)
  d = pol.update(
    stale, 40.0 * CV.KPH_TO_MS, gear="drive", has_near_lead=False,
    standstill=False, gas=False, long_enabled=True, now=100.0,
  )
  assert d.v_cap_ms is None and d.mode == "idle"
  park = _live(sdi_type=SDI_POINT, sdi_dist_m=100.0, sdi_speed_kph=60.0)
  d2 = _dec(pol, park, gear="park", now=100.0)
  assert d2.v_cap_ms is None


def test_apply_a_pos_cap():
  assert apply_a_pos_cap(1.5, 0.85, gas=False) == 0.85
  assert apply_a_pos_cap(-1.0, 0.85, gas=False) == -1.0
  assert apply_a_pos_cap(1.5, 0.85, gas=True) == 1.5
  assert apply_a_pos_cap(1.5, None, gas=False) == 1.5


def test_parse_carrot_sdi_and_next_tbt():
  from openpilot.sunnypilot.nav.protocol import parse_carrot
  snap = parse_carrot(
    {
      "nRoadLimitSpeed": 60,
      "nSdiType": 4,
      "nSdiDist": 194,
      "nSdiSpeedLimit": 60,
      "nTBTDistNext": 220,
      "nTBTTurnTypeNext": 2,
      "nGoPosDist": 3200,
      "nGoPosTime": 400,
      "szGoalName": "测试停车场",
      "szPosRoadName": "园区路",
    },
    now=50.0, link_ok=True, link_state=2, enabled=True,
  )
  assert snap is not None
  assert snap.sdi_type == 4 and snap.sdi_dist_m == 194.0 and snap.sdi_speed_kph == 60.0
  assert snap.tbt_dist_next == 220.0 and snap.tbt_type_next == 2
  assert snap.go_time_s == 400.0
  assert "停车" in snap.goal_name

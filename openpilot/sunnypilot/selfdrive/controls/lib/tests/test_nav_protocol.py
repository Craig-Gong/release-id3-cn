from openpilot.sunnypilot.nav.envelope import EnvelopeVerifier, envelope_hmac
from openpilot.sunnypilot.nav.protocol import lane_hint, parse_carrot
from openpilot.sunnypilot.nav.snapshot import NavSnapshot
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.junction_hud import build_junction_view
import json


def test_hmac_accepts_canonical_envelope():
  data = {"nRoadLimitSpeed": 60, "trafficLight": "red"}
  seq, ts = 3, 1_735_000_000_000
  digest = envelope_hmac("999999", seq, ts, data)
  raw = json.dumps({"v": 1, "seq": seq, "ts": ts, "data": data, "hmac": digest}).encode()
  v = EnvelopeVerifier("999999")
  assert v.accept(raw, now_ms=ts) == data


def test_hmac_rejects_bad_digest():
  data = {"nRoadLimitSpeed": 60}
  raw = json.dumps({"v": 1, "seq": 1, "ts": 1_735_000_000_000, "data": data, "hmac": "0" * 32}).encode()
  assert EnvelopeVerifier("999999").accept(raw, now_ms=1_735_000_000_000) is None


def test_red_remain_go_does_not_stop():
  snap = parse_carrot({
    "nRoadLimitSpeed": 60,
    "trafficLight": "red",
    "trafficLightRemainS": 1,
    "trafficLightDistM": 20,
  }, now=1.0, link_ok=True, link_state=2, enabled=True)
  assert snap is not None
  assert snap.remain_go
  assert snap.stop_for_light is False


def test_red_without_remain_stops():
  snap = parse_carrot({
    "nRoadLimitSpeed": 60,
    "trafficLight": "red",
    "trafficLightDistM": 25,
  }, now=1.0, link_ok=True, link_state=2, enabled=True)
  assert snap is not None
  assert snap.stop_for_light
  assert snap.accel_target == -2.0


def test_idle_view_when_no_light():
  snap = NavSnapshot(iqlink_enabled=True, link_ok=True, traffic_light="none")
  view = build_junction_view(
    engaged=True, has_lead=False, model_stop=False, standstill_hold=False,
    snap=snap, green_flash=False,
  )
  assert view.show and view.idle
  assert view.headline == "暂无信号"
  assert view.detail == "等待识别"


def test_red_view_and_follow_lead():
  snap = NavSnapshot(iqlink_enabled=True, link_ok=True, traffic_light="red",
                     stop_for_light=True, dist_m=18, remain_s=6)
  stop = build_junction_view(
    engaged=True, has_lead=False, model_stop=False, standstill_hold=True,
    snap=snap, green_flash=False,
  )
  assert stop.headline == "红灯"
  assert "米" in stop.detail
  follow = build_junction_view(
    engaged=True, has_lead=True, model_stop=True, standstill_hold=True,
    snap=snap, green_flash=False,
  )
  assert follow.headline == "跟前车"


def test_lane_hint_left():
  snap = NavSnapshot(lane_recommend="left")
  assert lane_hint(snap) == "靠左车道"

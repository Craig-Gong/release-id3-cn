from openpilot.sunnypilot.nav.protocol import parse_carrot


def _parse(payload, now=10.0):
  return parse_carrot(payload, now=now, link_ok=True, link_state=2, enabled=True)


def test_turn_bucket_is_turn_maneuver():
  snap = _parse({"nRoadLimitSpeed": 50, "nTBTTurnType": 1, "nTBTDist": 80})
  assert snap is not None
  assert snap.maneuver == "turn"
  assert snap.send_turn is True
  assert snap.maneuver_dir == "left"


def test_near_lc_promotes_send_turn_but_stays_fork():
  snap = _parse({"nRoadLimitSpeed": 50, "nTBTTurnType": 3, "nTBTDist": 80})
  assert snap is not None
  assert snap.maneuver == "fork"
  assert snap.send_turn is True
  assert snap.maneuver_dir == "left"


def test_highway_lc_does_not_promote_send_turn():
  snap = _parse({"nRoadLimitSpeed": 80, "nTBTTurnType": 3, "nTBTDist": 80})
  assert snap is not None
  assert snap.maneuver == "fork"
  assert snap.send_turn is False


def test_urban_lc_still_promotes_under_70():
  snap = _parse({"nRoadLimitSpeed": 60, "nTBTTurnType": 3, "nTBTDist": 80})
  assert snap is not None
  assert snap.maneuver == "fork"
  assert snap.send_turn is True


def test_straight_lc_does_not_promote():
  snap = _parse({
    "nRoadLimitSpeed": 80, "nTBTTurnType": 3, "nTBTDist": 80,
    "laneRecommend": "straight",
  })
  assert snap is not None
  assert snap.maneuver == "fork"
  assert snap.send_turn is False


def test_far_lc_is_fork_without_send_turn():
  snap = _parse({"nRoadLimitSpeed": 80, "nTBTTurnType": 3, "nTBTDist": 400})
  assert snap is not None
  assert snap.maneuver == "fork"
  assert snap.send_turn is False


def test_exit_is_not_send_turn():
  snap = _parse({"nRoadLimitSpeed": 80, "nTBTTurnType": 6, "nTBTDist": 80})
  assert snap is not None
  assert snap.maneuver == "exit"
  assert snap.send_turn is False


def test_arrive_drops_send_turn():
  snap = _parse({
    "nRoadLimitSpeed": 40, "nTBTTurnType": 1, "nTBTDist": 40, "nGoPosDist": 80,
  })
  assert snap is not None
  assert snap.maneuver == "arrive"
  assert snap.send_turn is False


def test_keepalive_without_limit_is_none():
  assert _parse({"trafficLight": "red"}) is None


def test_far_turn_hud_is_straight_ahead():
  from openpilot.sunnypilot.nav.hud_copy import STRAIGHT_AHEAD, TURN_LEFT
  from openpilot.sunnypilot.nav.protocol import lane_hint
  from openpilot.sunnypilot.selfdrive.controls.lib.helpers.junction_hud import build_lane_guide_view

  far = _parse({"nRoadLimitSpeed": 60, "nTBTTurnType": 1, "nTBTDist": 400})
  assert far is not None and far.send_turn is False
  assert lane_hint(far) == STRAIGHT_AHEAD
  view = build_lane_guide_view(onroad=True, snap=far)
  assert view.kind == "straight" and view.empty is False
  assert view.capsule is not None

  near = _parse({"nRoadLimitSpeed": 60, "nTBTTurnType": 1, "nTBTDist": 80})
  assert near is not None and near.send_turn is True
  assert lane_hint(near) == TURN_LEFT


def test_red_remain_go_keeps_stop_for_light():
  """Countdown remainS==1 must not clear head-car red stop."""
  snap = _parse({
    "nRoadLimitSpeed": 50,
    "trafficLight": "red",
    "trafficLightDistM": 12,
    "trafficLightRemainS": 1,
  })
  assert snap is not None
  assert snap.remain_go is True
  assert snap.stop_for_light is True
  assert snap.traffic_light == "red"


def test_guessed_light_distance_is_not_trusted():
  snap = _parse({"nRoadLimitSpeed": 50, "trafficLight": "red", "trafficLightDistM": 40})
  assert snap.stop_for_light is True
  assert snap.dist_ok is False
  # No distance braking target from a guess: cruise target stays the road limit.
  assert abs(snap.speed_target - 50 / 3.6) < 1e-6


def test_trusted_light_distance_source():
  snap = _parse({"nRoadLimitSpeed": 50, "trafficLight": "red", "trafficLightDistM": 40,
                 "trafficLightDistSrc": "route"})
  assert snap.dist_ok is True
  assert snap.speed_target < 50 / 3.6


def test_yellow_without_real_distance_does_not_stop():
  snap = _parse({"nRoadLimitSpeed": 50, "trafficLight": "yellow", "trafficLightDistM": 20})
  assert snap.stop_for_light is False


def test_light_ts_subtracts_phone_age():
  snap = _parse({"nRoadLimitSpeed": 50, "trafficLight": "red", "trafficLightAgeMs": 1500}, now=100.0)
  assert abs(snap.light_ts - 98.5) < 1e-6
  dark = _parse({"nRoadLimitSpeed": 50, "trafficLight": "none"}, now=100.0)
  assert dark.light_ts == 0.0


def test_rtor_only_near_and_not_right_arrow():
  near = _parse({"nRoadLimitSpeed": 50, "trafficLight": "red", "nTBTTurnType": 2, "nTBTDist": 40})
  assert near.stop_for_light is False
  far = _parse({"nRoadLimitSpeed": 50, "trafficLight": "red", "nTBTTurnType": 2, "nTBTDist": 120})
  assert far.stop_for_light is True
  arrow = _parse({"nRoadLimitSpeed": 50, "trafficLight": "red", "nTBTTurnType": 2, "nTBTDist": 40,
                  "trafficLightDir": "right"})
  assert arrow.stop_for_light is True

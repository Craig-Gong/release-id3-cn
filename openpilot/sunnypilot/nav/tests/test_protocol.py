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

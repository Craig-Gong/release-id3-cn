"""Unit tests for iqlink protocol mapping.

Run: pytest iqpilot/iqlink/tests/test_protocol.py
"""

from openpilot.iqpilot.iqlink.protocol import map_carrot_to_nav_fields


def test_basic_speed_and_turn():
  raw = {
    "nRoadLimitSpeed": 80,
    "nTBTTurnType": 2,
    "nTBTDist": 90,
    "szTBTMainText": "右转",
    "nGoPosDist": 1200,
    "nGoPosTime": 180,
    "goalPosY": 32.03,
    "goalPosX": 118.90,
    "szGoalName": "家",
  }
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert f["active"] is True
  # TBT distance cap removed: near turn keeps bare road limit; desire still fires.
  assert abs(f["speedTarget"] - 80 / 3.6) < 1e-3
  assert f["navSpeedTargetActive"] is False
  assert f["shouldSendTurnDesire"] is True
  assert f["nextManeuverDirection"] == "right"
  assert f["destinationName"] == "家"


def test_aggressive_lane_change_window():
  raw = {
    "nRoadLimitSpeed": 100,
    "nTBTTurnType": 4,
    "nTBTDist": 700,
  }
  f = map_carrot_to_nav_fields(raw, aggressive_lc=True)
  assert f is not None
  assert f["shouldSendLaneChangeDesire"] is True
  assert f["command"] == "laneChange"
  assert f["nextManeuverType"] == "exit"

  f2 = map_carrot_to_nav_fields(raw, aggressive_lc=False)
  assert f2 is not None
  assert f2["shouldSendLaneChangeDesire"] is False


def test_lane_recommend_straight_suppresses_lc():
  raw = {
    "nRoadLimitSpeed": 100,
    "nTBTTurnType": 4,
    "nTBTDist": 700,
    "laneRecommend": "straight",
  }
  f = map_carrot_to_nav_fields(raw, aggressive_lc=True)
  assert f is not None
  assert f["shouldSendLaneChangeDesire"] is False
  assert f["command"] == "none"
  # Maneuver HUD still describes the exit/LC.
  assert f["nextManeuverValid"] is True


def test_lane_recommend_left_keeps_lc():
  raw = {
    "nRoadLimitSpeed": 100,
    "nTBTTurnType": 4,
    "nTBTDist": 700,
    "laneRecommend": "left",
  }
  f = map_carrot_to_nav_fields(raw, aggressive_lc=True)
  assert f is not None
  assert f["shouldSendLaneChangeDesire"] is True


def test_camera_ignored_this_phase():
  # SDI / speed-camera pressure disabled; road limit owns speedTarget.
  raw = {
    "nRoadLimitSpeed": 120,
    "nSdiType": 1,
    "nSdiDist": 200,
    "nSdiSpeedLimit": 60,
  }
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert f["cameraValid"] is False
  assert abs(f["speedTarget"] - 120 / 3.6) < 1e-3
  assert f["longitudinalProvider"] == "route"


def test_rgdata_wrapper():
  raw = {"rgdata": {"nRoadLimitSpeed": 50}}
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert f["active"] is True


def test_rejects_without_heartbeat():
  assert map_carrot_to_nav_fields({"nTBTDist": 10}) is None


def test_green_wave_ignored():
  """C5-b: nGreenWaveSpeed must not affect speedTarget."""
  raw = {
    "nRoadLimitSpeed": 80,
    "nGreenWaveSpeed": 48,
  }
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert abs(f["speedTarget"] - 80 / 3.6) < 1e-3
  assert f["longitudinalEngaged"] is True


def test_tbt_near_turn_keeps_road_limit():
  """Near turn: no distance speed cap; turn desire still sent for lateral."""
  raw = {
    "nRoadLimitSpeed": 80,
    "nTBTTurnType": 1,
    "nTBTDist": 40,
  }
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert f["shouldSendTurnDesire"] is True
  assert abs(f["speedTarget"] - 80 / 3.6) < 1e-3
  assert f["navSpeedTargetActive"] is False


def test_tbt_far_turn_no_cap():
  raw = {
    "nRoadLimitSpeed": 80,
    "nTBTTurnType": 1,
    "nTBTDist": 500,
  }
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert abs(f["speedTarget"] - 80 / 3.6) < 1e-3
  assert f["navSpeedTargetActive"] is False


def test_red_light_stop_keeps_engaged():
  raw = {
    "nRoadLimitSpeed": 60,
    "trafficLight": "red",
    "trafficLightRemainS": 20,
    "trafficLightDistM": 40,
  }
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  # Distance curve (+2 m early-stop compensate): not slam to 0 at 40 m.
  assert 0.0 < f["speedTarget"] < 60 / 3.6
  assert f["longitudinalEngaged"] is True
  assert f["valid"] is True
  assert f["accelTarget"] == -2.0
  assert f["cameraType"] != "redLight"
  assert f["trafficLight"] == "red"
  assert f["trafficLightDistM"] == 40.0
  assert f["trafficLightRemainS"] == 20.0


def test_red_remain_3_still_stops():
  # APK "go intent" at remainS==3: still red-stop (armed), no fake green.
  raw = {
    "nRoadLimitSpeed": 60,
    "trafficLight": "red",
    "trafficLightRemainS": 3,
    "trafficLightDistM": 25,
  }
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert f["accelTarget"] == -2.0
  assert f["speedTarget"] < 60 / 3.6
  assert f["trafficLight"] == "red"


def test_red_remain_1_still_stops():
  # Countdown==1 is still red: do not release nav stop / HMS before green.
  raw = {
    "nRoadLimitSpeed": 60,
    "trafficLight": "red",
    "trafficLightRemainS": 1,
    "trafficLightDistM": 5,
  }
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert f["accelTarget"] == -2.0
  assert f["speedTarget"] < 60 / 3.6
  assert f["trafficLight"] == "red"
  assert f["longitudinalEngaged"] is True


def test_red_without_remain_does_not_prestart():
  # Omitted/0 RemainS must not be treated as <=1 (BleCrypto omits zero).
  raw = {"nRoadLimitSpeed": 60, "trafficLight": "red", "trafficLightDistM": 30}
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert f["accelTarget"] == -2.0
  assert f["speedTarget"] < 60 / 3.6


def test_red_light_without_dist_holds_stop():
  # APK red with no meters → hold stopped.
  raw = {"nRoadLimitSpeed": 60, "trafficLight": "red"}
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert f["speedTarget"] == 0.0
  assert f["accelTarget"] == -2.0
  assert f["longitudinalEngaged"] is True
  assert f["valid"] is True


def test_red_light_right_turn_no_nav_stop():
  raw = {
    "nRoadLimitSpeed": 60,
    "trafficLight": "red",
    "trafficLightDistM": 30,
    "nTBTTurnType": 2,
    "nTBTDist": 40,
  }
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert f["nextManeuverDirection"] == "right"
  # Right-turn-on-red: no nav red stop (accel stays off the red-stop -2.0 path).
  assert f["accelTarget"] != -2.0
  # No TBT speed cap either — bare road limit.
  assert abs(f["speedTarget"] - 60 / 3.6) < 1e-3


def test_yellow_far_does_not_stop():
  raw = {
    "nRoadLimitSpeed": 60,
    "trafficLight": "yellow",
    "trafficLightDistM": 80,
  }
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert abs(f["speedTarget"] - 60 / 3.6) < 1e-3


def test_yellow_near_stops():
  raw = {
    "nRoadLimitSpeed": 60,
    "trafficLight": "yellow",
    "trafficLightDistM": 20,
  }
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert 0.0 <= f["speedTarget"] < 60 / 3.6
  assert f["longitudinalEngaged"] is True
  assert f["accelTarget"] == -2.0


def test_green_restores_limit():
  raw = {
    "nRoadLimitSpeed": 70,
    "trafficLight": "green",
    "trafficLightRemainS": 10,
  }
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert abs(f["speedTarget"] - 70 / 3.6) < 1e-3
  assert f["cameraType"] != "redLight"


def test_vision_stop_without_green():
  # vision_stop no longer forces nav stop — E2E owns implicit stops.
  raw = {"nRoadLimitSpeed": 60, "trafficLight": "none"}
  f = map_carrot_to_nav_fields(raw, vision_stop=True)
  assert f is not None
  assert abs(f["speedTarget"] - 60 / 3.6) < 1e-3


def test_vision_stop_ignored_when_gaode_green():
  raw = {"nRoadLimitSpeed": 60, "trafficLight": "green"}
  f = map_carrot_to_nav_fields(raw, vision_stop=True)
  assert f is not None
  assert abs(f["speedTarget"] - 60 / 3.6) < 1e-3


def test_section_sdi_ignored_this_phase():
  raw = {
    "nRoadLimitSpeed": 100,
    "nSdiType": 1,
    "nSdiDist": 500,
    "nSdiSpeedLimit": 80,
    "nSdiBlockType": 2,
    "nSdiBlockDist": 120,
  }
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert abs(f["speedTarget"] - 100 / 3.6) < 1e-3
  assert f["cameraValid"] is False
  assert f["cameraDistance"] == 0.0


def test_vp_pos_is_not_destination():
  """Phone GPS ego report must not mark destinationValid."""
  raw = {
    "nRoadLimitSpeed": 60,
    "vpPosPointLat": 32.03,
    "vpPosPointLon": 118.90,
  }
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert f["destinationValid"] is False
  assert abs(f["destinationLatitude"]) < 1e-6


def test_remain_distance_marks_destination_without_poi_name():
  """Fullscreen Gaode often has remain km but no szGoalName in a11y."""
  raw = {
    "nRoadLimitSpeed": 80,
    "nGoPosDist": 17000,
    "nGoPosTime": 1200,
  }
  f = map_carrot_to_nav_fields(raw)
  assert f is not None
  assert f["destinationValid"] is True
  assert f["distanceRemaining"] == 17000
  assert f["destinationName"] == ""

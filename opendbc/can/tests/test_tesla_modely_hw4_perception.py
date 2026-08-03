from opendbc.can import CANPacker, CANParser


DBC = "tesla_modely_hw4_perception"


def test_hw4_perception_dbc_decodes_navigation_lanes_and_traffic():
  packer = CANPacker(DBC)
  parser = CANParser(DBC, [
    ("UI_driverAssistMapData", float("nan")),
    ("DAS_lanes", float("nan")),
    ("APP_trafficControl", float("nan")),
  ], 0)
  frames = []
  for message, values in (
    ("UI_driverAssistMapData", {
      "UI_navRouteActive": 1,
      "UI_gpsRoadMatch": 1,
      "UI_nextBranchDist": 120,
      "UI_nextBranchRightOffRamp": 1,
    }),
    ("DAS_lanes", {
      "DAS_leftLaneExists": 1,
      "DAS_rightLaneExists": 1,
      "DAS_virtualLaneWidth": 3.5,
      "DAS_virtualLaneViewRange": 80,
    }),
    ("APP_trafficControl", {
      "APP_tcFeatureState": 3,
      "APP_tcControlSource": 3,
      "APP_tcControlType": 3,
      "APP_tcControlDistance": 42,
      "APP_tcControlLightState": 1,
    }),
  ):
    address, data, _ = packer.make_can_msg(message, 0, values)
    frames.append((address, data, 0))

  parser.update([(1_000_000_000, frames)])

  assert parser.vl["UI_driverAssistMapData"]["UI_navRouteActive"] == 1
  assert parser.vl["UI_driverAssistMapData"]["UI_nextBranchDist"] == 120
  assert parser.vl["DAS_lanes"]["DAS_virtualLaneWidth"] == 3.5625
  assert parser.vl["APP_trafficControl"]["APP_tcControlDistance"] == 42
  assert parser.vl["APP_trafficControl"]["APP_tcControlLightState"] == 1


def test_hw4_perception_dbc_decodes_multiplexed_vehicle_group():
  packer = CANPacker(DBC)
  parser = CANParser(DBC, [("DAS_object", float("nan"))], 2)
  address, data, _ = packer.make_can_msg("DAS_object", 2, {
    "DAS_objectId": 0,
    "DAS_leadVehType": 2,
    "DAS_leadVehRelevantForControl": 1,
    "DAS_leadVehDx": 25,
    "DAS_leadVehVxRel": -2,
    "DAS_leadVehDy": 0,
    "DAS_leadVehId": 7,
  })

  parser.update([(1_000_000_000, [(address, data, 2)])])

  assert parser.vl["DAS_object"]["DAS_objectId"] == 0
  assert parser.vl["DAS_object"]["DAS_leadVehType"] == 2
  assert parser.vl["DAS_object"]["DAS_leadVehDx"] == 25
  assert parser.vl["DAS_object"]["DAS_leadVehVxRel"] == -2
  assert parser.vl["DAS_object"]["DAS_leadVehId"] == 7


def test_hw4_perception_dbc_decodes_pedestrian_and_status_context():
  packer = CANPacker(DBC)
  parser = CANParser(DBC, [
    ("APP_pedestrianDetection", float("nan")),
    ("DAS_status", float("nan")),
    ("DAS_integratedSafetyFront", float("nan")),
  ], 0)
  frames = [
    packer.make_can_msg("APP_pedestrianDetection", 0, {
      "APP_pedestrianDetectedFrontMain": 1,
      "APP_pedestrianDetectedBackup": 1,
      "APP_closestPedestrian1dX": 3.2,
      "APP_closestPedestrian1dY": -1.6,
    }),
    packer.make_can_msg("DAS_status", 0, {
      "DAS_blindSpotRearLeft": 2,
      "DAS_blindSpotRearRight": 1,
      "DAS_sideCollisionWarning": 2,
      "DAS_forwardCollisionWarning": 1,
    }),
    packer.make_can_msg("DAS_integratedSafetyFront", 0, {
      "DAS_targetDistanceFront": 12.0,
      "DAS_relativeVelocityFront": -4.0,
      "DAS_timeToImpactFront": 30,
      "DAS_predictedImpactOvrlapFront": 62.5,
    }),
  ]
  parser.update([(1_000_000_000, [(address, data, 0) for address, data, _ in frames])])

  assert parser.vl["APP_pedestrianDetection"]["APP_pedestrianDetectedFrontMain"] == 1
  assert parser.vl["APP_pedestrianDetection"]["APP_closestPedestrian1dX"] == 3.2
  assert parser.vl["APP_pedestrianDetection"]["APP_closestPedestrian1dY"] == -1.6
  assert parser.vl["DAS_status"]["DAS_blindSpotRearLeft"] == 2
  assert parser.vl["DAS_status"]["DAS_sideCollisionWarning"] == 2
  assert parser.vl["DAS_integratedSafetyFront"]["DAS_targetDistanceFront"] == 12.0
  assert parser.vl["DAS_integratedSafetyFront"]["DAS_relativeVelocityFront"] == -4.0
  assert parser.vl["DAS_integratedSafetyFront"]["DAS_predictedImpactOvrlapFront"] == 62.5


def test_hw4_perception_dbc_decodes_multiplexed_road_sign_groups():
  packer = CANPacker(DBC)
  parser = CANParser(DBC, [("UI_driverAssistRoadSign", float("nan"))], 2)

  stop_address, stop_data, _ = packer.make_can_msg("UI_driverAssistRoadSign", 2, {
    "UI_roadSign": 1,
    "UI_stopSignStopLineDist": 12.0,
    "UI_stopSignStopLineConf": 100,
  })
  parser.update([(1_000_000_000, [(stop_address, stop_data, 2)])])
  assert parser.vl["UI_driverAssistRoadSign"]["UI_roadSign"] == 1
  assert parser.vl["UI_driverAssistRoadSign"]["UI_stopSignStopLineDist"] == 12.0
  assert parser.vl["UI_driverAssistRoadSign"]["UI_stopSignStopLineConf"] == 100

  light_address, light_data, _ = packer.make_can_msg("UI_driverAssistRoadSign", 2, {
    "UI_roadSign": 2,
    "UI_trafficLightStopLineDist": 30.0,
    "UI_trafficLightStopLineConf": 90,
  })
  parser.update([(2_000_000_000, [(light_address, light_data, 2)])])
  assert parser.vl["UI_driverAssistRoadSign"]["UI_roadSign"] == 2
  assert parser.vl["UI_driverAssistRoadSign"]["UI_trafficLightStopLineDist"] == 30.0
  assert parser.vl["UI_driverAssistRoadSign"]["UI_trafficLightStopLineConf"] == 90

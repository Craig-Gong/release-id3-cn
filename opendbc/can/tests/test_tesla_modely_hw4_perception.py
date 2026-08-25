import math
from pathlib import Path

from opendbc.can import CANDefine, CANPacker, CANParser


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


def test_hw4_perception_dbc_decodes_long_control_muxes():
  packer = CANPacker(DBC)
  parser = CANParser(DBC, [("DAS_longControl", float("nan"))], 2)
  frames = [
    packer.make_can_msg("DAS_longControl", 2, {
      "DAS_longControlStack": 2,
      "DAS_torqueProfiler_accelMinPed": -3.0,
      "DAS_torqueProfiler_targetSpeedPed": 80.0,
    }),
    packer.make_can_msg("DAS_longControl", 2, {
      "DAS_longControlStack": 4,
      "DAS_aebControl_active": 2,
      "DAS_aebControl_targetAccelDis": -3.0,
    }),
  ]
  parser.update([(1_000_000_000, frames)])

  assert parser.vl_all["DAS_longControl"]["DAS_longControlStack"] == [2.0, 4.0]
  assert parser.vl_all["DAS_longControl"]["DAS_torqueProfiler_targetSpeedPed"][0] == 80.0
  assert parser.vl_all["DAS_longControl"]["DAS_aebControl_active"][1] == 2.0
  assert parser.vl_all["DAS_longControl"]["DAS_aebControl_targetAccelDis"][1] == -3.0


def test_hw4_perception_dbc_decodes_parking_and_party_safety_status():
  packer = CANPacker(DBC)
  parser = CANParser(DBC, [
    ("PARK_oocStatus", float("nan")),
    ("DAS_status2", float("nan")),
    ("DAS_status", float("nan")),
  ], 0)
  frames = [
    packer.make_can_msg("PARK_oocStatus", 0, {
      "PARK_oocDistance": 180,
      "PARK_oocConfidence": 90,
      "PARK_oocVehicleX": 50,
      "PARK_oocVehicleY": -20,
      "PARK_oocCollisionSide": 1,
    }),
    packer.make_can_msg("DAS_status2", 0, {
      "DAS_pmmObstacleSeverity": 3,
      "DAS_longCollisionWarning": 2,
    }),
    packer.make_can_msg("DAS_status", 0, {
      "DAS_sideCollisionAvoid": 2,
      "DAS_sideCollisionWarning": 1,
      "DAS_sideCollisionInhibit": 1,
    }),
  ]
  parser.update([(1_000_000_000, frames)])

  assert parser.vl["PARK_oocStatus"]["PARK_oocDistance"] == 180
  assert parser.vl["PARK_oocStatus"]["PARK_oocVehicleY"] == -20
  assert parser.vl["DAS_status2"]["DAS_pmmObstacleSeverity"] == 3
  assert parser.vl["DAS_status"]["DAS_sideCollisionAvoid"] == 2
  assert parser.vl["DAS_status"]["DAS_sideCollisionWarning"] == 1
  assert parser.vl["DAS_status"]["DAS_sideCollisionInhibit"] == 1


def test_hw4_perception_dbc_labels_vehicle_validated_pedestrian_coordinate_units():
  definitions = CANDefine(DBC)
  assert definitions.dv["DAS_status2"]["DAS_longCollisionWarning"][2] == "PEDESTRIAN"

  dbc_path = Path(__file__).parents[2] / "dbc" / f"{DBC}.dbc"
  pedestrian_lines = [line for line in dbc_path.read_text().splitlines() if "APP_closestPedestrian" in line and line.lstrip().startswith("SG_")]
  assert len(pedestrian_lines) == 6
  assert all('[0|0] "m"' in line for line in pedestrian_lines)


def test_hw4_perception_dbc_decodes_vehicle_energy_tpms_and_temperature_pages():
  packer = CANPacker(DBC)
  parser = CANParser(DBC, [
    ("BMS_hvBusStatus", float("nan")),
    ("VCSEC_TPMSData", float("nan")),
    ("DIR_temperature", float("nan")),
    ("BMS_kwhCounter", float("nan")),
    ("DI_estimatedBrakeTemp", float("nan")),
  ], 1)
  frames = [
    packer.make_can_msg("BMS_hvBusStatus", 1, {"BMS_dcLinkVoltage": 400.0, "BMS_packCurrent": -50.0}),
    packer.make_can_msg("VCSEC_TPMSData", 1, {
      "VCSEC_TPMSDataIndex": 0,
      "VCSEC_TPMSPressure0": 2.5,
      "VCSEC_TPMSTemperature0": 30,
      "VCSEC_TPMSLocation0": 0,
    }),
    packer.make_can_msg("VCSEC_TPMSData", 1, {
      "VCSEC_TPMSDataIndex": 4,
      "VCSEC_TPMSRecommendedColdPressureFront": 2.9,
      "VCSEC_TPMSRecommendedColdPressureRear": 3.0,
    }),
    packer.make_can_msg("DIR_temperature", 1, {
      "DIR_tempIndex": 0,
      "DIR_inverterTQF": 2,
      "DIR_inverterT": 50,
      "DIR_statorT": 55,
    }),
    packer.make_can_msg("BMS_kwhCounter", 1, {
      "BMS_kwhDischargeTotal": 120.5,
      "BMS_kwhChargeTotal": 100.25,
    }),
    packer.make_can_msg("DI_estimatedBrakeTemp", 1, {
      "DI_brakeFLTemp": 80,
      "DI_brakeFRTemp": 81,
      "DI_brakeRLTemp": 60,
      "DI_brakeRRTemp": 61,
    }),
  ]
  parser.update([(1_000_000_000, frames)])

  assert parser.vl["BMS_hvBusStatus"]["BMS_dcLinkVoltage"] == 400.0
  assert parser.vl["BMS_hvBusStatus"]["BMS_packCurrent"] == -50.0
  assert parser.vl_all["VCSEC_TPMSData"]["VCSEC_TPMSDataIndex"] == [0.0, 4.0]
  assert parser.vl_all["VCSEC_TPMSData"]["VCSEC_TPMSPressure0"][0] == 2.5
  assert math.isclose(parser.vl_all["VCSEC_TPMSData"]["VCSEC_TPMSRecommendedColdPressureFront"][1], 2.9)
  assert parser.vl["DIR_temperature"]["DIR_inverterT"] == 50.0
  assert parser.vl["DIR_temperature"]["DIR_statorT"] == 55.0
  assert parser.vl["BMS_kwhCounter"]["BMS_kwhDischargeTotal"] == 120.5
  assert parser.vl["DI_estimatedBrakeTemp"]["DI_brakeRRTemp"] == 61.0

from collections import defaultdict

import pytest

from iqdbc.car import gen_empty_fingerprint, structs
from iqdbc.car.structs import CarParams
from iqdbc.car.fw_versions import match_fw_to_car
from iqdbc.car.tesla.fingerprints import FW_VERSIONS
from iqdbc.car.tesla.interface import CarInterface
from iqdbc.car.tesla.teslacan import TeslaCAN
from iqdbc.car.tesla.radar_interface import RADAR_START_ADDR
from iqdbc.car.tesla.carcontroller import CarController
from iqdbc.car.tesla.carstate import stock_autosteer_invalid
from iqdbc.car.tesla.values import (CAR, FW_PATTERN, LEGACY_DAS_STEERING_FW, TeslaFlags, TeslaSafetyFlags,
                                    get_platform_codes, is_legacy_das_steering)
from iqdbc.lvbs.car.interfaces import _apply_tesla_options
from iqdbc.lvbs.car.tesla.values import TeslaFlagsIQ, TeslaSafetyFlagsIQ
from iqdbc.can import CANPacker, CANParser

Ecu = CarParams.Ecu
EPS_ADDR = 0x730


def fw_match(fw: bytes):
  car_fw = [CarParams.CarFw(ecu=Ecu.eps, fwVersion=fw, address=EPS_ADDR, subAddress=0, brand='tesla')]
  exact, matches = match_fw_to_car(car_fw, '0' * 17, log=False)
  return exact, matches


class TestTeslaFwPattern:
  def test_all_known_fw_parses(self):
    for car, ecus in FW_VERSIONS.items():
      for fws in ecus.values():
        for fw in fws:
          assert FW_PATTERN.match(fw) is not None, f'{car}: unparsed FW version: {fw}'

  def test_model_code_identifies_one_platform(self):
    # a new platform reusing an existing model code would silently misfingerprint
    platforms = defaultdict(set)
    for car, ecus in FW_VERSIONS.items():
      for fws in ecus.values():
        for model, _, _ in get_platform_codes(fws):
          platforms[model].add(car)

    for model, cars in platforms.items():
      assert len(cars) == 1, f'model code {model} maps to multiple platforms: {cars}'

  def test_exact_match_still_wins(self):
    for car, ecus in FW_VERSIONS.items():
      for fws in ecus.values():
        for fw in fws:
          exact, matches = fw_match(fw)
          assert exact, f'{fw} fell back to fuzzy matching'
          assert matches == {car}, f'{fw} matched {matches}, expected {car}'

  @pytest.mark.parametrize("fw, expected", [
    # a firmware bump within a known series, the case that used to fingerprint as MOCK
    (b'TeMYG4_Main_0.0.0 (99),Y4003.14.0', CAR.TESLA_MODEL_Y),
    (b'TeMYG4_Main_0.0.0 (99),E4H015.09.0', CAR.TESLA_MODEL_3),
    (b'TeM3_SP_XP002p2_0.0.0 (40),XPR003.12.0', CAR.TESLA_MODEL_X),
    # Tesla bumps the series within a platform (E4014 -> E4015, Y4002 -> Y4003)
    (b'TeMYG4_Main_0.0.0 (12),Y4004.01.0', CAR.TESLA_MODEL_Y),
    # an unknown model code is a car we don't support
    (b'TeCT_Main_0.0.0 (1),CT001.01.0', None),
    (b'garbage', None),
  ])
  def test_unknown_fw_fuzzy_match(self, fw, expected):
    exact, matches = fw_match(fw)
    if expected is None:
      assert matches == set(), f'{fw} unexpectedly matched {matches}'
    else:
      assert not exact
      assert matches == {expected}


class TestTeslaLegacyDasSteering:
  def test_reproduces_known_table(self):
    for car, ecus in FW_VERSIONS.items():
      for fws in ecus.values():
        for fw in fws:
          expected = fw in LEGACY_DAS_STEERING_FW.get(car, [])
          assert is_legacy_das_steering(car, fw) == expected, f'{car}: wrong legacy DAS verdict for {fw}'

  @pytest.mark.parametrize("car, fw, expected", [
    # below a family's known modern cutoff (Y4/003 splits at 003.04.0)
    (CAR.TESLA_MODEL_Y, b'TeMYG4_Legacy3Y_0.0.0 (5),Y4003.03.9', True),
    (CAR.TESLA_MODEL_Y, b'TeMYG4_Main_0.0.0 (99),Y4003.14.0', False),
    # E4/015 splits at 015.04.5
    (CAR.TESLA_MODEL_3, b'TeMYG4_Main_0.0.0 (68),E4H015.03.9', True),
    (CAR.TESLA_MODEL_3, b'TeMYG4_Main_0.0.0 (99),E4H015.09.0', False),
    # families with no known modern FW: interpolate legacy, extrapolate modern
    (CAR.TESLA_MODEL_3, b'TeM3_E014p10_0.0.0 (16),E014.18.00', True),
    (CAR.TESLA_MODEL_3, b'TeM3_E014p10_0.0.0 (30),E014.22.0', False),
    # numeric, not lexical, version compare (XPR003.10.0 > XPR003.6.0)
    (CAR.TESLA_MODEL_X, b'TeM3_SP_XP002p2_0.0.0 (30),XPR003.9.0', True),
    # an unknown series has no history to compare against
    (CAR.TESLA_MODEL_Y, b'TeMYG4_Main_0.0.0 (12),Y4004.01.0', False),
    (CAR.TESLA_MODEL_Y, b'garbage', False),
  ])
  def test_unknown_fw(self, car, fw, expected):
    assert is_legacy_das_steering(car, fw) == expected

  def test_flag_set_from_fuzzy_match(self):
    for fw, legacy in ((b'TeMYG4_Legacy3Y_0.0.0 (5),Y4003.03.9', True),
                       (b'TeMYG4_Main_0.0.0 (99),Y4003.14.0', False)):
      car_fw = [CarParams.CarFw(ecu=Ecu.eps, fwVersion=fw, address=EPS_ADDR, subAddress=0, brand='tesla')]
      CP = CarInterface.get_params(CAR.TESLA_MODEL_Y, gen_empty_fingerprint(), car_fw, False, False, False)
      assert bool(CP.flags & TeslaFlags.LEGACY_DAS_STEERING) == legacy
      assert bool(CP.safetyConfigs[0].safetyParam & TeslaSafetyFlags.LEGACY_DAS_STEERING) == legacy


def test_fsd_visualization_params():
  CP = structs.CarParams(brand="tesla")
  CP_IQ = structs.IQCarParams()
  _apply_tesla_options(CP, CP_IQ, {"IQTeslaFsdVisualization": "1"})
  assert CP_IQ.flags & TeslaFlagsIQ.FSD_VISUALIZATION
  assert CP_IQ.iqSafetyFlags & TeslaSafetyFlagsIQ.FSD_VISUALIZATION


@pytest.mark.parametrize("enabled, expected", [(False, True), (True, False)])
def test_fsd_visualization_invalid_lkas(enabled, expected):
  CP = structs.CarParams()
  CP_IQ = structs.IQCarParams(iqSafetyFlags=TeslaSafetyFlagsIQ.FSD_VISUALIZATION if enabled else 0)
  assert stock_autosteer_invalid(CP, CP_IQ, 3) == expected


class TestTeslaFingerprint:
  def test_radar_detection(self):
    # Test radar availability detection for cars with radar DBC defined
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_no_radar_car(self):
    # Model X doesn't have radar DBC defined, should always be unavailable
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_X, fingerprint, [], False, False, False)
      assert CP.radarUnavailable  # Always unavailable since no radar DBC


class TestTeslaCan:
  class DummyPacker:
    def make_can_msg(self, name, bus, values):
      return name, bus, values

  def test_vehicle_bus_odometer_decodes_kilometers(self):
    packer = CANPacker("tesla_model3_vehicle")
    parser = CANParser("tesla_model3_vehicle", [("ID3B6UI_odometer", 1)], 1)

    message = packer.make_can_msg("ID3B6UI_odometer", 1, {
      "UI_odometer": 29150.377,
      "UI_odometerCounter": 1,
      "UI_odometerChecksum": 0,
    })
    parser.update([1_000_000_000, [message]])

    assert parser.vl["ID3B6UI_odometer"]["UI_odometer"] == 29150.377

  def test_longitudinal_command_does_not_reference_missing_jerk_attr(self):
    CP = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
    tesla_can = TeslaCAN(CP, self.DummyPacker())

    name, bus, values = tesla_can.create_longitudinal_command(4, 1.0, 0, 20.0, True, False)

    assert name == "DAS_control"
    assert bus == 0
    assert values["DAS_jerkMax"] <= 4.9
    assert values["DAS_jerkMax"] >= 0.0

  def test_longitudinal_jerk_accumulates_and_resets(self):
    CP = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
    tesla_can = TeslaCAN(CP, self.DummyPacker())

    first = tesla_can.create_longitudinal_command(4, 1.0, 0, 20.0, True, False)[2]
    second = tesla_can.create_longitudinal_command(4, 1.0, 1, 20.0, True, False)[2]
    inactive = tesla_can.create_longitudinal_command(4, 0.0, 2, 20.0, False, False)[2]

    assert second["DAS_jerkMax"] > first["DAS_jerkMax"] > 0.0
    assert inactive["DAS_jerkMax"] == 0.0

  def test_longitudinal_comfort_mode_hysteresis(self):
    CP = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
    tesla_can = TeslaCAN(CP, self.DummyPacker())

    coast = tesla_can.create_longitudinal_command(4, 0.0, 0, 20.0, True, False, set_speed_kph=64.0, comfort_mode=True)[2]
    drive = tesla_can.create_longitudinal_command(4, 0.08, 1, 20.0, True, False, set_speed_kph=64.0, comfort_mode=True)[2]
    drive_held = tesla_can.create_longitudinal_command(4, 0.04, 2, 20.0, True, False, set_speed_kph=64.0, comfort_mode=True)[2]
    coast_again = tesla_can.create_longitudinal_command(4, 0.03, 3, 20.0, True, False, set_speed_kph=64.0, comfort_mode=True)[2]
    regen = tesla_can.create_longitudinal_command(4, -0.08, 4, 20.0, True, False, set_speed_kph=64.0, comfort_mode=True)[2]
    regen_held = tesla_can.create_longitudinal_command(4, -0.04, 5, 20.0, True, False, set_speed_kph=64.0, comfort_mode=True)[2]
    final_coast = tesla_can.create_longitudinal_command(4, -0.03, 6, 20.0, True, False, set_speed_kph=64.0, comfort_mode=True)[2]

    assert coast["DAS_setSpeed"] == 72.0
    assert coast["DAS_jerkMin"] == -0.7
    assert coast["DAS_jerkMax"] == 0.7
    assert drive["DAS_setSpeed"] == 64.0
    assert drive_held["DAS_setSpeed"] == 64.0
    assert coast_again["DAS_setSpeed"] == 72.0
    assert regen["DAS_setSpeed"] == 0
    assert regen_held["DAS_setSpeed"] == 0
    assert final_coast["DAS_setSpeed"] == 72.0

  def test_longitudinal_comfort_mode_crosses_directly_between_drive_and_regen(self):
    CP = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
    tesla_can = TeslaCAN(CP, self.DummyPacker())

    tesla_can.create_longitudinal_command(4, 0.08, 0, 20.0, True, False, comfort_mode=True)
    regen = tesla_can.create_longitudinal_command(4, -0.08, 1, 20.0, True, False, comfort_mode=True)[2]
    drive = tesla_can.create_longitudinal_command(4, 0.08, 2, 20.0, True, False, set_speed_kph=64.0, comfort_mode=True)[2]

    assert regen["DAS_setSpeed"] == 0
    assert drive["DAS_setSpeed"] == 64.0

  def test_longitudinal_comfort_mode_stopping_bypasses_coast(self):
    CP = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
    tesla_can = TeslaCAN(CP, self.DummyPacker())

    values = tesla_can.create_longitudinal_command(4, -0.01, 0, 20.0, True, False, comfort_mode=True, stopping=True)[2]

    assert values["DAS_setSpeed"] == 0
    assert values["DAS_jerkMin"] == -4.9

  def test_route_zero_crossing_commands_stay_in_coast(self):
    CP = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
    tesla_can = TeslaCAN(CP, self.DummyPacker())
    commands = [0.001925, -0.007605, -0.003657, 0.001745, 0.014604, -0.000265, -0.008631, 0.003343,
                0.000620, -0.001185, -0.000301, 0.003632, 0.006865, -0.003242, -0.000854, 0.016680]

    set_speeds = [
      tesla_can.create_longitudinal_command(4, accel, counter % 8, 11.18, True, False, comfort_mode=True)[2]["DAS_setSpeed"]
      for counter, accel in enumerate(commands)
    ]

    assert set(set_speeds) == {11.18 * 3.6}

  def test_longitudinal_command_uses_explicit_set_speed(self):
    CP = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
    tesla_can = TeslaCAN(CP, self.DummyPacker())

    name, bus, values = tesla_can.create_longitudinal_command(4, 1.0, 0, 20.0, True, False, set_speed_kph=64.0)

    assert name == "DAS_control"
    assert bus == 0
    assert values["DAS_setSpeed"] == 64.0

  def test_longitudinal_command_preserves_decel_when_explicit_set_speed_present(self):
    CP = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
    tesla_can = TeslaCAN(CP, self.DummyPacker())

    name, bus, values = tesla_can.create_longitudinal_command(4, -0.5, 0, 20.0, True, False, set_speed_kph=64.0)

    assert name == "DAS_control"
    assert bus == 0
    assert values["DAS_setSpeed"] == 0
    assert values["DAS_accelMin"] < 0


class TestTeslaCarControllerIQParams:
  def test_iq_params_override_set_speed(self):
    CP = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
    CP.openpilotLongitudinalControl = True
    controller = CarController(CAR.TESLA_MODEL_3.config.dbc_dict, CP, structs.IQCarParams())

    class DummyCruise:
      cancel = False

    class DummyActuators:
      steeringAngleDeg = 0.0
      accel = 1.0
      longControlState = structs.CarControl.Actuators.LongControlState.pid

      def as_builder(self):
        return self

    class DummyCarControl:
      actuators = DummyActuators()
      latActive = False
      longActive = True
      cruiseControl = DummyCruise()

    class DummyCarState:
      hands_on_level = 0
      out = type("Out", (), {"vEgoRaw": 20.0, "steeringAngleDeg": 0.0, "steeringRateDeg": 0.0, "steeringTorque": 0.0, "vEgo": 20.0})()
      das_accCancel = False
      cruise_override = False
      das_control = {"DAS_controlCounter": 0}

    cc_iq = structs.IQCarControl(params=[
      structs.IQCarControl.Param(
        key="enhancedStockLongitudinalControl.setSpeedKph",
        type="float",
        value=b"64.0",
      )
    ])

    captured = {}

    def fake_longitudinal_command(state, accel, cntr, v_ego, active, cruise_override, set_speed_kph=None, **kwargs):
      captured["set_speed_kph"] = set_speed_kph
      return ("DAS_control", 0, {"DAS_setSpeed": set_speed_kph})

    controller.tesla_can.create_longitudinal_command = fake_longitudinal_command

    new_actuators, _ = controller.update(DummyCarControl(), cc_iq, DummyCarState(), 0)
    assert captured["set_speed_kph"] == 64.0
    assert new_actuators.accel == 1.0

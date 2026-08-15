import pytest

from opendbc.car import structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.tesla.radar_interface import RadarInterface as OEMRadarInterface
from opendbc.car.tesla.values import CAR
from opendbc.sunnypilot.car.interfaces import _initialize_tesla_radar_backend
from opendbc.sunnypilot.car.tesla.ars408.interface import ARS408RadarInterface
from opendbc.sunnypilot.car.tesla.ars408.selector import RadarInterface
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP, TeslaSafetyFlagsSP


def params(*, radar_unavailable: bool = True) -> tuple[structs.CarParams, structs.CarParamsSP]:
  cp, cp_sp = structs.CarParams(), structs.CarParamsSP()
  cp.brand = "tesla"
  cp.carFingerprint = CAR.TESLA_MODEL_3
  cp.radarUnavailable = radar_unavailable
  return cp, cp_sp


def test_default_mode_returns_real_oem_interface_without_changing_availability() -> None:
  cp, cp_sp = params(radar_unavailable=False)
  _initialize_tesla_radar_backend(cp, cp_sp, {"TeslaRadarBackend": "0"})
  assert isinstance(RadarInterface(cp, cp_sp), OEMRadarInterface)
  assert not cp.radarUnavailable
  assert not cp_sp.flags & TeslaFlagsSP.ARS408_RADAR


def test_ars408_mode_sets_cached_flags_and_returns_isolated_backend() -> None:
  cp, cp_sp = params()
  _initialize_tesla_radar_backend(cp, cp_sp, {"TeslaRadarBackend": "1"})
  assert not cp.radarUnavailable
  assert cp.deprecated.radarTimeStep == pytest.approx(1.0 / 14.0)
  assert cp_sp.flags & TeslaFlagsSP.ARS408_RADAR
  assert cp_sp.safetyParam & TeslaSafetyFlagsSP.ARS408_RADAR
  assert isinstance(RadarInterface(cp, cp_sp), ARS408RadarInterface)


def test_off_and_invalid_modes_fail_closed() -> None:
  for value in ("2", "invalid", "99"):
    cp, cp_sp = params(radar_unavailable=False)
    _initialize_tesla_radar_backend(cp, cp_sp, {"TeslaRadarBackend": value})
    assert cp.radarUnavailable
    assert cp_sp.flags & TeslaFlagsSP.RADAR_DISABLED
    selected = RadarInterface(cp, cp_sp)
    assert type(selected) is RadarInterfaceBase

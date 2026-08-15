from opendbc.car import structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR, RadarInterface as OEMRadarInterface
from opendbc.sunnypilot.car.tesla.ars408.interface import ARS408RadarInterface
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP


def RadarInterface(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> RadarInterfaceBase:
  if CP_SP.flags & TeslaFlagsSP.ARS408_RADAR:
    return ARS408RadarInterface(CP, CP_SP)
  if CP_SP.flags & TeslaFlagsSP.RADAR_DISABLED:
    return RadarInterfaceBase(CP, CP_SP)
  return OEMRadarInterface(CP, CP_SP)


__all__ = ("RADAR_START_ADDR", "RadarInterface")

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.values import VolkswagenFlags
from opendbc.sunnypilot.car.volkswagen.vze_speed_limit_hold import VzeSpeedLimitHold


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self._vze_hold = VzeSpeedLimitHold()

  def update_vze_speed_limit(self, ret_sp, cam_cp, v_ego: float) -> None:
    if not (self.CP.flags & VolkswagenFlags.MEB):
      return
    vze = cam_cp.vl["VZE_04"]
    raw = vze["VZE_Verkehrszeichen_1"]
    display_mode = vze["VZE_Anzeigemodus"]
    kph = raw * CV.MPH_TO_KPH if display_mode == 1 else raw
    accepted_kph = self._vze_hold.update(float(kph or 0.0), v_ego)
    ret_sp.speedLimit = accepted_kph * CV.KPH_TO_MS if accepted_kph > 0 else 0.0

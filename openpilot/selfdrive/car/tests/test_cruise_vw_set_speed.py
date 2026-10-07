import sys
from unittest.mock import MagicMock

# Host runs lack libparams/msgq; the set-speed path touches neither.
_params_mod = MagicMock()
_params_mod.UnknownKeyName = type("UnknownKeyName", (Exception,), {})
sys.modules.setdefault("openpilot.common.params", _params_mod)
sys.modules.setdefault(
  "openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist",
  MagicMock(ACTIVE_STATES=(1, 2)),
)
sys.modules.setdefault(
  "openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers",
  MagicMock(compare_cluster_target=lambda *a, **k: (False, False)),
)

from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_UNSET

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


def _vw_helper() -> VCruiseHelper:
  CP = car.CarParams(brand="volkswagen", openpilotLongitudinalControl=True, pcmCruise=False)
  CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
  h = VCruiseHelper(CP, CP_SP)
  h.get_minimum_set_speed(True)
  return h


def test_vw_set_takes_current_speed_in_experimental():
  for v_kph in (0.0, 20.0, 50.0, 80.0, 120.0):
    h = _vw_helper()
    h.initialize_v_cruise(car.CarState(vEgo=v_kph * CV.KPH_TO_MS), True, False)
    assert h.v_cruise_kph == max(round(v_kph), h.v_cruise_min)


def test_vw_resume_keeps_last_set_speed():
  h = _vw_helper()
  h.initialize_v_cruise(car.CarState(vEgo=60 * CV.KPH_TO_MS), True, False)
  h.v_cruise_kph_last = h.v_cruise_kph
  CS = car.CarState(vEgo=40 * CV.KPH_TO_MS, buttonEvents=[ButtonEvent(type=ButtonType.resumeCruise, pressed=False)])
  h.initialize_v_cruise(CS, True, False)
  assert h.v_cruise_kph == 60


def test_non_vw_keeps_experimental_floor():
  h = VCruiseHelper(car.CarParams(brand="toyota", pcmCruise=False), custom.CarParamsSP(pcmCruiseSpeed=False))
  h.get_minimum_set_speed(True)
  h.initialize_v_cruise(car.CarState(vEgo=50 * CV.KPH_TO_MS), True, False)
  assert h.v_cruise_kph == 105
  assert h.v_cruise_kph != V_CRUISE_UNSET

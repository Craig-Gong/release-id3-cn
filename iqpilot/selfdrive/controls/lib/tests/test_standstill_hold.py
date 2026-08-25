from types import SimpleNamespace

# Match openpilot.common.realtime.DT_MDL without importing the full stack.
DT_MDL = 0.05
_STANDSTILL_HOLD_RELEASE_S = 1.0


def _load_planner_cls():
  from openpilot.iqpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerIQ
  return LongitudinalPlannerIQ


def _build_planner(*, nav_stop=False, forcing_stop=False, model_stop=False):
  LongitudinalPlannerIQ = _load_planner_cls()
  planner = LongitudinalPlannerIQ.__new__(LongitudinalPlannerIQ)
  planner.nav_stop_request = nav_stop
  planner.forcing_stop = forcing_stop
  planner.iq_dynamic = SimpleNamespace(stop_light_detected=model_stop, model_stopped=model_stop)
  planner._standstill_hold = False
  planner._standstill_hold_s = 0.0
  planner._green_launch = False
  planner._hold_released = False
  return planner


def _sm(*, standstill=True, gas=False, accel=False, v_ego=0.0, light="none"):
  return {
    "carState": SimpleNamespace(standstill=standstill, gasPressed=gas, vEgo=v_ego),
    "iqCarState": SimpleNamespace(accelPressed=accel),
    "iqNavState": SimpleNamespace(trafficLight=light),
  }


def test_hold_blocks_brief_model_go():
  planner = _build_planner()
  sm = _sm()
  should_stop, a_target = planner.apply_standstill_hold(True, -0.4, 0.0, sm)
  assert should_stop
  assert planner._standstill_hold

  should_stop, a_target = planner.apply_standstill_hold(False, 0.8, 0.0, sm)
  assert should_stop
  assert a_target <= 0.0
  assert planner._standstill_hold


def test_hold_releases_after_stable_go():
  planner = _build_planner()
  sm = _sm()
  planner.apply_standstill_hold(True, -0.4, 0.0, sm)
  for _ in range(int(_STANDSTILL_HOLD_RELEASE_S / DT_MDL)):
    should_stop, a_target = planner.apply_standstill_hold(False, 0.8, 0.0, sm)
  assert should_stop is False
  assert a_target == 0.8
  assert planner._standstill_hold is False


def test_nav_red_keeps_hold_even_if_model_wants_go():
  planner = _build_planner(nav_stop=True)
  sm = _sm(light="red")
  planner.apply_standstill_hold(True, -2.0, 0.0, sm)
  for _ in range(int(2.0 / DT_MDL)):
    should_stop, a_target = planner.apply_standstill_hold(False, 0.8, 0.0, sm)
  assert should_stop
  assert a_target <= 0.0
  assert planner._standstill_hold


def test_left_arrow_red_holds_without_nav_stop_request():
  # Approach curve can leave speedTarget > 0; left-arrow red must still hold.
  planner = _build_planner(nav_stop=False)
  sm = _sm(light="red")
  sm["iqNavState"] = SimpleNamespace(
    trafficLight="red",
    nextManeuverType=SimpleNamespace(name="turn"),
    nextManeuverDirection=SimpleNamespace(name="turnLeft"),
    nextManeuverDistance=40.0,
  )
  planner.apply_standstill_hold(True, -0.4, 0.0, sm)
  for _ in range(int(2.0 / DT_MDL)):
    should_stop, a_target = planner.apply_standstill_hold(False, 0.8, 0.0, sm)
  assert should_stop
  assert a_target <= 0.0
  assert planner._standstill_hold


def test_gas_releases_hold():
  planner = _build_planner(nav_stop=True)
  sm_stop = _sm(light="red")
  planner.apply_standstill_hold(True, -2.0, 0.0, sm_stop)
  should_stop, a_target = planner.apply_standstill_hold(False, 0.8, 0.0, _sm(gas=True, light="red"))
  assert should_stop is False
  assert a_target == 0.8
  assert planner._standstill_hold is False


def test_model_stop_does_not_pin_after_stable_go():
  # Sticky vision-stop at the line must not keep the hold once shouldStop is clear.
  planner = _build_planner(model_stop=True)
  sm = _sm()
  planner.apply_standstill_hold(True, -0.5, 0.0, sm)
  for _ in range(int(_STANDSTILL_HOLD_RELEASE_S / DT_MDL)):
    should_stop, a_target = planner.apply_standstill_hold(False, 0.8, 0.0, sm)
  assert should_stop is False
  assert a_target == 0.8
  assert planner._standstill_hold is False
  should_stop, a_target = planner.apply_standstill_hold(False, 0.8, 0.0, sm)
  assert should_stop is False
  assert planner._standstill_hold is False


def test_nav_green_releases_despite_vision_stop():
  planner = _build_planner(nav_stop=True, model_stop=True)
  planner.apply_standstill_hold(True, -2.0, 0.0, _sm(light="red"))
  planner.nav_stop_request = False
  sm_green = _sm(light="green")
  for _ in range(int(_STANDSTILL_HOLD_RELEASE_S / DT_MDL)):
    should_stop, a_target = planner.apply_standstill_hold(True, -0.4, 0.0, sm_green)
  assert should_stop is False
  assert planner._standstill_hold is False
  should_stop, a_target = planner.apply_standstill_hold(True, -0.4, 0.0, sm_green)
  assert should_stop is False
  assert planner._standstill_hold is False

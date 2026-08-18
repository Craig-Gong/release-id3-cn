from types import SimpleNamespace

from opendbc.sunnypilot.car.tesla.speed_limit_controller import TeslaSpeedLimitController, create_speed_wheel_frame
from opendbc.sunnypilot.car.tesla.carstate_ext import CarStateExt
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP


IDLE_TEMPLATE = bytes.fromhex("2955000000000080")


def fake_state(current_speed=20.0, target_speed=25.0, template_time=1_000_000_000,
               speed_units="KPH", stock_longitudinal=False, autopilot_active=False):
  return SimpleNamespace(
    out=SimpleNamespace(
      cruiseState=SimpleNamespace(enabled=True, speedCluster=current_speed),
      brakePressed=False,
    ),
    tesla_speed_limit_target=target_speed,
    tesla_speed_limit_target_valid=True,
    tesla_speed_button_template=IDLE_TEMPLATE,
    tesla_speed_button_template_nanos=template_time,
    tesla_speed_units=speed_units,
    tesla_stock_longitudinal_active=stock_longitudinal,
    tesla_autopilot_active=autopilot_active,
    tesla_manual_speed_adjustment_counter=0,
    tesla_speed_auto_resume_gesture_counter=0,
  )


def fake_control(enabled=True, cancel=False):
  return SimpleNamespace(enabled=enabled, cruiseControl=SimpleNamespace(cancel=cancel))


def test_speed_wheel_frame_changes_only_signed_right_tick():
  assert create_speed_wheel_frame(IDLE_TEMPLATE, 1) == bytes.fromhex("2955000100000080")
  assert create_speed_wheel_frame(IDLE_TEMPLATE, -1) == bytes.fromhex("2955003f00000080")


def test_controller_sends_one_tick_then_waits_for_speed_feedback():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state(template_time=2_000_000_000)

  assert controller.update(fake_control(), state, 1_050_000_000) == []
  sends = controller.update(fake_control(), state, 1_550_000_000)
  assert len(sends) == 1
  assert sends[0].address == 0x3C2
  assert sends[0].src == 1
  assert sends[0].dat == bytes.fromhex("2955000100000080")
  assert controller.remaining_steps == 18
  assert controller.update(fake_control(), state, 1_900_000_000) == []

  state.out.cruiseState.speedCluster = 20.3
  assert len(controller.update(fake_control(), state, 2_150_000_000)) == 1
  state.tesla_speed_button_template_nanos = 2_200_000_000
  assert controller.update(fake_control(), state, 2_400_000_000) == []


def test_first_target_waits_and_ignores_intermediate_max_change():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state(current_speed=110.0 / 3.6, target_speed=110.0 / 3.6, template_time=2_000_000_000)

  assert controller.update(fake_control(), state, 1_000_000_000) == []
  state.tesla_speed_limit_target = 100.0 / 3.6
  assert controller.update(fake_control(), state, 1_200_000_000) == []

  # A MAX change while the target is settling is not a manual override and
  # must not prevent the final 100 km/h correction.
  state.out.cruiseState.speedCluster = 109.0 / 3.6
  assert controller.update(fake_control(), state, 1_400_000_000) == []
  assert not controller.manual_override_active
  sends = controller.update(fake_control(), state, 1_700_000_000)
  assert len(sends) == 1
  assert sends[0].dat == bytes.fromhex("2955003f00000080")


def test_first_transient_110_target_never_ticks_before_settling_at_100():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state(current_speed=100.0 / 3.6, target_speed=110.0 / 3.6, template_time=2_000_000_000)

  assert controller.update(fake_control(), state, 1_000_000_000) == []
  assert controller.update(fake_control(), state, 1_150_000_000) == []
  state.tesla_speed_limit_target = 100.0 / 3.6
  assert controller.update(fake_control(), state, 1_200_000_000) == []
  assert controller.update(fake_control(), state, 1_700_000_000) == []
  assert controller.last_tx_nanos == 0


def test_controller_waits_for_changed_target_to_stabilize_before_tick():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state(current_speed=25.0, target_speed=25.0, template_time=1_500_000_000)
  assert controller.update(fake_control(), state, 1_000_000_000) == []

  state.out.cruiseState.speedCluster = 20.0
  state.tesla_speed_limit_target = 26.0
  assert controller.update(fake_control(), state, 1_050_000_000) == []
  assert controller.update(fake_control(), state, 1_490_000_000) == []
  assert len(controller.update(fake_control(), state, 1_550_000_000)) == 1


def test_controller_quantizes_target_in_vehicle_display_units():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  mph = 0.44704

  state = fake_state(current_speed=60.0 * mph, target_speed=60.4 * mph, speed_units="MPH")
  assert controller.update(fake_control(), state, 1_050_000_000) == []

  state.tesla_speed_limit_target = 60.6 * mph
  assert controller.update(fake_control(), state, 1_060_000_000) == []
  assert len(controller.update(fake_control(), state, 1_560_000_000)) == 1
  assert controller.remaining_steps == 1


def test_controller_operates_during_stock_longitudinal_source():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state(stock_longitudinal=True)

  assert controller.update(fake_control(), state, 1_050_000_000) == []
  state.tesla_speed_button_template_nanos = 1_550_000_000
  assert len(controller.update(fake_control(), state, 1_550_000_000)) == 1


def test_controller_never_sends_speed_ticks_while_autopilot_is_active():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state(autopilot_active=True, template_time=3_000_000_000)

  assert controller.update(fake_control(), state, 1_000_000_000) == []
  assert controller.update(fake_control(), state, 1_600_000_000) == []
  assert controller.update(fake_control(), state, 2_200_000_000) == []
  assert controller.last_tx_nanos == 0


def test_controller_does_not_retry_forever_without_feedback():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state(template_time=2_500_000_000)

  assert controller.update(fake_control(), state, 1_050_000_000) == []
  assert len(controller.update(fake_control(), state, 1_550_000_000)) == 1
  assert controller.update(fake_control(), state, 2_800_000_000) == []
  assert controller.update(fake_control(), state, 3_000_000_000) == []

  state.out.cruiseState.speedCluster += 1.0 / 3.6
  state.tesla_speed_button_template_nanos = 3_000_000_000
  assert len(controller.update(fake_control(), state, 3_100_000_000)) == 1


def test_manual_adjustment_pauses_until_up_down_resume_gesture():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state(current_speed=25.0, target_speed=25.0, template_time=2_000_000_000)
  assert controller.update(fake_control(), state, 1_050_000_000) == []
  assert controller.update(fake_control(), state, 1_550_000_000) == []

  state.out.cruiseState.speedCluster = 26.0
  state.tesla_manual_speed_adjustment_counter = 1
  assert controller.update(fake_control(), state, 1_600_000_000) == []
  assert controller.manual_override_active

  state.tesla_manual_speed_adjustment_counter = 2
  state.tesla_speed_auto_resume_gesture_counter = 1
  state.tesla_speed_button_template_nanos = 1_650_000_000
  assert len(controller.update(fake_control(), state, 1_650_000_000)) == 1
  assert not controller.manual_override_active


def test_resume_gesture_ignores_repeated_second_direction_until_wheel_returns_idle():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state(current_speed=26.0, target_speed=25.0, template_time=1_000_000_000)
  state._tesla_speed_resume_up_nanos = 0
  state._tesla_speed_resume_down_nanos = 0
  state._tesla_speed_resume_wait_idle = False
  control = fake_control()
  up = bytes.fromhex("2955000100000080")
  down = bytes.fromhex("2955003f00000080")

  assert controller.update(control, state, 1_000_000_000) == []

  CarStateExt.update_speed_button_template(state, up, 1_100_000_000)
  assert controller.update(control, state, 1_100_000_000) == []
  assert controller.manual_override_active

  CarStateExt.update_speed_button_template(state, down, 1_200_000_000)
  controller.update(control, state, 1_200_000_000)
  assert not controller.manual_override_active

  # A physical detent can repeat its non-zero 0x3C2 value before returning
  # idle. It belongs to the completed gesture and must not re-arm override.
  CarStateExt.update_speed_button_template(state, down, 1_250_000_000)
  controller.update(control, state, 1_250_000_000)
  assert not controller.manual_override_active

  CarStateExt.update_speed_button_template(state, IDLE_TEMPLATE, 1_300_000_000)
  CarStateExt.update_speed_button_template(state, up, 1_400_000_000)
  controller.update(control, state, 1_400_000_000)
  assert controller.manual_override_active


def test_resume_gesture_ignores_delayed_manual_speed_feedback():
  up = bytes.fromhex("2955000100000080")
  down = bytes.fromhex("2955003f00000080")

  for first_direction, second_direction in ((up, down), (down, up)):
    controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
    state = fake_state(current_speed=25.0, target_speed=25.0, template_time=2_000_000_000)
    state._tesla_speed_resume_up_nanos = 0
    state._tesla_speed_resume_down_nanos = 0
    state._tesla_speed_resume_wait_idle = False
    control = fake_control()

    assert controller.update(control, state, 1_000_000_000) == []
    assert controller.update(control, state, 1_500_000_000) == []

    # The wheel event can reach CarState before the corresponding set-speed
    # feedback. Manual override must still be active at this point.
    CarStateExt.update_speed_button_template(state, up, 1_600_000_000)
    assert controller.update(control, state, 1_600_000_000) == []
    assert controller.manual_override_active
    CarStateExt.update_speed_button_template(state, IDLE_TEMPLATE, 1_700_000_000)

    # Start a fresh gesture after the preceding manual event has aged out.
    CarStateExt.update_speed_button_template(state, first_direction, 3_300_000_000)
    assert controller.update(control, state, 3_300_000_000) == []
    assert controller.manual_override_active
    CarStateExt.update_speed_button_template(state, IDLE_TEMPLATE, 3_350_000_000)
    CarStateExt.update_speed_button_template(state, second_direction, 3_400_000_000)
    controller.update(control, state, 3_400_000_000)
    assert not controller.manual_override_active

    # Tesla can publish the set-speed result after the gesture counter. This is
    # feedback from the completed manual gesture, not a new manual adjustment.
    state.out.cruiseState.speedCluster = 26.0
    state.tesla_speed_button_template_nanos = 3_500_000_000
    controller.update(control, state, 3_500_000_000)
    assert not controller.manual_override_active


def test_manual_override_clears_when_speed_limit_changes():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state(current_speed=25.0, target_speed=25.0)
  assert controller.update(fake_control(), state, 1_050_000_000) == []

  state.out.cruiseState.speedCluster = 26.0
  state.tesla_manual_speed_adjustment_counter = 1
  assert controller.update(fake_control(), state, 1_100_000_000) == []
  assert controller.manual_override_active

  state.tesla_speed_limit_target = 27.0
  state.tesla_speed_button_template_nanos = 1_150_000_000
  assert controller.update(fake_control(), state, 1_150_000_000) == []
  state.tesla_speed_button_template_nanos = 1_650_000_000
  assert len(controller.update(fake_control(), state, 1_650_000_000)) == 1
  assert not controller.manual_override_active


def test_manual_override_clears_after_cruise_disengages():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state(current_speed=25.0, target_speed=25.0)
  assert controller.update(fake_control(), state, 1_050_000_000) == []

  state.out.cruiseState.speedCluster = 26.0
  state.tesla_manual_speed_adjustment_counter = 1
  assert controller.update(fake_control(), state, 1_100_000_000) == []
  assert controller.manual_override_active

  state.out.cruiseState.enabled = False
  assert controller.update(fake_control(enabled=False), state, 1_150_000_000) == []
  assert not controller.manual_override_active

  state.out.cruiseState.enabled = True
  state.tesla_speed_button_template_nanos = 1_700_000_000
  assert controller.update(fake_control(), state, 1_200_000_000) == []
  assert len(controller.update(fake_control(), state, 1_700_000_000)) == 1


def test_controller_stops_at_target_and_when_controls_are_inactive():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state(current_speed=25.0)
  assert controller.update(fake_control(), state, 1_050_000_000) == []

  state.out.cruiseState.speedCluster = 20.0
  assert controller.update(fake_control(enabled=False), state, 1_060_000_000) == []
  assert controller.update(fake_control(cancel=True), state, 1_070_000_000) == []


def test_controller_rejects_stale_template_or_invalid_limit():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state(template_time=1_000_000_000)
  assert controller.update(fake_control(), state, 2_500_000_001) == []

  state.tesla_speed_button_template_nanos = 1_500_000_000
  state.tesla_speed_limit_target_valid = False
  assert controller.update(fake_control(), state, 1_600_000_000) == []

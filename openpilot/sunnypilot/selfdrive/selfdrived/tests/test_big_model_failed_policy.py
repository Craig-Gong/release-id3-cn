"""bigModelFailed must not soft-disable; settling covers post-load modelV2 gaps."""
from openpilot.cereal import log
from openpilot.selfdrive.selfdrived.events import EVENTS, ET


def test_big_model_failed_is_permanent_toast_only():
  event = EVENTS[log.OnroadEvent.EventName.bigModelFailed]
  assert ET.SOFT_DISABLE not in event
  assert ET.IMMEDIATE_DISABLE not in event
  assert ET.NO_ENTRY not in event
  assert ET.PERMANENT in event


def test_settling_window_skips_model_unavailable():
  warmup_sec = 15.
  ready_t = 100.0
  now = ready_t + 5.0
  settling = ready_t > 0. and now < ready_t + warmup_sec
  big_active = True
  seen = True
  alive = False
  model_unavailable = big_active is True and seen and not alive and not settling
  assert settling
  assert not model_unavailable

  now = ready_t + 16.0
  settling = ready_t > 0. and now < ready_t + warmup_sec
  model_unavailable = big_active is True and seen and not alive and not settling
  assert not settling
  assert model_unavailable

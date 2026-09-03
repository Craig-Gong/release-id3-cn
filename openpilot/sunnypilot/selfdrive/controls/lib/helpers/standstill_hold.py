"""Hold after a model / vision stop until go is stable. No IQ-link / remainS."""
from openpilot.common.realtime import DT_MDL

_STANDSTILL_HOLD_RELEASE_S = 1.0
_RELEASE_V_EGO = 2.0
_STANDSTILL_V = 0.3


class StandstillHold:
  def __init__(self):
    self.hold = False
    self.hold_s = 0.0
    self.hold_released = False

  def reset(self) -> None:
    self.hold = False
    self.hold_s = 0.0
    self.hold_released = False

  def apply(self, should_stop: bool, a_target: float, v_ego: float, *,
            standstill: bool, gas: bool, model_stop: bool) -> tuple[bool, float]:
    if gas or v_ego > _RELEASE_V_EGO:
      self.reset()
      return should_stop, a_target

    at_rest = standstill or v_ego <= _STANDSTILL_V
    if at_rest:
      arm = should_stop if self.hold_released else (should_stop or model_stop)
      if arm:
        self.hold = True
        self.hold_s = 0.0
      elif self.hold:
        self.hold_s += DT_MDL
        if self.hold_s >= _STANDSTILL_HOLD_RELEASE_S:
          self.hold = False
          self.hold_released = True
      if self.hold:
        return True, min(float(a_target), 0.0)
    else:
      self.hold_released = False
      if self.hold:
        self.hold = False
        self.hold_s = 0.0
    return should_stop, a_target

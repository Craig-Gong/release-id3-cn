"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""

from iqpilot.common.params import Params, UnknownKeyName
from iqpilot.selfdrive.longitudinal_settings import (
  LONGITUDINAL_MODE_DYNAMIC,
  LONGITUDINAL_MODE_PILOT,
  PERSONALITY_VALUES,
  apply_longitudinal_mode,
  get_follow_distance_state,
  get_longitudinal_mode,
  set_valid_personality,
)
from iqpilot.selfdrive.ui.mici.widgets.stock_button import BigButton, BigMultiToggle, BigToggle, BigParamControl
from iqpilot.system.ui.lib.multilang import tr


class SafeParamControl(BigParamControl):
  """BigParamControl that tolerates a param missing from the COMPILED params registry.

  A key added to params_keys.h only exists at runtime once params_pyx.so is rebuilt; a
  .py-only / stale prebuilt deploy leaves get_bool/put_bool raising UnknownKeyName, which
  would crash the UI on construction. Default to `default_on` for display and no-op the
  write instead of crashing — mirrors the plannerd defensive read in long_mpc.py.
  """

  def __init__(self, text: str, param: str, default_on: bool = True, toggle_callback=None):
    self._default_on = default_on
    BigToggle.__init__(self, text, "", toggle_callback=toggle_callback)
    self.param = param
    self.params = Params()
    self.set_checked(self._safe_get())

  def _safe_get(self) -> bool:
    try:
      return self.params.get_bool(self.param)
    except UnknownKeyName:
      return self._default_on

  def refresh(self):
    self.set_checked(self._safe_get())

  def _handle_mouse_release(self, mouse_pos):
    BigToggle._handle_mouse_release(self, mouse_pos)
    try:
      self.params.put_bool(self.param, self._checked)
    except UnknownKeyName:
      pass


class MappedParamToggle(BigMultiToggle):
  """Multi-option toggle whose options map to arbitrary param values (int or float, drum-style).

  Up to PILL_LIMIT options render as the stock vertical pill column; more options would
  overflow the box, so they instead show the current value as a sub-label and cycle on tap.
  """
  PILL_LIMIT = 4

  def __init__(self, text: str, param: str, options: list[str], values: list | None = None):
    super().__init__(text, options)
    self._param = param
    self._values = values if values is not None else list(range(len(options)))
    self._params = Params()
    self.refresh()

  def _value_only(self) -> bool:
    return len(self._options) > self.PILL_LIMIT

  def _width_hint(self) -> int:
    if self._value_only():
      return BigButton._width_hint(self)
    return super()._width_hint()

  def _draw_content(self, btn_x: float, btn_y: float, btn_width: float, btn_height: float):
    if self._value_only():
      BigButton._draw_content(self, btn_x, btn_y, btn_width, btn_height)
    else:
      super()._draw_content(btn_x, btn_y, btn_width, btn_height)

  def refresh(self):
    try:
      raw = self._params.get(self._param, return_default=True)
    except UnknownKeyName:
      raw = self._values[0]
    try:
      cur = float(raw)
    except (TypeError, ValueError):
      cur = float(self._values[0])
    idx = min(range(len(self._values)), key=lambda i: abs(float(self._values[i]) - cur))
    self.set_value(self._options[idx])

  def _handle_mouse_release(self, mouse_pos):
    super()._handle_mouse_release(mouse_pos)
    idx = self._options.index(self.value)
    try:
      self._params.put(self._param, self._values[idx])
    except UnknownKeyName:
      pass


class FollowDistanceSelector(BigMultiToggle):
  OPTIONS = ["aggressive", "standard", "relaxed", "stock"]

  def __init__(self):
    self._display_options = [tr(option) for option in self.OPTIONS]
    super().__init__(tr("Follow Distance"), self._display_options)
    self._params = Params()
    self.refresh()

  def refresh(self):
    selection, enabled = get_follow_distance_state(self._params)
    self.set_value(self._display_options[3 if selection is None else selection])
    self.set_enabled(enabled)

  def _handle_mouse_release(self, mouse_pos):
    if get_longitudinal_mode(self._params) not in (LONGITUDINAL_MODE_DYNAMIC, LONGITUDINAL_MODE_PILOT):
      return
    BigButton._handle_mouse_release(self, mouse_pos)
    selection, _ = get_follow_distance_state(self._params)
    if selection is None:
      self.refresh()
      return
    next_selection = PERSONALITY_VALUES[(PERSONALITY_VALUES.index(selection) + 1) % len(PERSONALITY_VALUES)]
    set_valid_personality(self._params, next_selection)
    self.set_value(self._display_options[next_selection])


class IQModeSelector(BigMultiToggle):
  """Longitudinal mode selector: Stock ACC / IQ.Chill / IQ.Dynamic / IQ.Pilot.

  A single tap cycles to the next mode and applies the matching param combo immediately.
  """
  OPTIONS = ["Stock ACC", "IQ.Chill", "IQ.Dynamic", "IQ.Pilot"]

  def __init__(self, mode_callback=None):
    self._display_options = [tr(option) for option in self.OPTIONS]
    super().__init__(tr("IQ Mode"), self._display_options)
    self._params = Params()
    self._mode_callback = mode_callback
    self.refresh()

  def _index(self) -> int:
    return get_longitudinal_mode(self._params)

  def is_dynamic(self) -> bool:
    return self._index() == 2

  def refresh(self):
    self.set_value(self._display_options[self._index()])

  def _apply(self, idx: int):
    apply_longitudinal_mode(self._params, idx)
    self._params.put_bool("OnroadCycleRequested", True)

  def _handle_mouse_release(self, mouse_pos):
    nxt = (self._index() + 1) % len(self.OPTIONS)
    self._apply(nxt)
    self.set_value(self._display_options[nxt])
    if self._mode_callback:
      self._mode_callback()

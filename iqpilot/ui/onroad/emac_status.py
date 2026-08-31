"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""

import time

import pyray as rl

from iqpilot.common.params import Params
from iqpilot.ui.onroad.big_model_status import SourceState, draw_source_label, resolve_source
from iqpilot.selfdrive.ui import UI_BORDER_SIZE
from iqpilot.selfdrive.ui.onroad.driver_state import BTN_SIZE
from iqpilot.selfdrive.ui.ui_state import ui_state
from iqpilot.system.ui.lib.application import FontWeight, gui_app
from iqpilot.system.ui.lib.text_measure import measure_text_cached
from iqpilot.system.ui.widgets import Widget

_POLL_S = 1.0
_FONT_SIZE = 70


class EmacStatusRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self._last_poll = 0.0
    self._label = ""
    self._state = SourceState.HIDDEN

  def update(self):
    now = time.monotonic()
    if now - self._last_poll < _POLL_S:
      return
    self._last_poll = now
    self._label, self._state = resolve_source(self._params, ui_state.engaged)

  def _render(self, rect: rl.Rectangle):
    if self._state == SourceState.HIDDEN:
      return
    size = measure_text_cached(self._font, self._label, _FONT_SIZE)
    x = rect.x + UI_BORDER_SIZE + BTN_SIZE // 2 - size.x / 2
    y = rect.y + rect.height / 2 - size.y / 2
    draw_source_label(self._font, self._label, self._state, rl.Vector2(x, y), _FONT_SIZE)

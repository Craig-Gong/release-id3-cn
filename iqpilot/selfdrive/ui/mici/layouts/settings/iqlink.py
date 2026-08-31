"""mici settings: primary「蓝牙」one-tap toggle + status color (no secondary panel).

Params: IqlinkEnabled, IqlinkBleLinkState, IqlinkBleConnected,
IqlinkBlePeerConnected, IqlinkBlePairFailed.

Lamp (product C): see openpilot.selfdrive.ui.lib.iqlink_status
PSK is fixed and not shown on the primary tile.
"""

from __future__ import annotations

import pyray as rl

from iqpilot.common.params import Params, UnknownKeyName
from iqpilot.iqlink.konn3kt_ble_exclusive import sync_konn3kt_ble_for_iqlink
from iqpilot.selfdrive.ui.lib.iqlink_status import iqlink_hmac_up, iqlink_status_color
from iqpilot.selfdrive.ui.mici.widgets.stock_button import BigButton
from iqpilot.system.ui.lib.application import gui_app, MousePos


class IqlinkBigButton(BigButton):
  """Primary settings tile: tap toggles IqlinkEnabled; corner lamp = link status."""

  def __init__(self):
    super().__init__(
      "iqlink",
      "",
      gui_app.texture("icons/iq/bluetooth.png", 56, 56, keep_aspect_ratio=True),
      translate=True,
    )
    self._params = Params()

  def _get_label_font_size(self):
    return 64

  def _bridge_on(self) -> bool:
    """IqlinkEnabled param. Do NOT name _enabled — Widget uses that as a bool attr."""
    try:
      return self._params.get_bool("IqlinkEnabled")
    except UnknownKeyName:
      return False

  def _link_state(self) -> int:
    try:
      return int(self._params.get("IqlinkBleLinkState") or 0)
    except (UnknownKeyName, TypeError, ValueError):
      try:
        return 2 if self._params.get_bool("IqlinkBleConnected") else 0
      except UnknownKeyName:
        return 0

  def _link_up(self) -> bool:
    return iqlink_hmac_up(self._params)

  def _status_color(self) -> rl.Color:
    return iqlink_status_color(self._params)

  def _value_text(self) -> str:
    # No PSK on primary tile (fixed PSK; phone enters offline).
    if self._link_up():
      return "connected"
    if self._bridge_on() and self._link_state() == 1:
      return "connecting"
    if self._bridge_on():
      return "disconnected"
    return ""

  def _update_state(self):
    super()._update_state()
    self.set_value(self._value_text())

  def _render(self, _):
    super()._render(_)
    rl.draw_circle(int(self._rect.x + 30), int(self._rect.y + 30), 9, self._status_color())

  def _handle_mouse_release(self, mouse_pos: MousePos):
    super()._handle_mouse_release(mouse_pos)
    try:
      new_val = not self._bridge_on()
      self._params.put_bool("IqlinkEnabled", new_val)
      sync_konn3kt_ble_for_iqlink(self._params, new_val)
      if new_val:
        # Rising edge also cleared in ble loop; clear fail so lamp leaves red promptly.
        try:
          self._params.put_bool("IqlinkBlePairFailed", False)
        except UnknownKeyName:
          pass
    except UnknownKeyName:
      pass
    self.set_value(self._value_text())

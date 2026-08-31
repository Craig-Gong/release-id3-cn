"""Shared iqlink BLE status lamp colors for settings + home.

Product (2026-07 拍板):
  home hide — IqlinkEnabled=false (do not show red when off)
  yellow    — enabled, waiting / reconnecting / SoftBus-only / retry
  green     — HMAC only (LinkState==2 / IqlinkBleConnected); SoftBus alone ≠ green
  red       — sticky pair_failed (should be rare; device auto-retries → yellow)
"""

from __future__ import annotations

import pyray as rl

from iqpilot.common.params import UnknownKeyName

LINK_CONNECTED = 2

STATUS_RED = rl.Color(0xE0, 0x3A, 0x3A, 255)
STATUS_YELLOW = rl.Color(0xFB, 0xBF, 0x24, 255)
STATUS_GREEN = rl.Color(0x22, 0xC5, 0x5E, 255)


def _bool_param(params, key: str, default: bool = False) -> bool:
  try:
    return bool(params.get_bool(key))
  except UnknownKeyName:
    return default
  except Exception:
    return default


def _link_state(params) -> int:
  try:
    return int(params.get("IqlinkBleLinkState") or 0)
  except (UnknownKeyName, TypeError, ValueError):
    return 2 if _bool_param(params, "IqlinkBleConnected") else 0


def iqlink_hmac_up(params) -> bool:
  """True when nav can be pushed (HMAC session). SoftBus alone is not enough."""
  if _bool_param(params, "IqlinkBleConnected"):
    return True
  return _link_state(params) >= LINK_CONNECTED


def iqlink_link_up(params) -> bool:
  """Alias: nav-pushable HMAC link (not SoftBus peer)."""
  return iqlink_hmac_up(params)


def iqlink_home_visible(params) -> bool:
  """Home BT icon: hidden when bridge switch is off."""
  return _bool_param(params, "IqlinkEnabled")


def iqlink_status_color(params) -> rl.Color:
  """Lamp color for settings tile and home icon tint (when visible)."""
  bridge_on = _bool_param(params, "IqlinkEnabled")
  pair_failed = _bool_param(params, "IqlinkBlePairFailed")
  if not bridge_on:
    return STATUS_RED  # settings only; home should hide via iqlink_home_visible
  if iqlink_hmac_up(params):
    return STATUS_GREEN
  # Auto-retry in progress: yellow, not sticky red (pair_failed may still be set briefly).
  if pair_failed:
    return STATUS_YELLOW
  return STATUS_YELLOW

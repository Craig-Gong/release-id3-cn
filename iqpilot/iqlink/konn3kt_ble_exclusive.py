"""IQ-link and Konn3kt BLE settings transport share hci0 — keep them exclusive."""

from __future__ import annotations

from iqpilot.common.swaglog import cloudlog

KONN3KT_BLE_PARAM = "Konn3ktBleTransportEnabled"


def konn3kt_ble_for_iqlink(iqlink_enabled: bool) -> bool:
  """Konn3kt BLE on when IQ-link is off, and vice versa."""
  return not iqlink_enabled


def sync_konn3kt_ble_for_iqlink(params, iqlink_enabled: bool | None = None) -> bool:
  """Write Konn3ktBleTransportEnabled to mirror IqlinkEnabled. Returns True if changed."""
  if iqlink_enabled is None:
    try:
      iqlink_enabled = params.get_bool("IqlinkEnabled")
    except Exception:
      iqlink_enabled = False

  want = konn3kt_ble_for_iqlink(iqlink_enabled)
  try:
    cur = params.get_bool(KONN3KT_BLE_PARAM)
  except Exception:
    cur = True

  if cur == want:
    return False

  try:
    params.put_bool(KONN3KT_BLE_PARAM, want)
    cloudlog.info(
      f"iqlink: {KONN3KT_BLE_PARAM}={int(want)} (IqlinkEnabled={int(iqlink_enabled)})"
    )
    return True
  except Exception as e:
    cloudlog.warning(f"iqlink: failed to sync {KONN3KT_BLE_PARAM}: {e}")
    return False

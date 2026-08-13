#!/usr/bin/env python3
"""IQ-link (iqlink) paired-reconnect A/B helper — device side.

Does NOT reboot by itself. After each manual/scripted reboot, run this to
snapshot whether iqlink is advertising / link state over 60s.

A/B factor (settings BLE transport vs pure iqlink):
  Cell A: Konn3ktBleTransportEnabled=0  (stop competing GATT + BDADDR rewrite)
  Cell B: Konn3ktBleTransportEnabled=1  (default)

Phone side (already paired): open IQ-link, record time until CONNECTED.
Success target: reconnect ≤15s after boot, ≥4/5 reboots.
"""
from __future__ import annotations

import subprocess
import time
from datetime import datetime, timezone


IQLINK_UUID = "73f2c710-5e40-4d0d-8b7f-fde61f729100"
SETTINGS_UUID = "73f2c600-5e40-4d0d-8b7f-fde61f729100"


def _run(cmd: list[str], timeout: float = 8.0) -> str:
  try:
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=False)
    return (r.stdout or "") + (r.stderr or "")
  except Exception as e:
    return f"<err {e}>"


def _params() -> dict[str, str]:
  out: dict[str, str] = {}
  try:
    from openpilot.common.params import Params
    p = Params()
    for k in (
      "IqlinkEnabled", "Konn3ktBleTransportEnabled",
      "IqlinkBleLinkState", "IqlinkBleConnected", "IqlinkBlePeerConnected",
      "IqlinkBleDiscovering",
    ):
      try:
        if k.endswith("Enabled") or k.endswith("Connected") or k.endswith("Discovering"):
          out[k] = str(p.get_bool(k))
        else:
          raw = p.get(k)
          out[k] = raw.decode() if isinstance(raw, bytes) else str(raw)
      except Exception as e:
        out[k] = f"<err {e}>"
  except Exception as e:
    out["Params"] = f"<err {e}>"
  return out


def snapshot(label: str) -> None:
  print(f"=== {label} @ {datetime.now(timezone.utc).isoformat()} ===")
  print("ble-transportd:", _run(["systemctl", "is-active", "ble-transportd"]).strip())
  print("bluetooth:", _run(["systemctl", "is-active", "bluetooth"]).strip())
  # iqlinkd is manager child — detect via pgrep
  pg = _run(["pgrep", "-af", "iqpilot.iqlink.bridge"]).strip()
  print("iqlinkd:", "running" if pg and "pgrep" not in pg else "not_found")
  if pg and "pgrep" not in pg:
    print(" ", pg.splitlines()[0][:160])
  show = _run(["bluetoothctl", "show"])
  for line in show.splitlines():
    if any(k in line for k in ("Address", "Powered", "Discoverable", "Pairable", "UUID: Vendor", "ActiveInstances")):
      print(" ", line.strip())
  print(f"  adapter_lists_iqlink_uuid={IQLINK_UUID in show}")
  print(f"  adapter_lists_settings_uuid={SETTINGS_UUID in show}")
  for k, v in _params().items():
    print(f"{k}={v}")
  print()


def main() -> None:
  print("IQ-link paired-reconnect A/B (device snapshots)")
  print("Cell A: Konn3ktBleTransportEnabled=0  → reboot → run this + open IQ-link")
  print("Cell B: Konn3ktBleTransportEnabled=1  → reboot → run this + open IQ-link")
  print("Record phone time-to-CONNECTED each reboot.\n")
  snapshot("T+0s")
  time.sleep(30)
  snapshot("T+30s")
  time.sleep(30)
  snapshot("T+60s")
  print("Done.")


if __name__ == "__main__":
  main()

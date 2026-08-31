#!/usr/bin/env python3
"""Offline / on-device smoke checks for iqlink (BLE-only).

CLI helper (not imported by runtime).
"""

from __future__ import annotations

import argparse
import sys


def offline() -> int:
  from iqpilot.iqlink.protocol import map_carrot_to_nav_fields
  from iqpilot.iqlink.ble_gatt import (
    ADV_WINDOW_S,
    LINK_CONNECTED,
    LINK_CONNECTING,
    LINK_OFF,
    compute_hmac_hex,
  )

  f = map_carrot_to_nav_fields({
    "nRoadLimitSpeed": 60,
    "nTBTTurnType": 3,
    "nTBTDist": 600,
  })
  assert f and f["shouldSendLaneChangeDesire"]
  assert ADV_WINDOW_S == 120.0
  assert (LINK_OFF, LINK_CONNECTING, LINK_CONNECTED) == (0, 1, 2)
  mac = compute_hmac_hex("123456", 1, 1_720_000_000_000, {"nRoadLimitSpeed": 60})
  assert len(mac) == 32
  print("offline protocol + ble constants OK")
  return 0


def device() -> int:
  from iqpilot.common.params import Params

  p = Params()
  print(f"IqlinkEnabled={p.get_bool('IqlinkEnabled')}")
  print(f"IqlinkExclusive={p.get_bool('IqlinkExclusive')}")
  try:
    raw = p.get("IqlinkBleLinkState")
    ls = raw.decode() if isinstance(raw, bytes) else (str(raw) if raw is not None else "")
    print(f"IqlinkBleLinkState={ls}")
    print(f"IqlinkBleConnected={p.get_bool('IqlinkBleConnected')}")
    print(f"IqlinkBlePeerConnected={p.get_bool('IqlinkBlePeerConnected')}")
    print(f"IqlinkBlePairFailed={p.get_bool('IqlinkBlePairFailed')}")
    print(f"IqlinkBleDiscovering={p.get_bool('IqlinkBleDiscovering')}")
    psk = p.get("IqlinkBlePsk")
    psk_s = psk.decode() if isinstance(psk, bytes) else (str(psk or ""))
    print(f"IqlinkBlePsk=**{psk_s[-2:] if len(psk_s) >= 2 else psk_s}")
  except Exception as e:
    print(f"BLE params (rebuild params_pyx if UnknownKeyName): {e}")
  print("device smoke done (no 770x ports)")
  return 0


def main() -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument("--offline", action="store_true")
  ap.add_argument("--device", action="store_true")
  args = ap.parse_args()
  if args.device:
    return device()
  return offline()


if __name__ == "__main__":
  sys.exit(main())

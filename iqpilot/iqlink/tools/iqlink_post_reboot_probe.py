#!/usr/bin/env python3
"""Post-reboot IQ-link connect probe — run after each reboot.

Polls IqlinkBleConnected / link state for up to --timeout-s seconds.
Exit 0 if connected within window, else 1.
"""
from __future__ import annotations

import argparse
import time


def main() -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument("--timeout-s", type=float, default=60.0)
  ap.add_argument("--cell", default="?")
  args = ap.parse_args()

  from iqpilot.common.params import Params
  p = Params()
  t0 = time.monotonic()
  print(
    f"cell={args.cell} IqlinkEnabled={p.get_bool('IqlinkEnabled')} "
    f"Konn3ktBleTransportEnabled={p.get_bool('Konn3ktBleTransportEnabled')}"
  )
  while time.monotonic() - t0 < args.timeout_s:
    conn = p.get_bool("IqlinkBleConnected")
    peer = p.get_bool("IqlinkBlePeerConnected")
    link = p.get("IqlinkBleLinkState")
    if isinstance(link, bytes):
      link = link.decode()
    elapsed = time.monotonic() - t0
    print(f"t+{elapsed:5.1f}s link={link} connected={conn} peer={peer}")
    if conn or peer or str(link) in ("2", "connected"):
      print(f"SUCCESS cell={args.cell} connect_s={elapsed:.1f}")
      return 0
    time.sleep(1.0)
  print(f"FAIL cell={args.cell} no connect within {args.timeout_s:.0f}s")
  return 1


if __name__ == "__main__":
  raise SystemExit(main())

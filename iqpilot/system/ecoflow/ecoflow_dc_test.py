#!/usr/bin/env python3
"""Manual EcoFlow 12V DC test on C3XL / Mac (does not need ignition).

  cd /data/openpilot
  export PYTHONPATH=/data/openpilot
  # Prefer Params (set once); env also works:
  #   python3 -c "from openpilot.common.params import Params; p=Params();
  #     p.put('EcoflowPhone','155...'); p.put('EcoflowPassword','...');
  #     p.put('EcoflowSn','P231...'); p.put_bool('EcoflowEnabled', True)"

  python3 -m openpilot.iqpilot.system.ecoflow.ecoflow_dc_test status
  python3 -m openpilot.iqpilot.system.ecoflow.ecoflow_dc_test on
  python3 -m openpilot.iqpilot.system.ecoflow.ecoflow_dc_test off
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

from openpilot.iqpilot.system.ecoflow.client import (
  DC_MODE_CFG,
  DC_MODE_MPPT_CAR,
  DC_MODE_PROTO,
  EcoflowError,
  EcoflowSession,
)


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description="EcoFlow Delta 3 DC outlet test")
  parser.add_argument("action", choices=("status", "on", "off", "cycle"))
  parser.add_argument(
    "--mode",
    default=os.environ.get("ECOFLOW_DC_MODE", ""),
    choices=("", DC_MODE_PROTO, DC_MODE_MPPT_CAR, DC_MODE_CFG),
  )
  parser.add_argument("--wait", type=float, default=10.0)
  parser.add_argument("--from-params", action="store_true", default=True,
                      help="Load credentials from Params then env (default)")
  parser.add_argument("--from-env", action="store_true",
                      help="Load credentials from ECOFLOW_* env only")
  args = parser.parse_args(argv)

  session = None
  try:
    if args.from_env:
      session = EcoflowSession.from_env()
    else:
      try:
        session = EcoflowSession.from_params()
      except EcoflowError:
        session = EcoflowSession.from_env()
    mode = args.mode or EcoflowSession.default_dc_mode_for_sn(session.sn)
    kind = "phone" if session.use_phone else "email"
    print(f"login ({kind}) {session.account} sn={session.sn} api={session.api_base} mode={mode}")
    session.login()
    print(f"mqtt {session.mqtt_url}:{session.mqtt_port}")
    session.connect_mqtt()
    before = session.refresh_quotas(wait_s=min(args.wait, 6.0))
    hint = session.dc_state_hint()
    print(
      "telemetry (subset):",
      json.dumps(hint or {"sample_keys": list(before)[:12]}, ensure_ascii=False),
    )

    if args.action == "status":
      print("ok: status only")
      return 0

    if args.action == "cycle":
      for label, want in (("on", True), ("off", False)):
        print(f"set DC {label}...")
        ack = session.set_dc12v(want, mode=mode, wait_s=args.wait)
        print("set_reply:", json.dumps(ack, ensure_ascii=False) if ack else "TIMEOUT")
        time.sleep(1.0)
        print("after:", json.dumps(session.dc_state_hint(), ensure_ascii=False))
      print("ok: cycle done")
      return 0

    want_on = args.action == "on"
    print(f"set DC {'on' if want_on else 'off'}...")
    ack = session.set_dc12v(want_on, mode=mode, wait_s=args.wait)
    print("set_reply:", json.dumps(ack, ensure_ascii=False) if ack else "TIMEOUT")
    time.sleep(1.0)
    print("after:", json.dumps(session.dc_state_hint(), ensure_ascii=False))
    print("ok")
    return 0
  except EcoflowError as e:
    print(f"error: {e}", file=sys.stderr)
    return 1
  finally:
    if session is not None:
      try:
        session.disconnect()
      except Exception:
        pass


if __name__ == "__main__":
  raise SystemExit(main())

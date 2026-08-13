#!/usr/bin/env python3
"""Grab iqNavState samples; print only when speed/accel/dest changes."""
import json
import sys
import time

import cereal.messaging as messaging
from openpilot.common.params import Params

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
LABEL = sys.argv[2] if len(sys.argv) > 2 else "snap"

sm = messaging.SubMaster(["iqNavState"])
p = Params()
deadline = time.time() + DURATION
last = None
best_red = None
best_green = None
while time.time() < deadline:
  sm.update(200)
  if not sm.recv_frame.get("iqNavState", 0):
    continue
  n = sm["iqNavState"]
  dest = getattr(n, "destinationName", "") or ""
  row = {
    "active": bool(n.active),
    "speedTarget": round(float(n.speedTarget or 0), 4),
    "accelTarget": round(float(n.accelTarget or 0), 4),
    "dest": dest,
  }
  key = (row["speedTarget"], row["accelTarget"], row["dest"], row["active"])
  if key != last:
    print(LABEL, "change", json.dumps(row, ensure_ascii=False), flush=True)
    last = key
  if row["accelTarget"] <= -1.5 or "red" in dest.lower():
    best_red = row
  if ("green" in dest.lower() or row["accelTarget"] == 0) and row["speedTarget"] > 15:
    best_green = row

print(LABEL, "IqlinkBleLinkState", p.get("IqlinkBleLinkState"), flush=True)
print(LABEL, "best_red", json.dumps(best_red, ensure_ascii=False), flush=True)
print(LABEL, "best_green", json.dumps(best_green, ensure_ascii=False), flush=True)

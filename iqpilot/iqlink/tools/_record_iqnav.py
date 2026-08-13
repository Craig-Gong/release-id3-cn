#!/usr/bin/env python3
"""Record iqNavState for N seconds → jsonl. Usage: _record_iqnav.py OUT.jsonl [seconds]"""
from __future__ import annotations

import json
import sys
import time

import cereal.messaging as messaging

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/iqlink_nav_run.jsonl"
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 300.0


def jsafe(v):
  if v is None:
    return None
  if isinstance(v, (bool, int, float, str)):
    return v
  try:
    return str(v)
  except Exception:
    return repr(v)


sm = messaging.SubMaster(["iqNavState"])
t0 = time.time()
deadline = t0 + SECS
n_total = 0
n_fat = 0
last_key = None


def is_fat(n) -> bool:
  dest_ok = bool(getattr(n, "destinationValid", False)) or (
    float(getattr(n, "distanceRemaining", 0) or 0) > 50
  )
  man_ok = bool(getattr(n, "nextManeuverValid", False)) or (
    float(getattr(n, "nextManeuverDistance", 0) or 0) > 0
    and bool(jsafe(getattr(n, "nextManeuverDescription", "")) or "")
  )
  spd = float(getattr(n, "targetSpeed", 0) or 0)
  spd_kph = spd * 3.6
  spd_ok = spd_kph >= 35 or (bool(getattr(n, "active", False)) and spd_kph > 0)
  return bool(getattr(n, "active", False)) and dest_ok and (man_ok or spd_ok)


with open(OUT, "w", encoding="utf-8") as f:
  while time.time() < deadline:
    sm.update(200)
    if not sm.recv_frame.get("iqNavState", 0):
      continue
    n = sm["iqNavState"]
    n_total += 1
    fat = is_fat(n)
    if fat:
      n_fat += 1
    row = {
      "t": round(time.time() - t0, 2),
      "fat": fat,
      "active": bool(getattr(n, "active", False)),
      "destinationValid": bool(getattr(n, "destinationValid", False)),
      "nextManeuverValid": bool(getattr(n, "nextManeuverValid", False)),
      "dest": jsafe(getattr(n, "destinationName", "") or ""),
      "remainDist": round(float(getattr(n, "distanceRemaining", 0) or 0), 1),
      "maneuverDist": round(float(getattr(n, "nextManeuverDistance", 0) or 0), 1),
      "maneuverDesc": jsafe(getattr(n, "nextManeuverDescription", "") or ""),
      "targetSpeed_ms": round(float(getattr(n, "targetSpeed", 0) or 0), 4),
      "speedTarget_ms": round(float(getattr(n, "speedTarget", 0) or 0), 4),
      "accelTarget": round(float(getattr(n, "accelTarget", 0) or 0), 4),
      "cameraType": jsafe(getattr(n, "cameraType", None)),
      "trafficLight": jsafe(getattr(n, "trafficLight", None)),
    }
    key = (
      row["active"],
      row["dest"],
      row["remainDist"],
      row["maneuverDist"],
      row["maneuverDesc"],
      row["speedTarget_ms"],
      row["trafficLight"],
    )
    f.write(json.dumps(row, ensure_ascii=False) + "\n")
    f.flush()
    if key != last_key:
      last_key = key
      print(
        f"t={row['t']:.0f}s fat={fat} remain={row['remainDist']} "
        f"man={row['maneuverDist']}/{str(row['maneuverDesc'])[:20]} "
        f"spd={row['speedTarget_ms']:.2f} light={row['trafficLight']}",
        flush=True,
      )

ratio = (n_fat / n_total) if n_total else 0.0
summary = {
  "out": OUT,
  "secs": SECS,
  "n_total": n_total,
  "n_fat": n_fat,
  "fat_ratio": round(ratio, 3),
}
print("SUMMARY " + json.dumps(summary), flush=True)
with open(OUT + ".summary.json", "w", encoding="utf-8") as sf:
  json.dump(summary, sf, ensure_ascii=False, indent=2)

#!/usr/bin/env python3
import time
import cereal.messaging as messaging
sm = messaging.SubMaster(["iqNavState"])
deadline = time.time() + 10
last = None
while time.time() < deadline:
  sm.update(200)
  if not sm.recv_frame.get("iqNavState", 0):
    continue
  n = sm["iqNavState"]
  row = {
    "active": bool(n.active),
    "speedTarget": round(float(n.speedTarget or 0), 4),
    "accelTarget": round(float(n.accelTarget or 0), 4),
    "dest": n.destinationName,
    "maneuverDist": round(float(n.nextManeuverDistance or 0), 1),
    "maneuverDesc": n.nextManeuverDescription,
    "targetSpeed": round(float(n.targetSpeed or 0), 4),
    "remainDist": round(float(n.distanceRemaining or 0), 1),
  }
  key = tuple(row.values())
  if key != last:
    print(row, flush=True)
    last = key
print("snap_done", flush=True)

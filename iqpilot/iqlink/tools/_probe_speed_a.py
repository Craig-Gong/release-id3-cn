#!/usr/bin/env python3
"""One-shot device probe for speed-limit A (mapd HUD path)."""
from openpilot.common.params import Params
import cereal.messaging as messaging

p = Params()
keys = [
  "IQSpeedAssistMode", "IsOnroad", "IsOffroad", "MapdVersion",
  "OsmLocationName", "ShowSpeedLimits", "SpeedLimitController",
]
print("=== params ===")
for k in keys:
  try:
    print(k, p.get(k))
  except Exception as e:
    print(k, e)

print("=== cereal (3s) ===")
sm = messaging.SubMaster(["mapdOut", "iqLiveData", "deviceState", "iqPlan"])
for _ in range(15):
  sm.update(200)

print("deviceState.started", sm["deviceState"].started)
print("mapdOut_frame", sm.recv_frame.get("mapdOut", 0))
if sm.recv_frame.get("mapdOut", 0):
  m = sm["mapdOut"]
  print("mapdOut.speedLimit", m.speedLimit, "next", m.nextSpeedLimit, "dist", m.nextSpeedLimitDistance)
print("iqLiveData_frame", sm.recv_frame.get("iqLiveData", 0))
if sm.recv_frame.get("iqLiveData", 0):
  d = sm["iqLiveData"]
  for attr in ("speedLimit", "speedLimitAhead", "speedLimitAheadDistance", "roadName"):
    print(f"iqLiveData.{attr}", getattr(d, attr, None))
print("iqPlan_frame", sm.recv_frame.get("iqPlan", 0))
if sm.recv_frame.get("iqPlan", 0):
  plan = sm["iqPlan"]
  sl = getattr(plan, "speedLimit", None)
  if sl is not None:
    print("iqPlan.speedLimit", sl)
print("probe_done")

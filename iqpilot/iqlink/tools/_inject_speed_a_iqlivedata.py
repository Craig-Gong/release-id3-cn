#!/usr/bin/env python3
"""Offroad fake-car inject: publish iqLiveData speedLimit and confirm SubMaster sees it.

mapd/mapd_live_bridge only run onroad — this validates the HUD data plane without ignition.
"""
import time

import cereal.messaging as messaging
from openpilot.common.params import Params

LIMIT_MPS = 60.0 / 3.6  # 60 km/h
AHEAD_MPS = 40.0 / 3.6
AHEAD_DIST = 150.0


def main() -> None:
  p = Params()
  mode = p.get("IQSpeedAssistMode")
  print("IQSpeedAssistMode", mode)
  assert mode is not None and int(mode) >= 1, "IQSpeedAssistMode must be >=1 for HUD"

  pm = messaging.PubMaster(["iqLiveData"])
  sm = messaging.SubMaster(["iqLiveData"])
  time.sleep(0.3)

  for i in range(8):
    msg = messaging.new_message("iqLiveData")
    d = msg.iqLiveData
    d.speedLimitValid = True
    d.speedLimit = LIMIT_MPS
    d.speedLimitAheadValid = True
    d.speedLimitAhead = AHEAD_MPS
    d.speedLimitAheadDistance = AHEAD_DIST
    d.roadName = "fake-car-limit-probe"
    pm.send("iqLiveData", msg)
    sm.update(100)
    time.sleep(0.15)

  assert sm.recv_frame.get("iqLiveData", 0) > 0, "no iqLiveData received"
  got = sm["iqLiveData"]
  print(
    "got speedLimit", got.speedLimit,
    "ahead", got.speedLimitAhead,
    "dist", got.speedLimitAheadDistance,
    "road", got.roadName,
  )
  assert abs(got.speedLimit - LIMIT_MPS) < 0.05
  assert abs(got.speedLimitAhead - AHEAD_MPS) < 0.05
  assert abs(got.speedLimitAheadDistance - AHEAD_DIST) < 0.5
  print("speed_a_inject_ok")


if __name__ == "__main__":
  main()

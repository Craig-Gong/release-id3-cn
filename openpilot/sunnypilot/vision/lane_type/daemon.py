#!/usr/bin/env python3
"""Onroad VisionIPC → lane.onnx → /dev/shm/sp_lane_type.json.

CPU 0–3, OpenCV ≤2 threads, ~400 ms interval. No V-ASM / no cereal.
"""
from __future__ import annotations

import os
import sys
import time

# Prefer writable pydeps (opencv) ahead of system site-packages.
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
_PYDEPS = os.path.join(_ROOT, "pydeps")
if os.path.isdir(_PYDEPS) and _PYDEPS not in sys.path:
  sys.path.insert(0, _PYDEPS)

from openpilot.common.file_params import read_file_param
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import Ratekeeper, set_core_affinity
from openpilot.common.swaglog import cloudlog
from openpilot.cereal.visionipc import VisionStreamType
from openpilot.sunnypilot.vision.lane_type.inference import (
  CONFIDENCE_THRESHOLD, DEFAULT_LANE_MODEL_PATH, LaneInference,
)
from openpilot.sunnypilot.vision.lane_type.nv12 import nv12_y_plane
from openpilot.sunnypilot.vision.lane_type.snapshot import (
  LINE_UNKNOWN, LaneTypeSnapshot, write_lane_type,
)

LANE_INTERVAL_S = 0.40
CAMERA_POLL_S = 0.005
NO_FRAME_WARN_S = 2.0
PARAM_KEY = "LaneTypeOnnx"


def _enabled(params: Params) -> bool:
  try:
    return bool(params.get_bool(PARAM_KEY))
  except UnknownKeyName:
    return bool(read_file_param(PARAM_KEY, False))
  except Exception:
    return bool(read_file_param(PARAM_KEY, False))


def _idle_loop(err: str) -> None:
  """Stay alive so manager does not thrash-restart; keep publishing disabled/err."""
  while True:
    write_lane_type(LaneTypeSnapshot(err=err, ts=time.monotonic()))
    time.sleep(2.0)


def main() -> None:
  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    pass

  try:
    import cv2
    cv2.setNumThreads(2)
  except Exception as exc:
    cloudlog.error(f"lane_typed: OpenCV unavailable ({exc}); idle")
    _idle_loop("opencv missing")
    return

  params = Params()
  engine = LaneInference(DEFAULT_LANE_MODEL_PATH)
  if not engine.load():
    cloudlog.warning(f"lane_typed: model not loaded ({engine.error}); idle")
    _idle_loop(engine.error or "model missing")
    return

  cloudlog.info(f"lane_typed: loaded {DEFAULT_LANE_MODEL_PATH}")

  from msgq.visionipc import VisionIpcClient

  client = None
  last_infer = 0.0
  last_frame = 0.0
  rk = Ratekeeper(20, print_delay_threshold=None)
  stream = VisionStreamType.VISION_STREAM_NARROW_ROAD

  while True:
    if not _enabled(params):
      write_lane_type(LaneTypeSnapshot(err="disabled"))
      time.sleep(0.5)
      continue

    try:
      if client is None or not client.is_connected():
        available = VisionIpcClient.available_streams("camerad", block=False)
        if available and VisionStreamType.VISION_STREAM_NARROW_ROAD not in available:
          stream = VisionStreamType.VISION_STREAM_WIDE_ROAD
        client = VisionIpcClient("camerad", stream, True)
        if not client.connect(False):
          write_lane_type(LaneTypeSnapshot(err="camera unavailable", ts=time.monotonic()))
          time.sleep(1.0)
          rk.keep_time()
          continue

      buf = client.recv(timeout_ms=0)
      now = time.monotonic()
      if buf is None:
        if last_frame > 0.0 and (now - last_frame) > NO_FRAME_WARN_S:
          write_lane_type(LaneTypeSnapshot(err="no frames", ts=now))
        time.sleep(CAMERA_POLL_S)
        rk.keep_time()
        continue

      last_frame = now
      if now - last_infer < LANE_INTERVAL_S:
        rk.keep_time()
        continue

      frame = nv12_y_plane(buf.data, buf.width, buf.height, buf.stride)
      res = engine.infer(frame, buf.width, buf.height, conf_thresh=CONFIDENCE_THRESHOLD)
      last_infer = now
      snap = LaneTypeSnapshot(
        ts=now,
        left=int(res.get("leftLine", LINE_UNKNOWN)),
        right=int(res.get("rightLine", LINE_UNKNOWN)),
        left_conf=float(res.get("leftConf", 0.0) or 0.0),
        right_conf=float(res.get("rightConf", 0.0) or 0.0),
        valid=bool(res.get("valid")),
        err=str(res.get("error") or ""),
      )
      write_lane_type(snap)
    except Exception as exc:
      cloudlog.warning(f"lane_typed: {exc}")
      client = None
      write_lane_type(LaneTypeSnapshot(err=str(exc), ts=time.monotonic()))
      time.sleep(1.0)
    rk.keep_time()


if __name__ == "__main__":
  main()

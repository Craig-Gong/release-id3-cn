"""C3XL one-shot calibration reset (no OnroadCycle).

CalibrationParams is written infrequently, so watching "param is missing"
re-resets every completed block (~20%). Each daemon consumes its own flag.
"""
from __future__ import annotations

import os

_RESET_PREFIX = "/dev/shm/c3xl_calib_reset"
_DAEMONS = ("calibrationd", "lagd", "paramsd", "torqued")


def request_c3xl_calib_reset() -> None:
  for name in _DAEMONS:
    path = f"{_RESET_PREFIX}.{name}"
    with open(path, "w") as f:
      f.write("1")
      f.flush()
      os.fsync(f.fileno())


def consume_c3xl_calib_reset(name: str) -> bool:
  path = f"{_RESET_PREFIX}.{name}"
  try:
    os.unlink(path)
    return True
  except FileNotFoundError:
    return False

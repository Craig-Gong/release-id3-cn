"""Params that the on-device libparams.so does not know yet.

Manager forks children and they keep the Params class loaded at manager start.
A rsync of params.py is invisible until manager itself restarts. UI and planner
share this directory so a new slider still works.
"""
from __future__ import annotations

import os

FILE_PARAM_DIR = "/data/openpilot_extra_params"


def read_file_param(key: str, default=None):
  path = os.path.join(FILE_PARAM_DIR, key)
  try:
    with open(path) as f:
      raw = f.read().strip()
  except OSError:
    return default
  if raw == "":
    return default
  if isinstance(default, bool):
    return raw in ("1", "True", "true")
  if isinstance(default, int) and not isinstance(default, bool):
    try:
      return int(raw)
    except ValueError:
      return default
  try:
    return float(raw)
  except ValueError:
    return default


def write_file_param(key: str, value) -> None:
  os.makedirs(FILE_PARAM_DIR, exist_ok=True)
  path = os.path.join(FILE_PARAM_DIR, key)
  tmp = path + ".tmp"
  with open(tmp, "w") as f:
    f.write(str(value))
  os.replace(tmp, path)

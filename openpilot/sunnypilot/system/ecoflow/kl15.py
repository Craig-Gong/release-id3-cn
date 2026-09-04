"""MEB KL15 and chestnut SuperSpeed probe. No cereal / MQTT."""
from __future__ import annotations

import os

MEB_KLEMMEN_ADDR = 0x3C0
MEB_IGNITION_HOLD_S = 2.0
_DOCK_VID = "3801"
_DOCK_PID = "0001"


def meb_ignition_from_can(packets, now: float, last_on_ts: float | None):
  saw = False
  for msg in packets:
    try:
      cans = msg.can
    except Exception:
      continue
    for c in cans:
      if getattr(c, "src", 0) >= 128:
        continue
      if c.address != MEB_KLEMMEN_ADDR:
        continue
      saw = True
      dat = bytes(c.dat)
      if len(dat) >= 3 and (dat[2] & 0x02):
        last_on_ts = now
  if last_on_ts is not None and (now - last_on_ts) < MEB_IGNITION_HOLD_S:
    return True, last_on_ts, saw
  return False, last_on_ts, saw


def chestnut_superspeed_present() -> bool:
  base = "/sys/bus/usb/devices"
  try:
    names = os.listdir(base)
  except OSError:
    return False
  for name in names:
    d = os.path.join(base, name)
    try:
      with open(os.path.join(d, "idVendor"), encoding="utf-8") as f:
        vid = f.read().strip().lower()
      with open(os.path.join(d, "idProduct"), encoding="utf-8") as f:
        pid = f.read().strip().lower()
    except OSError:
      continue
    if vid != _DOCK_VID or pid != _DOCK_PID:
      continue
    speed = ""
    try:
      with open(os.path.join(d, "speed"), encoding="utf-8") as f:
        speed = f.read().strip()
    except OSError:
      pass
    if speed in ("5000", "10000"):
      return True
  return False

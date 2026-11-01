"""MEB KL15 and chestnut SuperSpeed probe. No cereal / MQTT."""
from __future__ import annotations

import os
from typing import Any, Iterable

MEB_KLEMMEN_ADDR = 0x3C0
# Hold past brief SubMaster/poll gaps; 0x3C0 is ~10 Hz but conflated reads can miss.
MEB_IGNITION_HOLD_S = 5.0
_DOCK_VID = "3801"
_DOCK_PID = "0001"


def _as_can_frames(packets: Any) -> list[Any]:
  """Normalize SubMaster['can'] (frame list) or Event / Event-list (.can)."""
  if packets is None:
    return []

  # Single cereal Event with .can
  if hasattr(packets, "can") and not hasattr(packets, "address"):
    try:
      return list(packets.can)
    except Exception:
      return []

  try:
    seq = list(packets)
  except TypeError:
    if hasattr(packets, "address"):
      return [packets]
    return []

  if not seq:
    return []

  # List of Events (drain_sock style)
  first = seq[0]
  if hasattr(first, "can") and not hasattr(first, "address"):
    out: list[Any] = []
    for msg in seq:
      try:
        out.extend(msg.can)
      except Exception:
        continue
    return out

  # SubMaster["can"]: already a list of CanData
  return seq


def meb_ignition_from_can(packets, now: float, last_on_ts: float | None):
  saw = False
  for c in _as_can_frames(packets):
    try:
      src = int(getattr(c, "src", 0))
      addr = int(c.address)
    except Exception:
      continue
    if src >= 128:
      continue
    if addr != MEB_KLEMMEN_ADDR:
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

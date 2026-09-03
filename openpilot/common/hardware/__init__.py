import glob
import os
from typing import cast

from openpilot.common.hardware.base import HardwareBase
from openpilot.common.hardware.comma.hardware import HardwareComma
from openpilot.common.hardware.pc.hardware import HardwarePc

AGNOS = os.path.isfile('/AGNOS')
COMMA_HARDWARE = AGNOS
PC = not COMMA_HARDWARE


if COMMA_HARDWARE:
  HARDWARE = cast(HardwareBase, HardwareComma())
else:
  HARDWARE = cast(HardwareBase, HardwarePc())


def has_cabin_camera() -> bool:
  """Road+wide only (C3XL) vs comma 3/3X with a cabin camera."""
  if os.getenv("DISABLE_DRIVER"):
    return False
  if os.getenv("USE_WEBCAM"):
    return True
  try:
    with open("/data/hardware_profile", encoding="utf-8") as f:
      if f.read().strip() == "c3xl":
        return False
  except OSError:
    pass
  try:
    count = 0
    for name_path in glob.glob("/sys/class/video4linux/v4l-subdev*/name"):
      with open(name_path, encoding="utf-8") as f:
        if f.read().strip() == "cam-sensor-driver":
          count += 1
    if count == 0:
      return True
    return count >= 3
  except OSError:
    return True

from enum import StrEnum
import glob
import os
from pathlib import Path

# Hardware capabilities belong to the physical device, not to a Git branch.
# This tree is for one C3XL: no cabin camera auto-selects the C3XL profile.
# Override with SUNNYPILOT_HARDWARE_PROFILE or /data/hardware_profile.
HARDWARE_PROFILE_FILE = Path(os.getenv("SUNNYPILOT_HARDWARE_PROFILE_FILE", "/data/hardware_profile"))


class HardwareProfile(StrEnum):
  STANDARD = "standard"
  C3XL = "c3xl"


def _no_cabin_camera() -> bool:
  if os.getenv("DISABLE_DRIVER"):
    return True
  if os.getenv("USE_WEBCAM"):
    return False
  try:
    count = 0
    for name_path in glob.glob("/sys/class/video4linux/v4l-subdev*/name"):
      with open(name_path, encoding="utf-8") as f:
        if f.read().strip() == "cam-sensor-driver":
          count += 1
    if count == 0:
      return False
    return count < 3
  except OSError:
    return False


def get_hardware_profile(value: str | None = None) -> HardwareProfile:
  if value is not None:
    raw_value = value
  elif env_value := os.getenv("SUNNYPILOT_HARDWARE_PROFILE"):
    raw_value = env_value
  elif HARDWARE_PROFILE_FILE.is_file():
    raw_value = HARDWARE_PROFILE_FILE.read_text().strip()
  elif os.path.isfile("/AGNOS") and _no_cabin_camera():
    return HardwareProfile.C3XL
  else:
    raw_value = HardwareProfile.STANDARD
  try:
    return HardwareProfile(raw_value)
  except ValueError:
    return HardwareProfile.STANDARD


def has_driver_camera(profile: HardwareProfile | None = None) -> bool:
  return (profile or get_hardware_profile()) != HardwareProfile.C3XL


def has_amplifier(profile: HardwareProfile | None = None) -> bool:
  return (profile or get_hardware_profile()) != HardwareProfile.C3XL


def allows_automatic_power_down(profile: HardwareProfile | None = None) -> bool:
  return (profile or get_hardware_profile()) != HardwareProfile.C3XL


def power_down_requested(*, automatic: bool, manual: bool,
                         profile: HardwareProfile | None = None) -> bool:
  return manual or (automatic and allows_automatic_power_down(profile))

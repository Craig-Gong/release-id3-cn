from enum import StrEnum
import os
from collections.abc import MutableMapping
from pathlib import Path


# Hardware capabilities belong to the physical device, not to a Git branch.
# The persistent file is authoritative. A raw `comma tici` model without that
# file is the C3XL recovery/install case; other hardware keeps upstream defaults.
HARDWARE_PROFILE_FILE = Path(os.getenv("SUNNYPILOT_HARDWARE_PROFILE_FILE", "/data/hardware_profile"))
HARDWARE_MODEL_FILE = Path(os.getenv("SUNNYPILOT_HARDWARE_MODEL_FILE", "/sys/firmware/devicetree/base/model"))

# C3XL OX03C10 road is 1928×1208; CTM/Chestnut catalogs target C4/mici 1344×760.
# IFE hardware resize closes that gap when opted in (docs/C3XL_IFE_HARDWARE_RESIZE.md).
C3XL_IFE_ENV = "C3XL_IFE_ROAD_SIZE"
C3XL_IFE_ROAD_SIZE = "1344x760"

class HardwareProfile(StrEnum):
  STANDARD = "standard"
  C3XL = "c3xl"


PANDA_TYPE_UNKNOWN = b"\x00"
PANDA_TYPE_TRES = b"\x09"


def infer_hardware_profile(model_file: Path | None = None) -> HardwareProfile:
  try:
    raw_model = (model_file or HARDWARE_MODEL_FILE).read_bytes().rstrip(b"\x00\r\n ")
  except OSError:
    return HardwareProfile.STANDARD
  return HardwareProfile.C3XL if raw_model == b"comma tici" else HardwareProfile.STANDARD


def get_hardware_profile(value: str | None = None) -> HardwareProfile:
  if value is not None:
    raw_value = value
  elif env_value := os.getenv("SUNNYPILOT_HARDWARE_PROFILE"):
    raw_value = env_value
  elif HARDWARE_PROFILE_FILE.is_file():
    raw_value = HARDWARE_PROFILE_FILE.read_text().strip()
  else:
    raw_value = infer_hardware_profile()
  return HardwareProfile(raw_value)


def persist_hardware_profile(profile: HardwareProfile | None = None) -> HardwareProfile:
  """Write /data/hardware_profile so native camerad IFE gates match Python inference."""
  selected = profile or get_hardware_profile()
  if selected != HardwareProfile.C3XL:
    return selected
  try:
    current = HARDWARE_PROFILE_FILE.read_text().strip() if HARDWARE_PROFILE_FILE.is_file() else ""
    if current != HardwareProfile.C3XL.value:
      HARDWARE_PROFILE_FILE.parent.mkdir(parents=True, exist_ok=True)
      HARDWARE_PROFILE_FILE.write_text(HardwareProfile.C3XL.value)
  except OSError:
    pass
  return selected


def c3xl_ife_road_requested(environment: MutableMapping[str, str] | None = None) -> bool:
  env = os.environ if environment is None else environment
  return env.get(C3XL_IFE_ENV) == C3XL_IFE_ROAD_SIZE


def c3xl_ife_profile_file_ready() -> bool:
  try:
    return HARDWARE_PROFILE_FILE.is_file() and HARDWARE_PROFILE_FILE.read_text().strip() == HardwareProfile.C3XL.value
  except OSError:
    return False


def apply_c3xl_ife_runtime(environment: MutableMapping[str, str] | None = None,
                           *, profile: HardwareProfile | None = None) -> bool:
  """C3XL IFE 1344×760 for CTM/C4 parity (device configured on).

  Persist /data/hardware_profile for native camerad gates, setdefault the IFE
  env so manager children inherit it, and drop C3XL_CTMV2_INPUT_RESIZE.
  Explicit C3XL_IFE_ROAD_SIZE=off disables.
  """
  env = os.environ if environment is None else environment
  selected = persist_hardware_profile(profile)
  if selected != HardwareProfile.C3XL:
    return False

  env.pop("C3XL_CTMV2_INPUT_RESIZE", None)
  val = env.get(C3XL_IFE_ENV)
  if val is not None and val.strip().lower() in {"", "0", "off", "false", "no"}:
    env.pop(C3XL_IFE_ENV, None)
    return False
  env.setdefault(C3XL_IFE_ENV, C3XL_IFE_ROAD_SIZE)
  return c3xl_ife_road_requested(env) and c3xl_ife_profile_file_ready()


def has_driver_camera(profile: HardwareProfile | None = None) -> bool:
  return (profile or get_hardware_profile()) != HardwareProfile.C3XL


def has_amplifier(profile: HardwareProfile | None = None) -> bool:
  return (profile or get_hardware_profile()) != HardwareProfile.C3XL


def allows_automatic_power_down(profile: HardwareProfile | None = None) -> bool:
  return (profile or get_hardware_profile()) != HardwareProfile.C3XL


def power_down_requested(*, automatic: bool, manual: bool,
                         profile: HardwareProfile | None = None) -> bool:
  return manual or (automatic and allows_automatic_power_down(profile))


def model_compile_cpu(cpu_count: int) -> int:
  """Return the upstream isolated CPU when present, otherwise the highest available CPU."""
  return min(7, max(0, cpu_count - 1))


def resolve_internal_panda_type(raw_type: bytes, profile: HardwareProfile | None = None) -> bytes:
  """Resolve the effective type for an already-identified internal Panda."""
  selected_profile = profile or get_hardware_profile()
  if selected_profile != HardwareProfile.C3XL:
    return raw_type
  if raw_type in (PANDA_TYPE_UNKNOWN, PANDA_TYPE_TRES):
    return PANDA_TYPE_TRES
  raise ValueError(f"C3XL internal SPI Panda reported unexpected raw type {raw_type.hex()!r}")

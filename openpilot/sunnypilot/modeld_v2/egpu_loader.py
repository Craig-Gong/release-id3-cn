import os
import threading
from collections.abc import Callable, MutableMapping
from typing import TypeVar

# Measured on C3XL (onemiless/dev-sp-egpu, 2026-08-29):
# LM 71.50 s, BMV3 73.54 s, TT 74.66 s, BMV2 75.58 s.
# 120 s keeps a margin for cold starts and USB scheduling.
C3XL_MODEL_LOAD_TIMEOUT = 120
C3XL_TINYGRAD_CACHE_HOME = "/data/cache"
# Cap chestnut GPU PPT for car 12V rails.
# Comma chestnut ≈100 W; 9060 XT 8GB TBP 150 W; Delta 3 12V = 126 W. PPT 100
# matches designed Lebowski budget with rail headroom. Raise after AC→12V ≥20 A.
C3XL_AM_POWER_LIMIT_W = "100"

T = TypeVar("T")


class EgpuModelLoadError(RuntimeError):
  pass


def configure_default_device(comma_hardware: bool, environment: MutableMapping[str, str] = os.environ, *, c3xl: bool = False) -> None:
  """Prevent tinygrad's default-device scan from probing the USB AMD GPU."""
  if comma_hardware:
    environment.setdefault("DEV", "QCOM")
  if c3xl:
    # /home is an ephemeral overlay on C3XL. Keep AMD firmware/compiler caches.
    environment.setdefault("XDG_CACHE_HOME", C3XL_TINYGRAD_CACHE_HOME)
    # PPT limit → SMU auto-downclocks under load; set AM_POWER_LIMIT before import to override.
    environment.setdefault("AM_POWER_LIMIT", C3XL_AM_POWER_LIMIT_W)


def load_with_timeout(load: Callable[[], T], timeout: float) -> T:
  result: list[T] = []
  error: list[Exception] = []
  done = threading.Event()

  def run() -> None:
    try:
      result.append(load())
    except Exception as e:
      error.append(e)
    finally:
      done.set()

  threading.Thread(target=run, name="egpu-model-loader", daemon=True).start()
  if not done.wait(timeout):
    raise TimeoutError(f"eGPU model load timed out after {timeout:g}s")
  if error:
    raise EgpuModelLoadError(f"eGPU model load failed: {error[0]}") from error[0]
  return result[0]

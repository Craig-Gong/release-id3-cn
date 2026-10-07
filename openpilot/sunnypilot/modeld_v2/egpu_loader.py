import os
import threading
import time
from collections.abc import Callable, MutableMapping


# Measured on the C3XL/UT3G device on 2026-08-29:
# LM 71.50 s, BMV3 73.54 s, TT 74.66 s, BMV2 75.58 s.
# Keep a bounded 44.42 s margin for cold starts and USB scheduling variance.
C3XL_MODEL_LOAD_TIMEOUT = 120
C3XL_TINYGRAD_CACHE_HOME = "/data/cache"
# C3XL default PPT (2026-09-04 onroad). Rails are EcoFlow 12V ~126 W and/or
# cigarette 10 A; comma chestnut ≈100 W. tinygrad getenv("AM_POWER_LIMIT", 0.0)
# > 0 → SMU SetPptLimit + clocks level=None. Full clocks had a first-frame
# bulk IN 0x81 timeout; after 100 W + offroad/READY, Lebowski loaded in 15.3 s
# with ChestnutActive=1 and no fallback. 100 W is the rail cap, not a USB-link
# fix. Other devices stay unset (max clocks). Export wins.
C3XL_AM_POWER_LIMIT_W = 100
# Official SP/OP Chestnut default (onemiless: tried 100 µs, reverted to 500).
# tinygrad getenv default is already 500; set it explicitly so C3XL does not
# inherit a tighter experimental poll.
C3XL_AMD_USB_POLL_US = 500

# READY can beat the GPU: EcoFlow / KL15 12 V arrives after modeld has started.
# The ASM bridge may enumerate on USB-C 5 V alone, or only once 12 V is up; the
# GPU is usable only when PCIe reaches L0. modeld decides eGPU once at startup,
# so wait for a usable link instead of committing to the small model.
C3XL_DOCK_WAIT_TIMEOUT = 30.0
# With EcoFlow status (2026-10-07 logs): 12 V confirmed 0–7 s after KL15 (slow
# MQTT login on a cold morning), dock enumerated ~5 s after that.
C3XL_DOCK_POWER_PENDING_TIMEOUT = 45.0  # 12 V requested, not yet confirmed
# MQTT logged in ~6 s after the network came up; without it 12 V cannot be switched on.
C3XL_DOCK_OFFLINE_TIMEOUT = 20.0
C3XL_DOCK_POWERED_TIMEOUT = 20.0  # counted from 12 V confirmation
C3XL_DOCK_MAX_TIMEOUT = 60.0
# Present at 5 Gbps but LTSSM unreadable this long: load anyway, as before.
C3XL_DOCK_UNKNOWN_GRACE = 10.0
# Settle after a freshly powered GPU first reaches L0.
C3XL_DOCK_SETTLE = 2.0
DOCK_WAIT_POLL = 0.5

DOCK_READY = "ready"
DOCK_ABSENT = "absent"
DOCK_NOT_READY = "not_ready"  # present, but USB < 5 Gbps or PCIe not in L0
DOCK_UNKNOWN = "unknown"  # present at 5 Gbps, LTSSM unreadable

POWER_ON = "on"  # EcoFlow telemetry confirms the 12 V rail
POWER_PENDING = "pending"  # EcoFlow enabled and live, rail not confirmed yet
POWER_OFFLINE = "offline"  # EcoFlow enabled and live, but no MQTT session to switch the rail
POWER_UNKNOWN = "unknown"  # no EcoFlow, disabled, or ecoflowd not publishing

# Survives modeld restart during the same onroad; /dev/shm clears on reboot.
# Offroad chestnut_statusd unlinks it so the next READY can try eGPU again.
CHESTNUT_SKIP_DRIVE_PATH = "/dev/shm/chestnut_skip_drive"
# A restarted modeld must not hold the model back again mid-drive (it may be
# engaged), so the startup wait happens once per onroad. Cleared offroad.
CHESTNUT_DOCK_DECIDED_PATH = "/dev/shm/chestnut_dock_decided"
# A dock has been used on this device, so a missing one at READY is most likely
# still powering up. Cleared when a wait finds nothing, so an unplugged dock
# costs one wait, not one per drive.
CHESTNUT_DOCK_SEEN_PATH = "/data/chestnut_dock_seen"
# 12 V was confirmed (or unknowable) and still no dock: it is unplugged, so an
# EcoFlow rail alone must not hold READY again until a dock is seen.
CHESTNUT_DOCK_ABSENT_PATH = "/data/chestnut_dock_absent"


def _touch(path: str) -> None:
  fd = os.open(path, os.O_CREAT | os.O_WRONLY | os.O_TRUNC, 0o644)
  try:
    os.write(fd, b"1")
    os.fsync(fd)
  finally:
    os.close(fd)


def _unlink(path: str) -> None:
  try:
    os.unlink(path)
  except FileNotFoundError:
    pass


def chestnut_skip_drive(path: str = CHESTNUT_SKIP_DRIVE_PATH) -> bool:
  return os.path.exists(path)


def set_chestnut_skip_drive(path: str = CHESTNUT_SKIP_DRIVE_PATH) -> None:
  _touch(path)


def clear_chestnut_skip_drive(path: str = CHESTNUT_SKIP_DRIVE_PATH) -> None:
  _unlink(path)


def _best_effort(action: Callable[[str], None], path: str) -> None:
  try:
    action(path)
  except OSError:
    pass


def mark_chestnut_dock_decided(path: str = CHESTNUT_DOCK_DECIDED_PATH) -> None:
  _best_effort(_touch, path)


def clear_chestnut_dock_decided(path: str = CHESTNUT_DOCK_DECIDED_PATH) -> None:
  _best_effort(_unlink, path)


def mark_chestnut_dock_seen(path: str = CHESTNUT_DOCK_SEEN_PATH, *,
                            absent_path: str = CHESTNUT_DOCK_ABSENT_PATH) -> None:
  if not os.path.exists(path):
    _best_effort(_touch, path)
  if os.path.exists(absent_path):
    _best_effort(_unlink, absent_path)


def clear_chestnut_dock_seen(path: str = CHESTNUT_DOCK_SEEN_PATH) -> None:
  _best_effort(_unlink, path)


def mark_chestnut_dock_absent(path: str = CHESTNUT_DOCK_ABSENT_PATH) -> None:
  _best_effort(_touch, path)


def should_wait_for_dock(present: bool, *, power_expected: bool = False,
                         decided_path: str = CHESTNUT_DOCK_DECIDED_PATH,
                         seen_path: str = CHESTNUT_DOCK_SEEN_PATH,
                         absent_path: str = CHESTNUT_DOCK_ABSENT_PATH) -> bool:
  if os.path.exists(decided_path):
    return False
  if present or os.path.exists(seen_path):
    return True
  return power_expected and not os.path.exists(absent_path)


def ecoflow_dock_power(read_status: Callable[[], object] | None = None,
                       monotonic: Callable[[], float] = time.monotonic) -> str:
  """EcoFlow is the only switched 12 V source here; its status says whether to keep waiting."""
  try:
    if read_status is None:
      from openpilot.sunnypilot.system.ecoflow.status import read_status
    status = read_status()
    if not status.enabled or not status.fresh(monotonic()):
      return POWER_UNKNOWN
    if status.dc12v is True:
      return POWER_ON
    return POWER_PENDING if status.mqtt else POWER_OFFLINE
  except Exception:
    return POWER_UNKNOWN


def keep_dock_seen_after_wait(state: str, power: str) -> bool:
  """An absent dock with 12 V not yet up is a power problem, not an unplugged dock."""
  return state != DOCK_ABSENT or power in (POWER_PENDING, POWER_OFFLINE)


class EgpuModelLoadError(RuntimeError):
  pass


def chestnut_dock_link_state() -> str:
  """Passive check only: USB speed from sysfs plus a read-only LTSSM EP0 read."""
  from openpilot.common.hardware.usb import get_usb_state, is_chestnut_runtime_device
  from openpilot.system.hardware.chestnut.status import MIN_USB_SPEED_MBPS, PCIE_L0, read_pcie_ltssm
  speeds = [d["speedMbps"] for d in get_usb_state() if is_chestnut_runtime_device(d)]
  if not speeds:
    return DOCK_ABSENT
  if max(speeds) < MIN_USB_SPEED_MBPS:
    return DOCK_NOT_READY
  try:
    ltssm = read_pcie_ltssm()
  except (OSError, RuntimeError):
    return DOCK_UNKNOWN
  return DOCK_READY if ltssm == PCIE_L0 else DOCK_NOT_READY


def dock_usable(state: str) -> bool:
  return state in (DOCK_READY, DOCK_UNKNOWN)


def wait_for_chestnut_dock(probe: Callable[[], str] = chestnut_dock_link_state, *,
                           power: Callable[[], str] | None = None,
                           timeout: float = C3XL_DOCK_WAIT_TIMEOUT,
                           power_pending_timeout: float = C3XL_DOCK_POWER_PENDING_TIMEOUT,
                           offline_timeout: float = C3XL_DOCK_OFFLINE_TIMEOUT,
                           powered_timeout: float = C3XL_DOCK_POWERED_TIMEOUT,
                           max_timeout: float = C3XL_DOCK_MAX_TIMEOUT,
                           unknown_grace: float = C3XL_DOCK_UNKNOWN_GRACE,
                           settle: float = C3XL_DOCK_SETTLE, poll: float = DOCK_WAIT_POLL,
                           on_change: Callable[[float, str, str], None] | None = None,
                           sleep: Callable[[float], None] = time.sleep,
                           monotonic: Callable[[], float] = time.monotonic) -> str:
  """Poll probe until the GPU link is usable or timeout; return the last state.

  Without a power signal the wait is a flat timeout. With one, wait up to
  power_pending_timeout for 12 V (offline_timeout while EcoFlow has no MQTT
  session yet), then powered_timeout from its confirmation, never beyond
  max_timeout. UNKNOWN (link unreadable) is accepted only after unknown_grace,
  so a readable L0 still gets its chance. on_change(elapsed, dock, rail) fires
  whenever either state changes.
  """
  start = monotonic()
  powered_since = None
  link_seen = False
  unknown_since = None
  waited = False
  last = None
  state = probe()
  while state != DOCK_READY:
    now = monotonic()
    rail = power() if power is not None else POWER_UNKNOWN
    if on_change is not None and (state, rail) != last:
      on_change(now - start, state, rail)
    last = (state, rail)
    if rail == POWER_ON and powered_since is None:
      powered_since = now
    link_seen = link_seen or rail == POWER_PENDING
    if powered_since is not None:
      deadline = min(powered_since + powered_timeout, start + max_timeout)
    elif rail == POWER_PENDING or (rail == POWER_OFFLINE and link_seen):
      deadline = start + power_pending_timeout
    elif rail == POWER_OFFLINE:
      deadline = start + offline_timeout
    else:
      deadline = start + timeout
    if state == DOCK_UNKNOWN:
      if unknown_since is None:
        unknown_since = now
      if now - unknown_since >= unknown_grace:
        return state
    else:
      unknown_since = None
    if now >= deadline:
      return state
    sleep(poll)
    waited = True
    state = probe()
  if waited:
    sleep(settle)
  return state


def load_with_progress(loader, stream, *, total_size: int | None = None, progress_callback=None):
  """Track I/O around the official loader without changing its pickle semantics."""
  class ProgressReader:
    bytes_read = 0

    def report(self, count):
      self.bytes_read += count
      if progress_callback is not None and total_size:
        progress_callback(min(1.0, self.bytes_read / total_size))

    def read(self, size=-1):
      data = stream.read(size)
      self.report(len(data))
      return data

    def readinto(self, buffer):
      view = memoryview(buffer).cast('B')
      offset = 0
      while offset < len(view):
        count = stream.readinto(view[offset:])
        if not count:
          raise EOFError('truncated out-of-band pickle buffer')
        offset += count
        self.report(count)
      return offset

  result = loader(ProgressReader())
  if progress_callback is not None:
    progress_callback(1.0)
  return result


def configure_default_device(comma_hardware: bool, environment: MutableMapping[str, str] = os.environ, *, c3xl: bool = False) -> None:
  """Prevent tinygrad's default-device scan from probing the USB AMD GPU."""
  if comma_hardware:
    environment.setdefault("DEV", "QCOM")
  if c3xl:
    # /home is an ephemeral overlay on C3XL. Keep AMD firmware and compiler
    # caches across reboots so model startup never depends on a live download.
    environment.setdefault("XDG_CACHE_HOME", C3XL_TINYGRAD_CACHE_HOME)
    # Limit the volatile SMU PPT before clocks are opened up. An explicit
    # environment override remains available for controlled testing.
    environment.setdefault("AM_POWER_LIMIT", str(C3XL_AM_POWER_LIMIT_W))
    # Follow the SP/OP Chestnut default. Keep tinygrad's other devices unchanged.
    environment.setdefault("AMD_USB_POLL_US", str(C3XL_AMD_USB_POLL_US))


def load_with_timeout[T](load: Callable[[], T], timeout: float) -> T:
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

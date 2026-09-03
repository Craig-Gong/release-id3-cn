import importlib
import os
import signal
import time
import subprocess
from collections.abc import Callable, ValuesView
from abc import ABC, abstractmethod
from multiprocessing import Process

from setproctitle import setproctitle

from openpilot.cereal import log
from opendbc.car.structs import car
import openpilot.cereal.messaging as messaging
import openpilot.system.sentry as sentry
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog


def close_inherited_params_lock() -> None:
  """Drop forked /data/params/.lock fds.

  manager may fork while FileLock is held. flock is not recursive across
  a second fd, so the child then deadlocks on Params() — IQ.OS stays on
  the boot logo. UI also re-execs and would keep the leftover fd.
  """
  try:
    names = os.listdir("/proc/self/fd")
  except OSError:
    return
  for name in names:
    try:
      fd = int(name)
    except ValueError:
      continue
    if fd < 3:
      continue
    try:
      target = os.readlink(f"/proc/self/fd/{fd}")
    except OSError:
      continue
    if target.endswith("/params/.lock"):
      try:
        os.close(fd)
      except OSError:
        pass


def launcher(proc: str, name: str) -> None:
  try:
    close_inherited_params_lock()
    # UI child must set EGL env and re-exec before pyray loads libEGL (IQ.OS).
    if name == "ui":
      try:
        from openpilot.sunnypilot.hardware.iqos_gl import apply_iqos_gl_env, reexec_if_needed
        apply_iqos_gl_env()
        reexec_if_needed()
      except Exception:
        pass

    # import the process
    mod = importlib.import_module(proc)

    # rename the process
    setproctitle(proc)

    # create new context since we forked
    messaging.reset_context()

    # add daemon name tag to logs
    cloudlog.bind(daemon=name)
    sentry.set_tag("daemon", name)

    # exec the process
    mod.main()
  except KeyboardInterrupt:
    cloudlog.warning(f"child {proc} got SIGINT")
  except Exception:
    # can't install the crash handler because sys.excepthook doesn't play nice
    # with threads, so catch it here.
    sentry.capture_exception()
    raise


def nativelauncher(pargs: list[str], cwd: str, name: str) -> None:
  os.environ['MANAGER_DAEMON'] = name
  close_inherited_params_lock()

  # exec the process
  os.chdir(cwd)
  # locationd_llk is often built with a host RUNPATH that does not match /data/openpilot.
  gen_dir = os.path.join(cwd, "models", "generated")
  if os.path.isdir(gen_dir):
    prev = os.environ.get("LD_LIBRARY_PATH", "")
    os.environ["LD_LIBRARY_PATH"] = gen_dir if not prev else f"{gen_dir}:{prev}"
  os.execvp(pargs[0], pargs)


def join_process(process: Process, timeout: float) -> None:
  # Process().join(timeout) will hang due to a python 3 bug: https://bugs.python.org/issue28382
  # We have to poll the exitcode instead
  t = time.monotonic()
  while time.monotonic() - t < timeout and process.exitcode is None:
    time.sleep(0.001)


class ManagerProcess(ABC):
  daemon = False
  sigkill = False
  should_run: Callable[[bool, Params, car.CarParams], bool]
  proc: Process | None = None
  enabled = True
  name = ""
  shutting_down = False

  @abstractmethod
  def start(self) -> None:
    pass

  def stop(self, retry: bool = True, block: bool = True, sig: signal.Signals | None = None) -> int | None:
    if self.proc is None:
      return None

    if self.proc.exitcode is None:
      if not self.shutting_down:
        cloudlog.info(f"killing {self.name}")
        if sig is None:
          sig = signal.SIGKILL if self.sigkill else signal.SIGINT
        self.signal(sig)
        self.shutting_down = True

        if not block:
          return None

      join_process(self.proc, 5)

      # If process failed to die send SIGKILL
      if self.proc.exitcode is None and retry:
        cloudlog.info(f"killing {self.name} with SIGKILL")
        self.signal(signal.SIGKILL)
        self.proc.join()

    ret = self.proc.exitcode
    cloudlog.info(f"{self.name} is dead with {ret}")

    if self.proc.exitcode is not None:
      self.shutting_down = False
      self.proc = None

    return ret

  def signal(self, sig: int) -> None:
    if self.proc is None:
      return

    # Don't signal if already exited
    if self.proc.exitcode is not None and self.proc.pid is not None:
      return

    # Can't signal if we don't have a pid
    if self.proc.pid is None:
      return

    cloudlog.info(f"sending signal {sig} to {self.name}")
    os.kill(self.proc.pid, sig)

  def get_process_state_msg(self):
    state = log.ManagerState.ProcessState.new_message()
    state.name = self.name
    if self.proc:
      state.running = self.proc.is_alive()
      state.shouldBeRunning = self.proc is not None and not self.shutting_down
      state.pid = self.proc.pid or 0
      state.exitCode = self.proc.exitcode or 0
    return state


class NativeProcess(ManagerProcess):
  def __init__(self, name, cwd, cmdline, should_run, enabled=True, sigkill=False):
    self.name = name
    self.cwd = cwd
    self.cmdline = cmdline
    self.should_run = should_run
    self.enabled = enabled
    self.sigkill = sigkill
    self.launcher = nativelauncher

  def start(self) -> None:
    # In case we only tried a non blocking stop we need to stop it before restarting
    if self.shutting_down:
      self.stop()

    if self.proc is not None:
      if self.proc.is_alive():
        return
      cloudlog.info(f"{self.name} died, restarting")
      self.proc = None

    cwd = os.path.join(BASEDIR, self.cwd)
    cloudlog.info(f"starting process {self.name}")
    self.proc = Process(name=self.name, target=self.launcher, args=(self.cmdline, cwd, self.name))
    self.proc.start()
    self.shutting_down = False


class PythonProcess(ManagerProcess):
  def __init__(self, name, module, should_run, enabled=True, sigkill=False):
    self.name = name
    self.module = module
    self.should_run = should_run
    self.enabled = enabled
    self.sigkill = sigkill
    self.launcher = launcher

  def start(self) -> None:
    # In case we only tried a non blocking stop we need to stop it before restarting
    if self.shutting_down:
      self.stop()

    if self.proc is not None:
      if self.proc.is_alive():
        return
      cloudlog.info(f"{self.name} died, restarting")
      self.proc = None

    cloudlog.info(f"starting python {self.module}")
    self.proc = Process(name=self.name, target=self.launcher, args=(self.module, self.name))
    self.proc.start()
    self.shutting_down = False


class DaemonProcess(ManagerProcess):
  """Python process that has to stay running across manager restart.
  This is used for athena so you don't lose SSH access when restarting manager."""
  def __init__(self, name, module, param_name, enabled=True):
    self.name = name
    self.module = module
    self.param_name = param_name
    self.enabled = enabled
    self.params = None

  @staticmethod
  def should_run(started, params, CP):
    return True

  def start(self) -> None:
    if self.params is None:
      self.params = Params()

    pid = self.params.get(self.param_name)
    if pid is not None:
      try:
        os.kill(int(pid), 0)
        with open(f'/proc/{pid}/cmdline') as f:
          if self.module in f.read():
            # daemon is running
            return
      except (OSError, FileNotFoundError):
        # process is dead
        pass

    cloudlog.info(f"starting daemon {self.name}")
    proc = subprocess.Popen(['python', '-m', self.module],
                               stdin=open('/dev/null'),
                               stdout=open('/dev/null', 'w'),
                               stderr=open('/dev/null', 'w'),
                               preexec_fn=os.setpgrp)

    self.params.put(self.param_name, proc.pid, block=True)

  def stop(self, retry=True, block=True, sig=None) -> None:
    pass


# Start vision / driving daemons before offroad stops on the first onroad cycle.
# IQ.OS sensord is often late; stopping offroad procs first delays it further.
ONROAD_BOOT_PRIORITY = (
  "camerad",
  "sensord",
  "card",
  "selfdrived",
  "modeld",
  "modeld_tinygrad",
  "calibrationd",
  "locationd",
  "plannerd",
  "controlsd",
  "radard",
)


def kick_onroad_boot(procs: ValuesView[ManagerProcess], started: bool, params: Params, CP: car.CarParams,
                     not_run: list[str] | None = None) -> None:
  if not started:
    return
  if not_run is None:
    not_run = []
  proc_by_name = {p.name: p for p in procs}
  for name in ONROAD_BOOT_PRIORITY:
    p = proc_by_name.get(name)
    if p is None or not p.enabled or name in not_run:
      continue
    if p.should_run(started, params, CP):
      p.start()


def ensure_running(procs: ValuesView[ManagerProcess], started: bool, params: Params, CP: car.CarParams,
                   not_run: list[str] | None=None) -> list[ManagerProcess]:
  if not_run is None:
    not_run = []

  if started:
    kick_onroad_boot(procs, started, params, CP, not_run)

  running = []
  for p in procs:
    if p.enabled and p.name not in not_run and p.should_run(started, params, CP):
      running.append(p)
    else:
      p.stop(block=False)

  for p in running:
    p.start()

  return running

"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""
from __future__ import annotations

import argparse
import fcntl
import glob
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field

from iqpilot.system.hardware.usb import (
  EGPU_DOCK_FW_PRODUCT,
  EGPU_DOCK_ROM_USB_IDS,
  EGPU_DOCK_USB_IDS,
  ensure_host_role,
  get_link_error_count,
  get_usb_state,
)

SUPERSPEED_MBPS = 5000
USBDEVFS_RESET = 0x5514
USBGPU_CHECK_ATTEMPTS = 2
USBGPU_CHECK_RETRY_INTERVAL = 1.0
ONROAD_REFUSED = "onroad (refused)"
BENCHMARK_MIN_PASS_S = 45.0
BENCHMARK_MAX_S = 120.0
DEFAULT_POLICY_GLOB = "/data/media/0/models/egpu_*_amd_policy.pkl"


@dataclass
class ActionResult:
  ok: bool
  detail: str
  extra: dict = field(default_factory=dict)

  def __str__(self) -> str:
    return self.detail


def _started() -> bool:
  try:
    from iqpilot.common.params import Params
    return Params().get_bool("IsOnroad")
  except Exception:
    return False


def egpu_dock_entries(devices: list[dict]) -> list[dict]:
  ids = EGPU_DOCK_USB_IDS + EGPU_DOCK_ROM_USB_IDS
  return [d for d in devices if (d["vendorId"], d["productId"]) in ids]


def egpu_link_status(devices: list[dict]) -> str:
  docks = egpu_dock_entries(devices)
  if not docks:
    return "not_detected"
  if len(docks) > 1:
    return "multiple"
  dock = docks[0]
  product = dock.get("product") or ""
  speed = int(dock.get("speedMbps") or 0)
  vid, pid = dock["vendorId"], dock["productId"]
  if (vid, pid) in EGPU_DOCK_ROM_USB_IDS:
    return "rom"
  if product != EGPU_DOCK_FW_PRODUCT:
    return "firmware_mismatch"
  if speed and speed < SUPERSPEED_MBPS:
    return f"slow_usb ({speed} Mbps)"
  return "ready"


def _usb_devnode(device: dict) -> str:
  return f"/dev/bus/usb/{int(device['busnum']):03d}/{int(device['devnum']):03d}"


def _link_precheck(devices: list[dict]) -> str | None:
  link = egpu_link_status(devices)
  if link == "not_detected":
    return "USB not connected"
  if link == "multiple":
    return "multiple docks"
  if link == "rom":
    return "ROM (needs flash)"
  if link == "firmware_mismatch":
    return "firmware mismatch"
  if link.startswith("slow_usb"):
    return link
  return None


def check_gpu(*, started: bool | None = None, timeout: float = 15.0) -> str | None:
  """Offroad tinygrad 1MB + repeated numpy readback. None = pass."""
  if started if started is not None else _started():
    return ONROAD_REFUSED

  devices = get_usb_state()
  if (pre := _link_precheck(devices)) is not None:
    return pre

  docks = egpu_dock_entries(devices)
  link_errors = int(docks[0].get("linkErrorCount") or 0) if docks else 0

  env = {**os.environ, "DEV": "USB+AMD:LLVM", "GMMU": "0"}
  code = (
    "from iqpilot.selfdrive.iqmodeld.egpu_helpers import prepare_egpu_runtime; "
    "prepare_egpu_runtime(); "
    "from tinygrad import Tensor; "
    "x = Tensor.rand(1 << 20).realize(); "
    "[x.numpy() for _ in range(8)]"
  )

  for attempt in range(USBGPU_CHECK_ATTEMPTS):
    try:
      result = subprocess.run(
        [sys.executable, "-c", code],
        env=env,
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
      )
    except subprocess.TimeoutExpired:
      return "GPU check timed out"

    if result.returncode == 0:
      break

    output = f"{result.stdout}\n{result.stderr}".lower()
    pcie_not_ready = "pcie link not up" in output or "read(0xb450" in output
    if pcie_not_ready and attempt + 1 < USBGPU_CHECK_ATTEMPTS:
      time.sleep(USBGPU_CHECK_RETRY_INTERVAL)
      continue
    return "12V / PCIe not ready" if pcie_not_ready else "GPU incompatible"

  after = get_usb_state()
  if egpu_link_status(after) == "not_detected":
    return "USB disconnected during GPU check"
  after_docks = egpu_dock_entries(after)
  after_errors = int(after_docks[0].get("linkErrorCount") or 0) if after_docks else 0
  if after_errors > link_errors:
    return "USB link errors increased"
  return None


def _find_policy_pkl() -> str | None:
  env = os.getenv("IQ_EGPU_BENCHMARK_POLICY")
  if env and os.path.isfile(env):
    return env
  matches = sorted(glob.glob(DEFAULT_POLICY_GLOB), key=os.path.getmtime, reverse=True)
  return matches[0] if matches else None


def benchmark_model_load(*, policy_path: str | None = None, timeout_s: float = BENCHMARK_MAX_S,
                         min_pass_s: float = BENCHMARK_MIN_PASS_S, started: bool | None = None) -> dict:
  """Offroad load gate: construct + warmup + first infer frame with bulk/F2 counters."""
  if started if started is not None else _started():
    return {"ok": False, "error": ONROAD_REFUSED}

  devices = get_usb_state()
  if (pre := _link_precheck(devices)) is not None:
    return {"ok": False, "error": pre}

  policy_path = policy_path or _find_policy_pkl()
  if not policy_path or not os.path.isfile(policy_path):
    return {"ok": False, "error": "no local policy pkl (set IQ_EGPU_BENCHMARK_POLICY)"}

  os.environ.setdefault("XDG_CACHE_HOME", "/data/.cache")
  os.environ.setdefault("DEV", "USB+AMD:LLVM")
  os.environ.setdefault("GMMU", "0")

  from iqpilot.common.params import Params
  from iqpilot.selfdrive.iqmodeld.egpu_helpers import get_egpu_bulk_stats, prepare_egpu_runtime, reset_egpu_bulk_stats
  from iqpilot.selfdrive.iqmodeld.egpu_model import resolve_egpu_model
  from iqpilot.selfdrive.iqmodeld.egpu_policy import POLICY_FORMAT, PolicyRunner, load_bundle
  import numpy as np

  reset_egpu_bulk_stats()
  report: dict = {"ok": False, "policy": policy_path, "timeout_s": timeout_s}
  t0 = time.perf_counter()

  def _elapsed() -> float:
    return time.perf_counter() - t0

  try:
    prepare_egpu_runtime()
    report["prepare_s"] = round(_elapsed(), 3)

    params = Params()
    meta = resolve_egpu_model(params)
    if meta is None:
      return {**report, "error": "no eGPU model selected"}

    bundle = load_bundle(policy_path)
    report["load_s"] = round(_elapsed(), 3)
    report["format"] = bundle.get("format")
    if bundle.get("format") != POLICY_FORMAT:
      return {**report, "error": f"unsupported bundle format {bundle.get('format')}"}

    runner = PolicyRunner(bundle["run_policy"], bundle["input_spec"], int(bundle["frame_skip"]),
                          meta["output_slices"]["hidden_state"], bundle.get("input_device", "AMD"))
    report["construct_s"] = round(_elapsed(), 3)

    img = bundle["input_spec"]["img"][0]
    desire = bundle["input_spec"]["desire_pulse"][0][2]
    out = runner.run(np.zeros((2, 6, img[2], img[3]), dtype=np.uint8),
                     np.zeros(desire, dtype=np.float32),
                     np.zeros(2, dtype=np.float32), np.zeros(2, dtype=np.float32))
    report["warmup_s"] = round(_elapsed(), 3)
    if out.shape[0] != meta["output_len"] or not np.isfinite(out).all():
      return {**report, "error": f"invalid warmup output len={out.shape[0]}"}

    out2 = runner.run(np.zeros((2, 6, img[2], img[3]), dtype=np.uint8),
                      np.zeros(desire, dtype=np.float32),
                      np.zeros(2, dtype=np.float32), np.zeros(2, dtype=np.float32))
    report["infer_s"] = round(_elapsed(), 3)
    report["output_len"] = int(out2.shape[0])
    report["total_s"] = round(_elapsed(), 3)
    report["bulk"] = get_egpu_bulk_stats()
    report["ok"] = True
    report["gate"] = "pass" if report["total_s"] <= timeout_s and report["warmup_s"] <= timeout_s else "slow"
    if report["total_s"] < min_pass_s:
      report["gate"] = "fast"
    return report
  except Exception as e:
    report["total_s"] = round(_elapsed(), 3)
    report["bulk"] = get_egpu_bulk_stats()
    report["error"] = str(e)[:512]
    if _elapsed() >= timeout_s:
      report["gate"] = "timeout"
    return report


def cycle_ecoflow_dc(*, started: bool | None = None, off_s: float = 60.0, on_s: float = 4.0,
                     wait_s: float = 8.0) -> ActionResult:
  """Offroad EcoFlow 12V off→on. Leaves DC on. Does not guarantee USB re-enumeration."""
  if started if started is not None else _started():
    return ActionResult(False, ONROAD_REFUSED)

  try:
    from iqpilot.system.ecoflow.client import EcoflowError, EcoflowSession
  except ImportError as e:
    return ActionResult(False, f"EcoFlow client missing ({e})")

  session = None
  try:
    session = EcoflowSession.from_params()
    session.login()
    session.connect_mqtt()
    off_ack = session.set_dc12v(False, wait_s=wait_s)
    time.sleep(off_s)
    on_ack = session.set_dc12v(True, wait_s=wait_s)
    time.sleep(on_s)
    dc_on = session.dc12v_is_on() if hasattr(session, "dc12v_is_on") else None
    detail = f"12V cycled (off {off_s:.0f}s); now {'on' if dc_on else 'unknown' if dc_on is None else 'off'}"
    return ActionResult(True, detail, {"off_ack": off_ack, "on_ack": on_ack, "dc_on": dc_on})
  except EcoflowError as e:
    return ActionResult(False, str(e))
  except Exception as e:
    return ActionResult(False, f"EcoFlow cycle failed: {e}")
  finally:
    if session is not None:
      try:
        session.disconnect()
      except Exception:
        pass


def reset_dock_usb(*, started: bool | None = None) -> ActionResult:
  """USBDEVFS reset of an enumerated dock. Offroad only; does not deauthorize the port."""
  if started if started is not None else _started():
    return ActionResult(False, ONROAD_REFUSED)

  devices = get_usb_state()
  docks = egpu_dock_entries(devices)
  if not docks:
    return ActionResult(False, "USB not connected")
  if len(docks) > 1:
    return ActionResult(False, "multiple docks")

  node = _usb_devnode(docks[0])
  try:
    fd = os.open(node, os.O_RDWR)
    try:
      fcntl.ioctl(fd, USBDEVFS_RESET)
    finally:
      os.close(fd)
  except OSError as e:
    return ActionResult(False, f"reset failed ({e})")

  time.sleep(1.0)
  link = egpu_link_status(get_usb_state())
  return ActionResult(True, f"reset {node}; link={link}; cycle 12V if GPU still wedged")


def try_recover(*, off_s: float = 60.0) -> ActionResult:
  """Offroad: host role → USB reset → EcoFlow 12V cycle (USB cable stays plugged)."""
  if _started():
    return ActionResult(False, ONROAD_REFUSED)

  host = "host ok" if ensure_host_role() else "host nudge failed"
  usb = reset_dock_usb(started=False)
  dc = cycle_ecoflow_dc(started=False, off_s=off_s)
  link = egpu_link_status(get_usb_state())
  ok = usb.ok and dc.ok
  return ActionResult(ok, f"{host}; usb: {usb.detail}; dc: {dc.detail}; link={link}",
                      {"usb": usb, "dc": dc, "link": link})


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description="IQ eGPU dock offroad self-test (C3XL + EcoFlow 12V)")
  parser.add_argument("--check", action="store_true", help="1MB tensor + 8x numpy readback")
  parser.add_argument("--benchmark-load", action="store_true", help="policy load + warmup gate (JSON report)")
  parser.add_argument("--benchmark-timeout", type=float, default=BENCHMARK_MAX_S, help="load gate timeout (default 120s)")
  parser.add_argument("--benchmark-policy", default=None, help="override policy pkl path")
  parser.add_argument("--json", action="store_true", help="print benchmark JSON only")
  parser.add_argument("--recover", action="store_true", help="host + USB reset + EcoFlow 12V cycle")
  parser.add_argument("--cycle-12v", action="store_true", help="EcoFlow DC off→on only")
  parser.add_argument("--host", action="store_true", help="write ssusb mode=host")
  parser.add_argument("--usb-reset", action="store_true", help="USBDEVFS reset enumerated dock")
  parser.add_argument("--off-s", type=float, default=60.0, help="EcoFlow off duration (default 60s)")
  parser.add_argument("--status", action="store_true", help="print USB dock summary")
  args = parser.parse_args(argv)

  if _started() and any((args.check, args.benchmark_load, args.recover, args.cycle_12v, args.host, args.usb_reset)):
    print(ONROAD_REFUSED, file=sys.stderr)
    return 2

  devices = get_usb_state()
  link = egpu_link_status(devices)
  print(f"link={link} port_errors={get_link_error_count()}")

  if args.status or not any((args.check, args.benchmark_load, args.recover, args.cycle_12v, args.host, args.usb_reset)):
    for d in egpu_dock_entries(devices):
      print(f"  {d['vendorId']:04x}:{d['productId']:04x} speed={d['speedMbps']} "
            f"product={d.get('product')!r} link_err={d.get('linkErrorCount')}")
    if not any((args.check, args.benchmark_load, args.recover, args.cycle_12v, args.host, args.usb_reset)):
      return 0 if link == "ready" else 1

  if args.host:
    ok = ensure_host_role()
    print(f"host: {'ok' if ok else 'failed'}")
    if not ok:
      return 1

  if args.usb_reset:
    result = reset_dock_usb()
    print(result)
    if not result.ok:
      return 1

  if args.cycle_12v:
    result = cycle_ecoflow_dc(off_s=args.off_s)
    print(result)
    if not result.ok:
      return 1

  if args.recover:
    result = try_recover(off_s=args.off_s)
    print(result)
    if not result.ok:
      return 1

  if args.check:
    err = check_gpu()
    if err:
      print(f"check failed: {err}", file=sys.stderr)
      return 1
    print("check passed: 1MB realize + 8x numpy")

  if args.benchmark_load:
    report = benchmark_model_load(policy_path=args.benchmark_policy, timeout_s=args.benchmark_timeout)
    if args.json:
      print(json.dumps(report, indent=2, sort_keys=True))
    else:
      print(json.dumps(report, indent=2, sort_keys=True))
    if not report.get("ok"):
      return 1
    if report.get("gate") == "timeout" or report.get("total_s", 0) > args.benchmark_timeout:
      return 1

  return 0


if __name__ == "__main__":
  raise SystemExit(main())

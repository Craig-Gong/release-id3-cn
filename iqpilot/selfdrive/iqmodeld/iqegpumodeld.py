"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""
from __future__ import annotations

import os
os.environ.setdefault("XDG_CACHE_HOME", "/data/.cache")
import pickle
import subprocess
import sys
import time

from iqpilot.system.hardware import TICI

os.environ.setdefault("GMMU", "0")
if TICI:
  os.environ.setdefault("DEV", "USB+AMD:LLVM")
else:
  os.environ.setdefault("DEV", "CPU")

import numpy as np
from setproctitle import setproctitle

import iqpilot.cereal.messaging as messaging
from iqpilot.cereal import car, log
from iqpilot.cereal.messaging import SubMaster
from iqpilot.cereal.services import SERVICE_LIST
from iqdbc.car.car_helpers import get_demo_car_params

from iqpilot.common.params import Params
from iqpilot.common.realtime import DT_MDL
from iqpilot.common.swaglog import cloudlog
from iqpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from iqpilot.system import sentry

from iqpilot.common.steer_delay import lateral_action_delay
from iqpilot.selfdrive.iqmodeld.daemon import CalibrationAtlas, CameraIngress, FrameDropMeter
from iqpilot.selfdrive.iqmodeld.driving_action import (
  DESIRE_LEN, LAT_SMOOTH_SECONDS, LONG_SMOOTH_SECONDS, get_action_from_model,
)
from iqpilot.selfdrive.iqmodeld.egpu_helpers import (
  download_onnx, download_precompiled, egpu_artifact_prefers_local_policy, egpu_oob_pkl_path, egpu_pkl_path, egpu_policy_pkl_path, egpu_present_consented, egpu_selected, local_onnx,
  patch_tinygrad_fetch_fw, prepare_egpu_runtime, quarantine_artifact, resolve_backend, usbgpu_present,
)
from iqpilot.selfdrive.iqmodeld.egpu_model import resolve_egpu_model
from iqpilot.selfdrive.iqmodeld.egpu_pipeline import EgpuPipeline, EgpuPipelineError, make_big_channel_payload
from iqpilot.selfdrive.iqmodeld.egpu_telemetry import EgpuDockTelemetry
from iqpilot.selfdrive.iqmodeld.messaging import DrivePacketMemory, populate_drive_messages, populate_odometry_message
from iqpilot.selfdrive.iqmodeld.metadata import Meta20hz
from iqpilot.selfdrive.iqmodeld.model_channel import BIG_CHANNEL, ModelChannel
from iqpilot.selfdrive.iqmodeld.egpu_policy import POLICY_FORMAT, PolicyRunner, load_bundle
from iqpilot.selfdrive.iqmodeld.model_warp import FrameWarp
from iqpilot.selfdrive.iqmodeld.parser import PhaseParser

PROCESS_NAME = "iqpilot.selfdrive.iqmodeld.iqegpumodeld"

PRESENCE_POLL_S = 5.0
COMPILE_TIMEOUT_S = 3600
LINK_UP_TIMEOUT_S = 10.0
SETUP_EXIT_AFTER = 3
MIN_LOAD_AVAIL_MB = 350
MEMORY_WAIT_S = 90.0
SETUP_RETRY_BASE_S = 3.0
SETUP_RETRY_MAX_S = 30.0


def park(reason: str) -> None:
  cloudlog.warning(f"iqegpumodeld parked: {reason}")
  params = Params()
  params.put_bool("UsbGpuFailed", True)
  params.put("UsbGpuLastError", reason[:512])
  while True:
    time.sleep(1)


def _wait_for_egpu(params: Params) -> None:
  while not usbgpu_present():
    params.put_bool("UsbGpuPresent", False)
    time.sleep(PRESENCE_POLL_S)
  params.put_bool("UsbGpuPresent", True)
  try:
    from iqpilot.system.hardware.egpu_dock.flash import link_up
  except Exception:
    return
  deadline = time.monotonic() + LINK_UP_TIMEOUT_S
  while time.monotonic() < deadline:
    try:
      if link_up():
        return
    except Exception:
      return
    time.sleep(0.5)


def _compile_in_subprocess(meta: dict, onnx_path: str, pkl_path: str) -> None:
  cmd = [sys.executable, "-m", "iqpilot.selfdrive.iqmodeld.tools.compile_egpu_model",
         "--model", meta["key"], "--onnx", onnx_path, "--output", pkl_path,
         "--progress-param", "UsbGpuSetupProgress", "--progress-base", "0.5", "--progress-span", "0.48"]
  compile_env = {**os.environ, "DEV": "USB+AMD:LLVM", "FLOAT16": "1",
                 "JIT_BATCH_SIZE": "0", "GMMU": "0", "TC_OPT": "2"}
  proc = subprocess.run(cmd, timeout=COMPILE_TIMEOUT_S, capture_output=True, text=True,
                        env=compile_env, preexec_fn=lambda: os.nice(20))
  if proc.returncode != 0:
    tail = (proc.stderr or proc.stdout or "").strip()[-800:]
    raise RuntimeError(f"eGPU model compile failed (rc={proc.returncode}): {tail}")


_precompiled_tried = False


def _ensure_artifact(params: Params, meta: dict) -> str:
  global _precompiled_tried
  oob_path = egpu_oob_pkl_path(meta)
  if os.path.isfile(oob_path):
    return oob_path
  policy_path = egpu_policy_pkl_path(meta)
  legacy_path = egpu_pkl_path(meta)

  params.put_bool("UsbGpuCompiled", False)
  params.put_bool("UsbGpuReady", False)

  if os.path.isfile(policy_path):
    if egpu_artifact_prefers_local_policy(params):
      cloudlog.warning(f"iqegpumodeld using cached policy (IQEgpuPreferLocalPolicy) -> {policy_path}")
    else:
      cloudlog.warning(f"iqegpumodeld using cached policy on disk -> {policy_path}")
    return policy_path

  if meta.get("egpu_oob_artifact") and not _precompiled_tried:
    _precompiled_tried = True
    params.put("UsbGpuSetupProgress", "0.0")
    oob_last = [-1.0]

    def _oob_prog(p: float) -> None:
      if p - oob_last[0] >= 0.02 or p >= 1.0:
        oob_last[0] = p
        params.put("UsbGpuSetupProgress", f"{p:.3f}")

    try:
      cloudlog.warning(f"iqegpumodeld downloading precompiled {meta['key']} (streamable) "
                       f"({int(meta['egpu_oob_artifact'].get('size', 0)) / 1e6:.0f}MB)")
      precompiled = download_precompiled(meta, progress_cb=_oob_prog, oob=True)
      if precompiled is not None:
        cloudlog.warning(f"iqegpumodeld precompiled ready -> {precompiled}")
        return precompiled
    except Exception as e:
      cloudlog.warning(f"iqegpumodeld streamable artifact unavailable ({e}); falling back")

  if os.path.isfile(policy_path):
    return policy_path

  if meta.get("egpu_policy_artifact") and not _precompiled_tried:
    _precompiled_tried = True
    params.put("UsbGpuSetupProgress", "0.0")
    dl_last = [-1.0]

    def _dl_prog(p: float) -> None:
      if p - dl_last[0] >= 0.02 or p >= 1.0:
        dl_last[0] = p
        params.put("UsbGpuSetupProgress", f"{p:.3f}")

    try:
      cloudlog.warning(f"iqegpumodeld downloading precompiled {meta['key']} policy "
                       f"({int(meta['egpu_policy_artifact'].get('size', 0)) / 1e6:.0f}MB)")
      precompiled = download_precompiled(meta, progress_cb=_dl_prog, policy=True)
      if precompiled is not None:
        cloudlog.warning(f"iqegpumodeld precompiled ready -> {precompiled}")
        return precompiled
    except Exception as e:
      cloudlog.warning(f"iqegpumodeld precompiled policy unavailable ({e}); falling back")

  if os.path.isfile(legacy_path):
    cloudlog.warning(f"iqegpumodeld using legacy per-tensor artifact {legacy_path}; policy artifact not hosted yet")
    return legacy_path

  onnx_path = local_onnx(meta)
  if onnx_path is None:
    params.put("UsbGpuSetupProgress", "0.0")
    cloudlog.warning(f"iqegpumodeld downloading {meta['key']} onnx ({meta.get('download', {}).get('size', 0) / 1e6:.0f}MB)")
    last = [-1.0]

    def _prog(p: float) -> None:
      if p - last[0] >= 0.02 or p >= 1.0:
        last[0] = p
        params.put("UsbGpuSetupProgress", f"{p * 0.5:.3f}")

    onnx_path = download_onnx(meta, progress_cb=_prog)

  cloudlog.warning(f"iqegpumodeld compiling {meta['key']} for USB-AMD (one-time, can take minutes)")
  _compile_in_subprocess(meta, onnx_path, policy_path)
  cloudlog.warning(f"iqegpumodeld compiled -> {policy_path}")
  return policy_path


def _mem_available_mb() -> int:
  try:
    with open("/proc/meminfo") as f:
      for line in f:
        if line.startswith("MemAvailable:"):
          return int(line.split()[1]) // 1024
  except OSError:
    pass
  return 1 << 20


def _wait_for_memory(need_mb: int) -> None:
  deadline = time.monotonic() + MEMORY_WAIT_S
  avail = _mem_available_mb()
  while avail < need_mb and time.monotonic() < deadline:
    cloudlog.warning(f"iqegpumodeld waiting for memory: {avail}MB available, need {need_mb}MB")
    time.sleep(5.0)
    avail = _mem_available_mb()
  if avail < need_mb:
    raise RuntimeError(f"insufficient memory to load the dock model: {avail}MB available, need {need_mb}MB")


def _load_infer_fn(pkl_path: str, meta: dict):
  prepare_egpu_runtime()
  from tinygrad.tensor import Tensor

  _wait_for_memory(MIN_LOAD_AVAIL_MB)
  bundle = load_bundle(pkl_path)
  if bundle.get("model_sha256") != meta["sha256"]:
    quarantine_artifact(pkl_path, "pkl model sha mismatch")
    raise RuntimeError(f"artifact model sha {bundle.get('model_sha256')} != {meta['sha256']}")
  if int(bundle.get("output_len", -1)) != int(meta["output_len"]):
    quarantine_artifact(pkl_path, "pkl output_len mismatch")
    raise RuntimeError(f"artifact output_len {bundle.get('output_len')} != {meta['output_len']}")
  if bundle.get("format") == POLICY_FORMAT:
    runner = PolicyRunner(bundle["run_policy"], bundle["input_spec"], int(bundle["frame_skip"]),
                          meta["output_slices"]["hidden_state"], bundle.get("input_device", "AMD"))
    return runner, bundle["input_spec"]
  jit = bundle["run_model"]
  input_dev = bundle.get("input_device", "AMD")
  input_spec = bundle["input_spec"]

  def infer(inputs: dict[str, np.ndarray]) -> np.ndarray:
    tensors = {name: Tensor(np.ascontiguousarray(inputs[name]), device=input_dev).realize()
               for name in input_spec}
    out, = jit(**tensors)
    return out.numpy().reshape(-1)

  return infer, input_spec


def _warmup(infer_fn, input_spec: dict, output_len: int) -> float:
  zeros = {name: np.zeros(shape, dtype=dtype) for name, (shape, dtype) in input_spec.items()}
  t0 = time.perf_counter()
  if isinstance(infer_fn, PolicyRunner):
    img = input_spec["img"][0]
    out = infer_fn.run(np.zeros((2, 6, img[2], img[3]), dtype=np.uint8), np.zeros(input_spec["desire_pulse"][0][2], dtype=np.float32),
                       np.zeros(2, dtype=np.float32), np.zeros(2, dtype=np.float32))
  else:
    out = infer_fn(zeros)
  dt = time.perf_counter() - t0
  if out.shape[0] != output_len or not np.isfinite(out).all():
    raise RuntimeError(f"warmup produced invalid output (len={out.shape[0]})")
  return dt


def main(demo: bool = False) -> None:
  cloudlog.warning("iqegpumodeld init")
  sentry.set_tag("daemon", PROCESS_NAME)
  cloudlog.bind(daemon=PROCESS_NAME)
  setproctitle(PROCESS_NAME)
  try:
    os.sched_setaffinity(0, {4, 5, 6})
    os.nice(-10)
  except OSError as e:
    cloudlog.warning(f"iqegpumodeld affinity/nice failed ({e}); continuing at defaults")

  params = Params()
  backend = resolve_backend(params.get_bool("IQEmacEnabled"), egpu_selected(params), egpu_present_consented(params))
  if backend != "egpu":
    park(f"backend resolution is {backend!r}, not egpu; refusing to own the big channel")

  channel = ModelChannel(BIG_CHANNEL, create=True)

  cloudlog.warning("iqegpumodeld waiting for camerad")
  cameras = CameraIngress(None)
  layout = cameras.layout

  _wait_for_egpu(params)
  os.environ["DEV"] = "USB+AMD:LLVM"
  prepare_egpu_runtime()
  params.put_bool("UsbGpuLoading", True)
  attempt = 0
  while True:
    try:
      meta = resolve_egpu_model(params)
      if meta is None:
        raise RuntimeError("selected big model is not in the catalog; check connectivity or pick another model")
      if meta.get("split"):
        params.put_bool("UsbGpuLoading", False)
        park(f"model {meta['key']} needs the Mac backend; the eGPU runs fused models only")
      warp = FrameWarp(cameras._primary.width, cameras._primary.height, meta["frame_skip"])
      pkl_path = _ensure_artifact(params, meta)
      infer_fn, input_spec = _load_infer_fn(pkl_path, meta)
      warm_s = _warmup(infer_fn, input_spec, meta["output_len"])
      break
    except Exception as e:
      attempt += 1
      subs = "; ".join(f"{type(x).__name__}: {x}" for x in (getattr(e, "exceptions", None) or []))
      err = f"{e} [{subs}]" if subs else str(e)
      params.put("UsbGpuLastError", err[:512])
      cloudlog.warning(f"iqegpumodeld setup attempt {attempt} failed: {e}; {subs}; retrying")
      # A GPU hang wedges the dock; retrying in-process or respawning just loops alerts/UI flicker.
      if "Device hang detected" in err or "Wait timeout" in err:
        params.put_bool("UsbGpuLoading", False)
        park(f"eGPU model load failed ({e}); cycle dock 12V and retry offroad")
      if attempt >= SETUP_EXIT_AFTER:
        params.put_bool("UsbGpuLoading", False)
        # tinygrad keeps the dock's flock in a failed device init, so a stale process can never
        # reopen it; exit and let the manager respawn a clean one.
        cloudlog.error(f"iqegpumodeld giving up after {attempt} setup failures; exiting for a clean restart")
        sys.exit(1)
      if not usbgpu_present():
        from iqpilot.system.hardware.usb import ensure_host_role
        if ensure_host_role():
          cloudlog.warning("iqegpumodeld: Type-C controller was out of host mode; restored")
          time.sleep(2.0)
        _wait_for_egpu(params)
      time.sleep(min(SETUP_RETRY_MAX_S, SETUP_RETRY_BASE_S * attempt))

  params.put_bool("UsbGpuLoading", False)
  params.put_bool("UsbGpuCompiled", True)
  params.put_bool("UsbGpuReady", True)
  params.put("UsbGpuSetupProgress", "1.0")
  cloudlog.warning(f"iqegpumodeld model: {meta['key']} ({meta['model_name']})")
  cloudlog.warning(f"iqegpumodeld model up (warmup {warm_s * 1e3:.0f}ms)")

  pipeline = EgpuPipeline(meta, infer_fn)
  telemetry_pm = messaging.PubMaster(["egpuDockState"])
  telemetry = EgpuDockTelemetry(telemetry_pm, big=True)
  telemetry_every = max(1, round((1.0 / DT_MDL) / SERVICE_LIST["egpuDockState"].frequency))

  sub = SubMaster(["deviceState", "carState", "roadCameraState", "extrinsicsCalibration",
                   "driverMonitoringState", "carControl", "lateralDelay", "iqNavState", "radarState"])
  if demo:
    CP = get_demo_car_params()
  else:
    CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  long_delay = CP.longitudinalActuatorDelay + LONG_SMOOTH_SECONDS

  parser = PhaseParser()
  memory = DrivePacketMemory()
  desire_logic = DesireHelper()
  frame_meter = FrameDropMeter(20.0)
  warps = CalibrationAtlas()
  prev_action = log.ModelDataV2.Action()
  slices = {k: v for k, v in meta["output_slices"].items() if k != "pad"}

  produced = 0
  stats: dict[str, list[float]] = {k: [] for k in ("pull", "warp", "infer", "publish", "loop")}
  iter_count = 0
  skip_count = 0
  last_pulled_fid = -1
  last_frame_mono = time.monotonic()
  t_loop = time.perf_counter()
  cloudlog.warning("iqegpumodeld starting")

  while True:
    frame_pair = cameras.pull()
    t_pull = time.perf_counter()
    if frame_pair is None:
      if time.monotonic() - last_frame_mono > 2.0:
        cloudlog.warning("iqegpumodeld camera stream silent >2s; reconnecting VisionIPC")
        cameras = CameraIngress(None)
        last_frame_mono = time.monotonic()
      continue
    last_frame_mono = time.monotonic()
    main_buf, extra_buf, main_stamp, extra_stamp = frame_pair

    stats["pull"].append(t_pull - t_loop)
    stats["loop"].append(time.perf_counter() - t_loop)
    t_loop = time.perf_counter()
    if last_pulled_fid >= 0 and main_stamp.frame_id > last_pulled_fid + 1:
      skip_count += main_stamp.frame_id - last_pulled_fid - 1
    last_pulled_fid = main_stamp.frame_id
    iter_count += 1
    if iter_count % 200 == 0:
      pcts = {k: {"p50": round(sorted(v)[len(v) // 2] * 1e3, 1),
                  "p90": round(sorted(v)[int(len(v) * 0.9)] * 1e3, 1)}
              for k, v in stats.items() if v}
      cloudlog.event("iqegpu_stats", **pcts, cam_skips=skip_count, window=iter_count)
      msg = " ".join(f"{k}=p50:{v['p50']:.0f}/p90:{v['p90']:.0f}ms" for k, v in pcts.items())
      cloudlog.warning(f"iqegpumodeld stages: {msg} cam_skips={skip_count} over {iter_count}")
      for v in stats.values():
        v.clear()
      skip_count = 0

    sub.update(0)

    v_ego = max(sub["carState"].vEgo, 0.0)
    lat_delay = lateral_action_delay(params, CP, sub["lateralDelay"].lateralDelay) + LAT_SMOOTH_SECONDS
    main_tfm, extra_tfm, live_calib_seen = warps.refresh(sub, layout.main_is_wide, layout.dual_camera)
    dropped_frames, frame_drop_ratio, _ = frame_meter.sample(main_stamp.frame_id)

    traffic = np.zeros(2, dtype=np.float32)
    traffic[int(sub["driverMonitoringState"].isRHD)] = 1
    desire_vec = np.zeros(DESIRE_LEN, dtype=np.float32)
    if 0 <= desire_logic.desire < DESIRE_LEN:
      desire_vec[desire_logic.desire] = 1

    frame_delay = DT_MDL
    action_delay = DT_MDL / 2
    lat_action_t = lat_delay + frame_delay + action_delay
    long_action_t = long_delay + frame_delay + action_delay
    action_t = np.array([lat_action_t, long_action_t], dtype=np.float32)

    started_at = time.perf_counter()
    try:
      warped = warp.run(main_buf, extra_buf, main_tfm, extra_tfm)
    except Exception as e:
      park(f"warp run failed: {e}")
    t_warp = time.perf_counter()
    stats["warp"].append(t_warp - started_at)

    try:
      output = pipeline.run(warped, desire_vec, traffic, action_t)
    except EgpuPipelineError as e:
      park(str(e))
    except Exception as e:
      park(f"eGPU inference failed: {e}")
    t_infer = time.perf_counter()
    stats["infer"].append(t_infer - t_warp)

    execution_time = time.perf_counter() - started_at
    sliced = {k: output[np.newaxis, sl] for k, sl in slices.items()}
    outputs = parser.parse_vision_outputs(sliced)

    action = get_action_from_model(outputs, prev_action, v_ego, float(lat_action_t), float(long_action_t),
                                   lat_smooth_seconds=meta.get("lat_smooth_seconds"))
    prev_action = action

    model_msg = messaging.new_message("modelV2")
    driving_msg = messaging.new_message("drivingModelData")
    pose_msg = messaging.new_message("cameraOdometry")
    iq_msg = messaging.new_message("iqDriveModelData")

    populate_drive_messages(
      driving_msg, model_msg, outputs, action, memory,
      main_stamp.frame_id, extra_stamp.frame_id, sub["roadCameraState"].frameId,
      frame_drop_ratio, main_stamp.timestamp_eof, execution_time,
      live_calib_seen, Meta20hz,
    )

    model_msg.modelV2.big = True

    desire_state = model_msg.modelV2.meta.desireState
    lane_change_prob = desire_state[log.Desire.laneChangeLeft] + desire_state[log.Desire.laneChangeRight]
    desire_logic.update(sub["carState"], sub["carControl"].latActive, lane_change_prob,
                        sub["iqNavState"], model_msg.modelV2, sub["radarState"])
    model_msg.modelV2.meta.laneChangeState = desire_logic.lane_change_state
    model_msg.modelV2.meta.laneChangeDirection = desire_logic.lane_change_direction
    driving_msg.drivingModelData.meta.laneChangeState = desire_logic.lane_change_state
    driving_msg.drivingModelData.meta.laneChangeDirection = desire_logic.lane_change_direction
    iq_msg.iqDriveModelData.turnSignalDirection = desire_logic.lane_turn_direction

    populate_odometry_message(pose_msg, outputs, main_stamp.frame_id, dropped_frames,
                              main_stamp.timestamp_eof, live_calib_seen)

    channel.write(main_stamp.frame_id, make_big_channel_payload(
      main_stamp.frame_id, live_calib_seen, execution_time, (t_infer - t_warp) * 1e3, {
        "modelV2": model_msg.to_bytes(),
        "drivingModelData": driving_msg.to_bytes(),
        "cameraOdometry": pose_msg.to_bytes(),
        "iqDriveModelData": iq_msg.to_bytes(),
      }))
    stats["publish"].append(time.perf_counter() - t_infer)
    produced += 1
    if produced == 1 or produced % 100 == 0:
      infer_ms = (t_infer - t_warp) * 1e3
      cloudlog.warning(f"iqegpumodeld producing: frame={main_stamp.frame_id} total={execution_time * 1e3:.0f}ms infer={infer_ms:.0f}ms count={produced}")

    if produced % telemetry_every == 0:
      telemetry.send()

    frame_meter.commit(main_stamp.frame_id)


if __name__ == "__main__":
  try:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true")
    args = ap.parse_args()
    main(demo=args.demo)
  except KeyboardInterrupt:
    cloudlog.warning("iqegpumodeld got SIGINT")
  except Exception:
    import traceback
    sentry.capture_exception()
    cloudlog.exception("iqegpumodeld crashed, parking")
    park(f"crashed: {traceback.format_exc(limit=8)}")

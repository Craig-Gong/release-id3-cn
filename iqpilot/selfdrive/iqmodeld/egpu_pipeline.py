"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""
from __future__ import annotations

import numpy as np

from iqpilot.selfdrive.iqmodeld.egpu_policy import PolicyRunner
from iqpilot.selfdrive.iqmodeld.temporal_state import MODEL_INPUT_SPEC, TemporalInputState, spec_from_meta


class EgpuPipelineError(RuntimeError):
  pass


class EgpuPipeline:

  def __init__(self, meta: dict, infer_fn):
    if meta.get("split"):
      raise EgpuPipelineError(f"model {meta['key']} is a split model; eGPU v1 runs fused models only")
    self.meta = meta
    self.infer_fn = infer_fn
    self.state = TemporalInputState(meta["frame_skip"], spec_from_meta(meta) or MODEL_INPUT_SPEC)
    self.hidden_slice = meta["output_slices"]["hidden_state"]
    self.output_len = int(meta["output_len"])

  def run(self, warped: np.ndarray, desire_vec: np.ndarray, traffic_convention: np.ndarray,
          action_t: np.ndarray) -> np.ndarray:
    if isinstance(self.infer_fn, PolicyRunner):
      out = np.asarray(self.infer_fn.run(warped, desire_vec, traffic_convention, action_t), dtype=np.float32).reshape(-1)
    else:
      inputs = self.state.push_and_materialize(warped, desire_vec, traffic_convention, action_t)
      out = np.asarray(self.infer_fn(inputs), dtype=np.float32).reshape(-1)
    if out.shape[0] != self.output_len:
      raise EgpuPipelineError(f"eGPU output length {out.shape[0]} != {self.output_len}")
    if not np.isfinite(out).all():
      raise EgpuPipelineError("eGPU output contains non-finite values")
    if not isinstance(self.infer_fn, PolicyRunner):
      self.state.note_hidden_state(out, self.hidden_slice)
    return out


def make_big_channel_payload(frame_id: int, live_calib_seen: bool, execution_time: float,
                             egpu_exec_ms: float, msgs: dict[str, bytes]) -> dict:
  return {
    "source": "egpu_big",
    "frame_id": int(frame_id),
    "live_calib_seen": bool(live_calib_seen),
    "model_execution_time": float(execution_time),
    "egpu_exec_ms": float(egpu_exec_ms),
    "msgs": msgs,
  }

#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  # Target system image is AGNOS 19.6 (hybrid C3XL boot chain in agnos.json).
  # IQ.OS reports "IQ.OS 4.9.x"; matching that string here prevents updated.py
  # from auto-flashing while still on IQ.OS. After a hybrid swap, /VERSION is 19.6.
  if [ -f /VERSION ] && grep -q '^IQ.OS' /VERSION; then
    export AGNOS_VERSION="$(cat /VERSION)"
  else
    export AGNOS_VERSION="19.6"
  fi
fi

export STAGING_ROOT="/data/safe_staging"

# IQ.OS 4.9 EGL has AR24, not the AB24 comma raylib requires.
if [ -f /VERSION ] && grep -q '^IQ.OS' /VERSION; then
  _EGL_SHIM="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/openpilot/sunnypilot/hardware/libiqos_egl_abgr_compat.so"
  if [ -f "$_EGL_SHIM" ]; then
    export LD_PRELOAD="${_EGL_SHIM}${LD_PRELOAD:+:}${LD_PRELOAD}"
  fi
  export LIBGL_ALWAYS_SOFTWARE=0
  # Adreno ICD is applied in the UI child (iqos_gl.py) so a failed
  # InitWindow can fall back to Mesa. Do not set it for the whole tree.
fi

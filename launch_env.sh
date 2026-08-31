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
  DEVICE_MODEL=""
  if [ -f /sys/firmware/devicetree/base/model ]; then
    DEVICE_MODEL="$(tr -d '\0' </sys/firmware/devicetree/base/model)"
  fi

  case "$DEVICE_MODEL" in
    *)
      # C3XL is on IQ.OS 4.9.3. Official 4be198e expects 4.9.7 and would
      # flash IQ.OS on first boot (launch agnos_init + updated.py). Do not.
      export AGNOS_VERSION="IQ.OS 4.9.3"
      export AGNOS_COMPAT_VERSIONS="IQ.OS 4.9.3,IQ.OS 4.9.7"
      ;;
  esac
fi

export STAGING_ROOT="/data/safe_staging"

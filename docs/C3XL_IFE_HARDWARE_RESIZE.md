# C3XL IFE hardware road-camera resize

C3XL OX03C10 road cameras are **1928×1208**. Chestnut / CTM catalogs target **comma four / mici 1344×760**. Without IFE, C3XL either feeds the wrong size into CTM or pays for GPU software resize (`C3XL_CTMV2_INPUT_RESIZE`) — that mismatch is the main CTM lag / `modeldLagging` path on this device. IFE scales in the ISP before VisionIPC so CTM sees native 1344×760 (full FOV scale, not a crop).

## Default on C3XL (this tree)

`manager_init` calls `apply_c3xl_ife_runtime()`:

1. Persist `/data/hardware_profile` = `c3xl` when the device is C3XL (file required by native camerad; inference alone is not enough).
2. `setdefault` `C3XL_IFE_ROAD_SIZE=1344x760` for child processes (camerad / modeld).
3. Unset `C3XL_CTMV2_INPUT_RESIZE` so it cannot fight IFE.

Camerad still requires OX03C10 @ 1928×1208, IFE-processed road streams, and camera 0/1. Cabin/BPS unchanged.

## Disable (native 1928×1208 models)

Before openpilot starts (e.g. `/data/continue.sh`):

```bash
export C3XL_IFE_ROAD_SIZE=off
```

Accepted disable values: empty, `0`, `off`, `false`, `no`. Then restart through the normal offroad lifecycle. Do not leave a stale `1344x760` export in continue.sh if you intend to disable.

## Deploy

- Python / manager: rsync + 熄火→READY (or reboot offroad).
- **camerad binary must include the IFE path** (`scons` camerad on device after first pull of this code). Env alone cannot resize without the rebuilt binary.
- modeld refuses startup if IFE is requested but VisionIPC is still 1928×1208 (usually means camerad not rebuilt) or `/data/hardware_profile` is not `c3xl`.

## Evidence (upstream onemiless)

Verified with CTMv2 at 1344×760 (~20 Hz, P50 ~40 ms). TSFM fallback also ran at 1344×760. Confirm other big-model artifacts include the 1344×760 compile keys before relying on IFE with them; otherwise disable IFE for that model.

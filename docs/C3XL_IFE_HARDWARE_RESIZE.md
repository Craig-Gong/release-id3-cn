# C3XL IFE hardware road-camera resize

C3XL OX03C10 road cameras are **1928×1208**. Chestnut **CTM** artifacts target **comma four / mici 1344×760**. That size mismatch (or the old GPU software-resize path) is why **CTM** lags / trips `modeldLagging` on C3XL; other models that already ship 1928×1208 keys are fine without IFE. Aligns with onemiless `dev-sp-egpu`: IFE is **opt-in**, default off.

IFE scales in the ISP before VisionIPC so CTM sees native 1344×760 (full FOV scale, not a crop). Model weights, frame cadence, control permissions and frame-drop checks are unchanged.

**UI:** when VisionIPC is **1344×760**, only the **wide** stream is cropped for dash/UR framing on the big screen. **Narrow/tele** keeps stock zoom 1.1 on the full IFE buffer. Modeld still uses IFE-patched `DEVICE_CAMERAS` when its env is set.

## Activation (same as onemiless)

Set in the startup environment **before** openpilot (verified device uses `/data/continue.sh`; not in the repo):

```bash
export C3XL_IFE_ROAD_SIZE=1344x760
# keep unset:
# C3XL_CTMV2_INPUT_RESIZE
```

Also need `/data/hardware_profile` exactly `c3xl`, OX03C10 @ 1928×1208, IFE-processed road cameras 0/1. Cabin/BPS unchanged.

This tree: when the env is already set, `manager_init` **persists** `/data/hardware_profile=c3xl` (camerad native gate needs the file; Python inference alone is not enough) and clears `C3XL_CTMV2_INPUT_RESIZE`. It does **not** turn IFE on by itself.

## Rollback

Unset `C3XL_IFE_ROAD_SIZE` in the same startup environment and restart through the normal offroad lifecycle. Other (non-CTM) models stay on native 1928×1208.

## Deploy

- Python / manager: rsync + 熄火→READY.
- **Rebuild camerad** once so the IFE C++ path exists; env alone cannot resize with an old binary.
- modeld refuses startup if IFE is requested but VisionIPC is still 1928×1208 or profile file is not `c3xl`.

## Evidence (onemiless)

CTMv2 @ 1344×760 ~20 Hz, P50 ~40 ms; TSFM fallback also at 1344×760. Only those were checked — do not leave IFE on for unrelated big-model artifacts that lack 1344×760 compile keys.

"""EcoflowEnabled source of truth: /data/ecoflow_params/.

IQ.OS rebuilds /data/params/d on boot. Native Params EcoflowEnabled defaults
to 0. Overlay file wins. Native Params is a cache only. Heal:
  * creds present + Enabled file missing → treat as on (leftover)
  * overlay 1 → mirror native True so other readers agree
  * native True + overlay missing → write overlay so the next IQ.OS wipe
    cannot turn the daemon off
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

ECOFLOW_PARAM_DIR = Path("/data/ecoflow_params")
STATUS_NAME = "status.json"
_TRUE = {"1", "true", "True", "yes", "on"}
_FALSE = {"0", "false", "False", "no", "off"}


def overlay_path(name: str, param_dir: Path | None = None) -> Path:
  return (param_dir or ECOFLOW_PARAM_DIR) / name


def read_overlay_text(name: str, param_dir: Path | None = None) -> str:
  try:
    return overlay_path(name, param_dir).read_text(encoding="utf-8").strip()
  except OSError:
    return ""


def overlay_enabled(param_dir: Path | None = None) -> bool | None:
  raw = read_overlay_text("EcoflowEnabled", param_dir)
  if not raw:
    return None
  if raw in _TRUE:
    return True
  if raw in _FALSE:
    return False
  return None


def write_overlay_enabled(on: bool, param_dir: Path | None = None) -> None:
  dest = param_dir or ECOFLOW_PARAM_DIR
  dest.mkdir(parents=True, exist_ok=True)
  path = dest / "EcoflowEnabled"
  tmp = path.with_suffix(".tmp")
  tmp.write_text("1" if on else "0", encoding="utf-8")
  os.replace(tmp, path)


def _params_get(params: Any, key: str) -> str:
  if params is None:
    return ""
  try:
    raw = params.get(key)
  except Exception:
    return ""
  if raw is None:
    return ""
  if isinstance(raw, (bytes, bytearray)):
    return raw.decode("utf-8", errors="replace").strip()
  return str(raw).strip()


def has_credentials(param_dir: Path | None = None, params: Any = None) -> bool:
  def _val(key: str) -> str:
    return read_overlay_text(key, param_dir) or _params_get(params, key)

  password = _val("EcoflowPassword")
  sn = _val("EcoflowSn")
  account = _val("EcoflowPhone") or _val("EcoflowEmail")
  return bool(password and sn and account)


def is_enabled(params: Any = None, param_dir: Path | None = None) -> bool:
  overlay = overlay_enabled(param_dir)
  if overlay is not None:
    return overlay
  if has_credentials(param_dir, params):
    return True
  if params is None:
    return False
  try:
    raw = params.get("EcoflowEnabled")
  except Exception:
    return False
  if raw is None:
    return False
  try:
    return bool(params.get_bool("EcoflowEnabled"))
  except Exception:
    return False


def set_enabled(on: bool, params: Any = None, param_dir: Path | None = None) -> None:
  """Write overlay first (survives IQ.OS wipe), then mirror native Params."""
  write_overlay_enabled(on, param_dir)
  if params is None:
    return
  try:
    params.put_bool("EcoflowEnabled", bool(on), block=True)
  except TypeError:
    try:
      params.put_bool("EcoflowEnabled", bool(on))
    except Exception:
      pass
  except Exception:
    pass


def heal_enabled(params: Any = None, param_dir: Path | None = None) -> bool:
  """Repair Enabled so a params/d rebuild cannot silently disable ecoflowd."""
  dest = param_dir or ECOFLOW_PARAM_DIR
  overlay = overlay_enabled(dest)

  if overlay is None and has_credentials(dest, params):
    write_overlay_enabled(True, dest)
    overlay = True

  if overlay is True:
    if params is not None:
      try:
        if not bool(params.get_bool("EcoflowEnabled")):
          set_enabled(True, params, dest)
      except Exception:
        write_overlay_enabled(True, dest)
    return True

  if overlay is False:
    return False

  native_on = False
  if params is not None:
    try:
      native_on = bool(params.get_bool("EcoflowEnabled"))
    except Exception:
      native_on = False
  if native_on:
    write_overlay_enabled(True, dest)
    return True
  return False


def write_status(payload: dict[str, Any], param_dir: Path | None = None) -> None:
  dest = param_dir or ECOFLOW_PARAM_DIR
  try:
    dest.mkdir(parents=True, exist_ok=True)
    path = dest / STATUS_NAME
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    os.replace(tmp, path)
  except OSError:
    pass


def read_status(param_dir: Path | None = None) -> dict[str, Any]:
  path = overlay_path(STATUS_NAME, param_dir)
  try:
    raw = json.loads(path.read_text(encoding="utf-8"))
  except (OSError, json.JSONDecodeError):
    return {}
  return raw if isinstance(raw, dict) else {}


def status_line(payload: dict[str, Any] | None = None, param_dir: Path | None = None) -> str:
  st = payload if payload is not None else read_status(param_dir)
  if not st:
    return "ecoflowd: no status"
  err = str(st.get("error") or "")
  if err:
    return err[:80]
  if st.get("line"):
    return str(st["line"])[:80]
  parts = []
  parts.append("KL15 on" if st.get("kl15") else "KL15 off")
  if st.get("off_in_s") is not None:
    try:
      parts.append(f"off in {int(st['off_in_s'])}s")
    except (TypeError, ValueError):
      pass
  parts.append("MQTT" if st.get("mqtt") else "no MQTT")
  tel = st.get("telemetry")
  if tel is True:
    parts.append("DC on")
  elif tel is False:
    parts.append("DC off")
  return " · ".join(parts)

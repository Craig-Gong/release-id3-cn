"""IQ.OS 4.9 display: native Adreno raylib, with Mesa fallback."""
from __future__ import annotations

import os
import sys
from pathlib import Path

# magic.py is smooth because IQ.OS /usr/local/venv raylib dlopens
# libEGL_adreno.so. sunnypilot's comma raylib goes through Mesa and
# picks llvmpipe (~2 FPS). Prefer the IQ.OS raylib for the UI process.
#
# Do not set MESA_LOADER_DRIVER_OVERRIDE=msm: InitWindow dies on 4.9.7.
# Comma-raylib + Adreno ICD also dies (boot logo). That path stays opt-in.

_TRYING = Path("/tmp/iqos_adreno_trying")
_FAIL = Path("/data/iqos_adreno_fail")
_NATIVE_TRYING = Path("/tmp/iqos_native_raylib_trying")
_NATIVE_FAIL = Path("/data/iqos_native_raylib_fail")
_UI_MODULE = "openpilot.selfdrive.ui.ui"
_IQOS_SITE = Path("/usr/local/venv/lib/python3.12/site-packages")
_IQOS_RAYLIB_SO = _IQOS_SITE / "raylib/_raylib_cffi.cpython-312-aarch64-linux-gnu.so"


def is_iqos() -> bool:
  try:
    return Path("/VERSION").read_text(encoding="utf-8").startswith("IQ.OS")
  except OSError:
    return False


def note_init_window_ok() -> None:
  for path in (_TRYING, _NATIVE_TRYING):
    try:
      path.unlink(missing_ok=True)
    except OSError:
      pass


def gl_renderer() -> str:
  try:
    import ctypes
    gles = ctypes.CDLL("libGLESv2.so.2")
    gles.glGetString.restype = ctypes.c_char_p
    gles.glGetString.argtypes = [ctypes.c_uint]
    value = gles.glGetString(0x1F01)  # GL_RENDERER
    return (value or b"").decode("utf-8", "replace")
  except OSError:
    return ""


def _strip_abgr_shim() -> None:
  preload = os.environ.get("LD_PRELOAD", "")
  if not preload:
    return
  parts = [p for p in preload.split(":") if p and "libiqos_egl_abgr_compat" not in p]
  if parts:
    os.environ["LD_PRELOAD"] = ":".join(parts)
  else:
    os.environ.pop("LD_PRELOAD", None)


def _mark_fail(path: Path, text: str) -> None:
  try:
    path.write_text(text, encoding="utf-8")
  except OSError:
    pass


def should_use_native_raylib() -> bool:
  if not is_iqos() or os.getenv("IQOS_NATIVE_RAYLIB") == "0":
    return False
  if _NATIVE_FAIL.exists() or not _IQOS_RAYLIB_SO.is_file():
    return False
  return True


def apply_iqos_gl_env() -> None:
  """Must run in the UI process before pyray/libEGL load."""
  if not is_iqos():
    return

  os.environ["LIBGL_ALWAYS_SOFTWARE"] = "0"
  if os.getenv("IQOS_EGL_ADRENO") != "1" or _FAIL.exists():
    return

  already = os.environ.get("IQOS_EGL_ADRENO_ACTIVE") == "1"
  if not already:
    if _TRYING.exists():
      _mark_fail(_FAIL, "InitWindow died with Adreno ICD; staying on Mesa\n")
      try:
        _TRYING.unlink(missing_ok=True)
      except OSError:
        pass
      return
    adreno_so = Path("/usr/lib/aarch64-linux-gnu/libEGL_adreno.so")
    adreno_json = Path(__file__).with_name("egl_adreno.json")
    if not adreno_so.is_file() or not adreno_json.is_file():
      return
    os.environ["IQOS_EGL_ADRENO_ACTIVE"] = "1"
    os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = str(adreno_json.resolve())

  if os.environ.get("IQOS_EGL_REEXEC") == "1" and os.environ.get("IQOS_EGL_ADRENO_ACTIVE") == "1":
    if _TRYING.exists():
      _mark_fail(_FAIL, "InitWindow died with Adreno ICD; staying on Mesa\n")
      try:
        _TRYING.unlink(missing_ok=True)
      except OSError:
        pass
      os.environ.pop("IQOS_EGL_ADRENO_ACTIVE", None)
      os.environ.pop("__EGL_VENDOR_LIBRARY_FILENAMES", None)
      return
    try:
      _TRYING.write_text("1\n", encoding="utf-8")
    except OSError:
      pass


def load_iqos_native_raylib() -> bool:
  """Import IQ.OS raylib/pyray so application.py binds Adreno, not Mesa."""
  if not should_use_native_raylib():
    return False
  if os.environ.get("IQOS_EGL_REEXEC") != "1":
    return False
  if _NATIVE_TRYING.exists():
    _mark_fail(_NATIVE_FAIL, "IQ.OS raylib InitWindow died; using comma Mesa\n")
    try:
      _NATIVE_TRYING.unlink(missing_ok=True)
    except OSError:
      pass
    return False

  site = str(_IQOS_SITE)
  sys.path.insert(0, site)
  try:
    import raylib  # noqa: F401
    import pyray  # noqa: F401
  except Exception:
    if site in sys.path:
      sys.path.remove(site)
    _mark_fail(_NATIVE_FAIL, "IQ.OS raylib import failed; using comma Mesa\n")
    return False
  if site in sys.path:
    sys.path.remove(site)
  try:
    _NATIVE_TRYING.write_text("1\n", encoding="utf-8")
  except OSError:
    pass
  os.environ["IQOS_NATIVE_RAYLIB_ACTIVE"] = "1"
  return True


def reexec_if_needed() -> None:
  """Clean process: drop the ABGR shim before IQ.OS raylib loads Adreno."""
  if not is_iqos():
    return
  if os.environ.get("IQOS_EGL_REEXEC") == "1":
    return
  want_native = should_use_native_raylib()
  want_adreno = os.environ.get("IQOS_EGL_ADRENO_ACTIVE") == "1"
  if not want_native and not want_adreno:
    return
  if want_native:
    _strip_abgr_shim()
  os.environ["IQOS_EGL_REEXEC"] = "1"
  os.execvpe(sys.executable, [sys.executable, "-m", _UI_MODULE], os.environ)

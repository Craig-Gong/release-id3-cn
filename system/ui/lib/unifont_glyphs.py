"""Collect CJK codepoints for the unifont atlas (po + ID.3 UI extras)."""

from __future__ import annotations

import json
from importlib.resources import as_file, files
from pathlib import Path

EXTRA_CHARS = "–‑✓×°§•X⚙✕◀▶✔⌫⇧␣○●↳çêüñ–‑✓×°§•€£¥"
UNIFONT_LANGUAGES = frozenset({"ar", "th", "zh-CHT", "zh-CHS", "ko", "ja"})
_UNIFONT_SIZE = 16
_GLYPH_PADDING = 6


def _fonts_dir() -> Path:
  return Path(str(files("openpilot.selfdrive.assets").joinpath("fonts")))


def _translations_dir() -> Path:
  return Path(str(files("openpilot.selfdrive.ui").joinpath("translations")))


def _load_font_data_arity() -> int:
  import raylib as _raylib

  return len(_raylib.ffi.typeof(_raylib.rl.LoadFontData).args)


def unifont_codepoints() -> tuple[int, ...]:
  chars = set(map(chr, range(32, 127))) | set(EXTRA_CHARS)

  translations_dir = _translations_dir()
  languages_file = translations_dir / "languages.json"
  if languages_file.is_file():
    with languages_file.open(encoding="utf-8") as fh:
      languages = json.load(fh)
    for _language, code in languages.items():
      po_path = translations_dir / f"app_{code}.po"
      try:
        chars.update(po_path.read_text(encoding="utf-8"))
      except FileNotFoundError:
        continue

  id3_path = _fonts_dir() / "id3_ui_cjk.txt"
  if id3_path.is_file():
    for line in id3_path.read_text(encoding="utf-8").splitlines():
      line = line.strip()
      if not line or line.startswith("#"):
        continue
      chars.update(line)

  return tuple(sorted(ord(c) for c in chars if c not in "\n\r"))


def build_unifont_from_otf(otf_path: Path, codepoints: tuple[int, ...] | None = None):
  """Build a runtime Font from unifont.otf + the full IQ zh/atlas codepoint set."""
  import pyray as rl

  if codepoints is None:
    codepoints = unifont_codepoints()

  data = otf_path.read_bytes()
  file_buf = rl.ffi.new("unsigned char[]", data)
  cp_buffer = rl.ffi.new("int[]", codepoints)
  cp_ptr = rl.ffi.cast("int *", cp_buffer)
  args = [
    rl.ffi.cast("unsigned char *", file_buf),
    len(data),
    _UNIFONT_SIZE,
    cp_ptr,
    len(codepoints),
    rl.FontType.FONT_DEFAULT,
  ]
  glyph_count_ptr = None
  if _load_font_data_arity() == 7:
    glyph_count_ptr = rl.ffi.new("int *", 0)
    args.append(glyph_count_ptr)
  glyphs = rl.load_font_data(*args)
  if glyphs == rl.ffi.NULL:
    raise RuntimeError(f"load_font_data failed for {otf_path.name}")

  glyph_count = glyph_count_ptr[0] if glyph_count_ptr is not None else len(codepoints)
  rects_ptr = rl.ffi.new("Rectangle **")
  image = rl.gen_image_font_atlas(glyphs, rects_ptr, glyph_count, _UNIFONT_SIZE, _GLYPH_PADDING, 0)
  if image.width == 0 or image.height == 0:
    rl.unload_image(image)
    raise RuntimeError("gen_image_font_atlas returned empty image")

  font = rl.load_font_from_image(image, rl.Color(255, 0, 255, 255), 32)
  rl.unload_image(image)
  return font


def try_load_expanded_unifont() -> object | None:
  """Load expanded unifont when unifont.otf is present; None → use prebuilt .fnt."""
  with as_file(files("openpilot.selfdrive.assets").joinpath("fonts")) as fonts_path:
    otf_path = Path(fonts_path) / "unifont.otf"
    if not otf_path.is_file():
      return None
    return build_unifont_from_otf(otf_path)

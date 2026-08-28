"""Collect CJK codepoints for the unifont atlas (po + ID.3 UI extras)."""

from __future__ import annotations

import hashlib
import json
from importlib.resources import as_file, files
from pathlib import Path

EXTRA_CHARS = "–‑✓×°§•X⚙✕◀▶✔⌫⇧␣○●↳çêüñ–‑✓×°§•€£¥"
UNIFONT_LANGUAGES = frozenset({"ar", "th", "zh-CHT", "zh-CHS", "ko", "ja"})
_UNIFONT_SIZE = 16
_GLYPH_PADDING = 6
_CACHE_STAMP = "id3_ui_cjk_v3"


def _fonts_dir() -> Path:
  return Path(str(files("openpilot.selfdrive.assets").joinpath("fonts")))


def _translations_dir() -> Path:
  return Path(str(files("openpilot.selfdrive.ui").joinpath("translations")))


def _cache_dir() -> Path:
  for candidate in (
    Path("/data/openpilot/.cache/unifont_expanded"),
    Path("/tmp/openpilot_unifont_cache"),
  ):
    try:
      candidate.mkdir(parents=True, exist_ok=True)
      return candidate
    except OSError:
      continue
  return Path("/tmp/openpilot_unifont_cache")


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
    for language in languages.values():
      chars.update(language)

  for code in UNIFONT_LANGUAGES:
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


def _codepoint_stamp(codepoints: tuple[int, ...]) -> str:
  digest = hashlib.sha256(",".join(map(str, codepoints)).encode()).hexdigest()[:16]
  return f"{_CACHE_STAMP}_{digest}"


def _glyph_metrics(glyphs, rects, glyph_count):
  entries = []
  min_offset_y, max_extent = None, 0
  for idx in range(glyph_count):
    glyph = glyphs[idx]
    rect = rects[idx]
    codepoint = glyph.value
    width = int(round(rect.width))
    height = int(round(rect.height))
    offset_y = int(round(glyph.offsetY))
    min_offset_y = offset_y if min_offset_y is None else min(min_offset_y, offset_y)
    max_extent = max(max_extent, offset_y + height)
    entries.append({
      "id": codepoint,
      "x": int(round(rect.x)),
      "y": int(round(rect.y)),
      "width": width,
      "height": height,
      "xoffset": int(round(glyph.offsetX)),
      "yoffset": offset_y,
      "xadvance": int(round(glyph.advanceX)),
    })

  if min_offset_y is None:
    raise RuntimeError("No glyphs were generated")

  line_height = int(round(max_extent - min_offset_y))
  base = int(round(max_extent))
  return entries, line_height, base


def _write_bmfont(path: Path, font_size: int, face: str, atlas_name: str, line_height: int, base: int, atlas_size, entries):
  if line_height != font_size:
    line_height = font_size
  lines = [
    f"info face=\"{face}\" size=-{font_size} bold=0 italic=0 charset=\"\" unicode=1 stretchH=100 smooth=0 aa=1 padding=0,0,0,0 spacing=0,0 outline=0",
    f"common lineHeight={line_height} base={base} scaleW={atlas_size[0]} scaleH={atlas_size[1]} pages=1 packed=0 alphaChnl=0 redChnl=4 greenChnl=4 blueChnl=4",
    f"page id=0 file=\"{atlas_name}\"",
    f"chars count={len(entries)}",
  ]
  for entry in entries:
    lines.append(
      ("char id={id:<4} x={x:<5} y={y:<5} width={width:<5} height={height:<5} "
       "xoffset={xoffset:<5} yoffset={yoffset:<5} xadvance={xadvance:<5} page=0  chnl=15").format(**entry)
    )
  path.write_text("\n".join(lines) + "\n")


def _export_bmfont_atlas(otf_path: Path, cache_dir: Path, codepoints: tuple[int, ...]) -> Path:
  import pyray as rl

  stamp_path = cache_dir / "stamp.txt"
  fnt_path = cache_dir / "unifont.fnt"
  png_path = cache_dir / "unifont.png"
  stamp = _codepoint_stamp(codepoints)
  if stamp_path.is_file() and stamp_path.read_text().strip() == stamp and fnt_path.is_file() and png_path.is_file():
    return fnt_path

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

  rects = rects_ptr[0]
  entries, line_height, base = _glyph_metrics(glyphs, rects, glyph_count)
  if not rl.export_image(image, png_path.as_posix()):
    rl.unload_image(image)
    raise RuntimeError("Failed to export unifont atlas image")
  rl.unload_image(image)

  _write_bmfont(fnt_path, _UNIFONT_SIZE, "unifont", png_path.name, line_height, base, (image.width, image.height), entries)
  stamp_path.write_text(stamp + "\n")
  return fnt_path


def build_unifont_from_otf(otf_path: Path, codepoints: tuple[int, ...] | None = None):
  """Build/load a bmfont atlas with full unicode mapping (not load_font_from_image)."""
  import pyray as rl

  if codepoints is None:
    codepoints = unifont_codepoints()
  fnt_path = _export_bmfont_atlas(otf_path, _cache_dir(), codepoints)
  return rl.load_font(fnt_path.as_posix())


def try_load_expanded_unifont() -> object | None:
  """Load expanded unifont when unifont.otf is present; None → use prebuilt .fnt."""
  with as_file(files("openpilot.selfdrive.assets").joinpath("fonts")) as fonts_path:
    otf_path = Path(fonts_path) / "unifont.otf"
    if not otf_path.is_file():
      return None
    return build_unifont_from_otf(otf_path)

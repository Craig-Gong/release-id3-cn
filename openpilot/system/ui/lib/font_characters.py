from functools import lru_cache
from importlib.resources import files


@lru_cache(maxsize=1)
def gb2312_characters() -> frozenset[str]:
  """Full GB2312 hanzi (~6763). Covers nearly all Amap road / POI names."""
  chars: set[str] = set()
  for quan in range(0xB0, 0xF8):
    for wei in range(0xA1, 0xFF):
      try:
        chars.add(bytes((quan, wei)).decode("gb2312"))
      except UnicodeDecodeError:
        pass
  return frozenset(chars)


def nav_live_font_chars() -> str:
  """Pull current IQ-link road/POI strings so rare glyphs outside GB2312 still load."""
  try:
    from openpilot.sunnypilot.nav.snapshot import read_snapshot
    snap = read_snapshot()
  except Exception:
    return ""
  parts = [snap.enter_road or "", snap.goal_name or ""]
  return "".join(parts)


def fallback_font_characters(language: str, extra_characters: str = "") -> set[str]:
  """Collect every glyph requested when loading a language fallback font."""
  translations_dir = files("openpilot.selfdrive.ui").joinpath("translations")
  characters = set(map(chr, range(32, 127))) | set(extra_characters)
  characters.update(translations_dir.joinpath(f"app_{language}.po").read_text(encoding="utf-8"))

  # Onroad alerts originate in selfdrived, outside of the normal UI PO extraction.
  from openpilot.selfdrive.ui.onroad.alert_localizer import localized_alert_characters
  characters.update(localized_alert_characters(language))

  # Nav HUD copy (到达提示/…) is drawn via Noto fallback on zh-CHS but is
  # not in app_*.po — without these glyphs Raylib renders '?'.
  try:
    from openpilot.sunnypilot.nav.hud_copy import overlay_font_chars
    characters.update(overlay_font_chars())
  except Exception:
    pass

  # Amap road / destination names are dynamic. The stock Noto SC asset was a
  # ~800-glyph UI subset, so names like 凤山西路 rendered as ???. Ship a full
  # SC face and request GB2312 (+ live shm) so Noto fallback still draws them.
  if language in ("zh-CHS", "zh-CHT"):
    characters.update(gb2312_characters())
    characters.update(nav_live_font_chars())

  characters.difference_update({"\n", "\r", "\t"})
  return characters

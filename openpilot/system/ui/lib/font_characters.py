from importlib.resources import files


def fallback_font_characters(language: str, extra_characters: str = "") -> set[str]:
  """Collect every glyph requested when loading a language fallback font."""
  translations_dir = files("openpilot.selfdrive.ui").joinpath("translations")
  characters = set(map(chr, range(32, 127))) | set(extra_characters)
  characters.update(translations_dir.joinpath(f"app_{language}.po").read_text(encoding="utf-8"))

  # Onroad alerts originate in selfdrived, outside of the normal UI PO extraction.
  from openpilot.selfdrive.ui.onroad.alert_localizer import localized_alert_characters
  characters.update(localized_alert_characters(language))

  # Nav HUD copy (米/公里/到达提示/…) is drawn via Noto fallback on zh-CHS but is
  # not in app_*.po — without these glyphs Raylib renders '?'.
  try:
    from openpilot.sunnypilot.nav.hud_copy import overlay_font_chars
    characters.update(overlay_font_chars())
  except Exception:
    pass

  characters.difference_update({"\n", "\r", "\t"})
  return characters

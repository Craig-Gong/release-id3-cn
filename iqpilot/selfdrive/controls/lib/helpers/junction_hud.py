"""Junction / traffic-stop HUD gate.

C3XL and the MEB cluster share this: show a stop-ahead cue when we are
stopping for a nav light or a vision model stop, and there is no lead.
IQ-link off has no color; IQ-link on may add red/yellow on the C3XL bar.
remainS is never a go gate.
"""

_LIGHTS = ("red", "yellow", "green")


def light_token(raw: str | None) -> str:
  token = str(raw or "none").strip().lower()
  return token if token in _LIGHTS else "none"


def junction_hud_active(*, has_lead: bool, nav_red_decel: bool, stop_light: bool,
                        standstill_hold: bool) -> bool:
  if has_lead:
    return False
  return bool(nav_red_decel or stop_light or standstill_hold)

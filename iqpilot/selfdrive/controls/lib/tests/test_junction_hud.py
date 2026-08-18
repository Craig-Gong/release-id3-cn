from openpilot.iqpilot.selfdrive.controls.lib.helpers.junction_hud import junction_hud_active, light_token


def test_light_token():
  assert light_token("RED") == "red"
  assert light_token("yellow") == "yellow"
  assert light_token("none") == "none"
  assert light_token("") == "none"
  assert light_token("blue") == "none"


def test_junction_hidden_when_lead():
  assert not junction_hud_active(has_lead=True, nav_red_decel=True, stop_light=True, standstill_hold=True)


def test_junction_nav_red_without_iqlink_model():
  assert junction_hud_active(has_lead=False, nav_red_decel=True, stop_light=False, standstill_hold=False)


def test_junction_model_stop_without_nav():
  assert junction_hud_active(has_lead=False, nav_red_decel=False, stop_light=True, standstill_hold=False)


def test_junction_hold_after_stop():
  assert junction_hud_active(has_lead=False, nav_red_decel=False, stop_light=False, standstill_hold=True)


def test_junction_idle():
  assert not junction_hud_active(has_lead=False, nav_red_decel=False, stop_light=False, standstill_hold=False)

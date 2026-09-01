from iqpilot.selfdrive.ui.lib.marquee_text import MarqueeState, advance_horizontal_marquee


def test_marquee_idle_when_text_fits():
  state = MarqueeState()
  assert advance_horizontal_marquee(state, "LTE", 40.0, 120.0, 0.05) == 0.0
  assert state.offset == 0.0


def test_marquee_scrolls_after_pause():
  state = MarqueeState()
  text = "Craig's iPhone"
  for _ in range(40):
    advance_horizontal_marquee(state, text, 220.0, 120.0, 0.05, pause_s=0.2)
  assert state.offset == 0.0
  offset = advance_horizontal_marquee(state, text, 220.0, 120.0, 0.05, pause_s=0.2)
  assert offset > 0.0


def test_marquee_resets_on_text_change():
  state = MarqueeState(text="old", offset=12.0, pause=0.0)
  assert advance_horizontal_marquee(state, "new ssid", 200.0, 100.0, 0.05) == 0.0
  assert state.text == "new ssid"
  assert state.offset == 0.0

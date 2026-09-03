from openpilot.sunnypilot.system.ecoflow.kl15 import chestnut_superspeed_present, meb_ignition_from_can


class _Can:
  def __init__(self, dat, address=0x3C0, src=0):
    self.dat = dat
    self.address = address
    self.src = src


class _Msg:
  def __init__(self, cans):
    self.can = cans


def test_kl15_bit_on():
  dat = bytes([0, 0, 0x02])
  on, last, saw = meb_ignition_from_can([_Msg([_Can(dat)])], now=10.0, last_on_ts=None)
  assert saw and on and last == 10.0


def test_kl15_hold():
  on, last, _ = meb_ignition_from_can([], now=11.5, last_on_ts=10.0)
  assert on
  on, _, _ = meb_ignition_from_can([], now=13.0, last_on_ts=10.0)
  assert on is False


def test_chestnut_probe_does_not_raise():
  assert chestnut_superspeed_present() in (True, False)

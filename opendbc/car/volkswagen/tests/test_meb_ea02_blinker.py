"""EA_02 Force blinker packing must fill COUNTER + VW MEB CHECKSUM."""
from opendbc.can.dbc import DBC
from opendbc.can.packer import CANPacker
from opendbc.car.volkswagen import mebcan
from opendbc.car.volkswagen.mqbcan import volkswagen_mqb_meb_checksum


def test_ea02_dbc_uses_checksum_counter_names():
  dbc = DBC("vw_meb_generated")
  msg = dbc.name_to_msg["EA_02"]
  assert "CHECKSUM" in msg.sigs
  assert "COUNTER" in msg.sigs
  assert "EA_02_CRC" not in msg.sigs
  assert "EA_02_BZ" not in msg.sigs


def test_create_blinker_control_sets_crc_and_blinken():
  packer = CANPacker("vw_meb_generated")
  stock = {
    "EA_Texte": 0,
    "ACF_Lampe_Hands_Off": 0,
    "EA_Infotainment_Anf": 0,
    "EA_Tueren_Anf": 0,
    "EA_Innenraumlicht_Anf": 0,
    "zFAS_Warnblinken": 0,
    "STP_Primaeranz": 0,
    "EA_Bremslichtblinken": 0,
    "EA_Blinken": 0,
    "EA_Unknown": 1,
  }
  addr, dat, bus = mebcan.create_blinker_control(packer, 0, stock, True, False)
  assert addr == 0x1F0
  assert bus == 0
  assert len(dat) == 8
  assert dat != bytes(8)
  assert dat[0] != 0  # CHECKSUM
  # COUNTER in low nibble of byte 1; packer starts at 0
  assert (dat[1] & 0x0F) == 0
  # EA_Blinken at bit 32 → byte 4, 3 bits LE
  assert (dat[4] & 0x07) == 1  # left
  expected = volkswagen_mqb_meb_checksum(0x1F0, None, bytearray(dat))
  assert dat[0] == expected

  _, dat2, _ = mebcan.create_blinker_control(packer, 0, stock, False, True)
  assert (dat2[4] & 0x07) == 2  # right
  assert (dat2[1] & 0x0F) == 1  # counter advanced
  assert dat2[0] == volkswagen_mqb_meb_checksum(0x1F0, None, bytearray(dat2))

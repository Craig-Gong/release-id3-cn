"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""
from iqpilot.system.hardware.egpu_dock.egpu_selftest import egpu_dock_entries, egpu_link_status
from iqpilot.system.hardware.usb import EGPU_DOCK_FW_PRODUCT, EGPU_DOCK_USB_IDS


def test_link_ready():
  vid, pid = EGPU_DOCK_USB_IDS[0]
  devices = [{"vendorId": vid, "productId": pid, "product": EGPU_DOCK_FW_PRODUCT, "speedMbps": 5000}]
  assert egpu_link_status(devices) == "ready"


def test_link_slow_usb():
  vid, pid = EGPU_DOCK_USB_IDS[0]
  devices = [{"vendorId": vid, "productId": pid, "product": EGPU_DOCK_FW_PRODUCT, "speedMbps": 480}]
  assert egpu_link_status(devices).startswith("slow_usb")


def test_entries_filter():
  vid, pid = EGPU_DOCK_USB_IDS[0]
  devices = [
    {"vendorId": vid, "productId": pid, "product": EGPU_DOCK_FW_PRODUCT, "speedMbps": 5000},
    {"vendorId": 0x1234, "productId": 0x5678, "product": "other", "speedMbps": 480},
  ]
  assert len(egpu_dock_entries(devices)) == 1

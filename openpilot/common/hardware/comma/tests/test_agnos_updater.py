import json
import os
import requests

TEST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)))
MANIFEST = os.path.join(TEST_DIR, "../agnos.json")


from openpilot.common.test import OpenpilotTestCase
class TestAgnosUpdater(OpenpilotTestCase):

  def test_manifest(self):
    with open(MANIFEST) as f:
      m = json.load(f)

    for img in m:
      r = requests.head(img['url'], timeout=10)
      r.raise_for_status()
      assert r.headers['Content-Type'] == "application/x-xz"
      if not img['sparse']:
        assert img['hash'] == img['hash_raw']

  def test_c3xl_boot_chain_matches_allowlist(self):
    from openpilot.sunnypilot.hardware.agnos import validate_agnos_manifest
    from openpilot.sunnypilot.hardware.profile import HardwareProfile

    with open(MANIFEST) as f:
      m = json.load(f)

    validate_agnos_manifest(m, profile=HardwareProfile.C3XL)

  def test_c3xl_rejects_official_196_abl(self):
    from openpilot.sunnypilot.hardware.agnos import UnsafeBootChainManifest, validate_agnos_manifest
    from openpilot.sunnypilot.hardware.profile import HardwareProfile

    with open(MANIFEST) as f:
      m = json.load(f)
    for img in m:
      if img["name"] == "abl":
        img["hash"] = "29fd7ed1c012e599420764840f9f11286d34dbff4adaf102a447f06d8c5e0b35"
        img["hash_raw"] = img["hash"]
        break

    with self.assertRaises(UnsafeBootChainManifest):
      validate_agnos_manifest(m, profile=HardwareProfile.C3XL)

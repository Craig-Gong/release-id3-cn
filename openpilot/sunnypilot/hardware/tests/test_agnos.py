import json
import os
from pathlib import Path
import subprocess

import pytest

from openpilot.sunnypilot.hardware.agnos import UnsafeBootChainManifest, validate_agnos_manifest
from openpilot.sunnypilot.hardware.profile import HardwareProfile


REPO_ROOT = Path(__file__).parents[4]
STANDARD_MANIFEST = Path(__file__).parents[3] / "common/hardware/comma/agnos.json"
C3XL_MANIFEST = Path(__file__).parents[3] / "common/hardware/comma/agnos-c3xl.json"


def read_manifest(path: Path = C3XL_MANIFEST) -> list[dict]:
  with path.open() as manifest_file:
    return json.load(manifest_file)


def test_c3xl_manifest_matches_allowlist() -> None:
  validate_agnos_manifest(read_manifest(), HardwareProfile.C3XL)


@pytest.mark.parametrize("name", ["xbl", "xbl_config", "abl", "aop", "devcfg", "boot"])
def test_c3xl_rejects_changed_boot_chain_image(name: str) -> None:
  manifest = read_manifest()
  partition = next(partition for partition in manifest if partition["name"] == name)
  partition["hash"] = "0" * 64

  with pytest.raises(UnsafeBootChainManifest, match=name):
    validate_agnos_manifest(manifest, HardwareProfile.C3XL)


def test_standard_profile_does_not_apply_c3xl_allowlist() -> None:
  manifest = read_manifest(STANDARD_MANIFEST)
  next(partition for partition in manifest if partition["name"] == "boot")["hash"] = "0" * 64
  validate_agnos_manifest(manifest, HardwareProfile.STANDARD)


def test_standard_manifest_is_official_agnos_19_7() -> None:
  partitions = {partition["name"]: partition for partition in read_manifest(STANDARD_MANIFEST)}
  assert partitions["abl"]["hash"] == "29fd7ed1c012e599420764840f9f11286d34dbff4adaf102a447f06d8c5e0b35"
  assert partitions["boot"]["hash"] == "6ecf6f987cd11968104abcccabbe268485d329cdb73012dfd3c381a6b8deb27d"
  assert partitions["system"]["hash_raw"] == "3c271e2b3d20d2f0a8bf6555a1319f3efb12845490967d6151195174a01e912f"


def test_c3xl_manifest_uses_official_19_7_system_with_validated_boot_chain() -> None:
  partitions = {partition["name"]: partition for partition in read_manifest(C3XL_MANIFEST)}
  assert partitions["boot"]["hash"] == "0191529aa97d90d1fa04b472d80230b777606459e1e1e9e2323c9519839827b4"
  assert partitions["system"]["hash_raw"] == "3c271e2b3d20d2f0a8bf6555a1319f3efb12845490967d6151195174a01e912f"


@pytest.mark.parametrize("profile,version,manifest", [
  ("standard", "19.7", "openpilot/system/hardware/comma/agnos.json"),
  ("c3xl", "19.7", "openpilot/system/hardware/comma/agnos-c3xl.json"),
])
def test_launch_environment_selects_profile_specific_agnos(profile: str, version: str, manifest: str) -> None:
  env = os.environ.copy()
  env["SUNNYPILOT_HARDWARE_PROFILE"] = profile
  output = subprocess.check_output(
    ["bash", "-c", "unset AGNOS_VERSION AGNOS_MANIFEST_FILE; source launch_env.sh; printf '%s\\n%s\\n' \"$AGNOS_VERSION\" \"$AGNOS_MANIFEST_FILE\""],
    cwd=REPO_ROOT, env=env, text=True,
  ).splitlines()
  assert output == [version, manifest]

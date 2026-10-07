import json
from Crypto.PublicKey import RSA
from pathlib import Path

from openpilot.common.test import OpenpilotTestCase
from openpilot.common.params import Params
from openpilot.system.athena.registration import register, UNREGISTERED_DONGLE_ID
from openpilot.system.athena.tests.helpers import MockResponse
from openpilot.common.hardware.hw import Paths


class TestRegistration(OpenpilotTestCase):

  def setup_method(self):
    # clear params and setup key paths
    self.params = Params()

    persist_dir = Path(Paths.persist_root()) / "comma"
    persist_dir.mkdir(parents=True, exist_ok=True)

    self.priv_key = persist_dir / "id_rsa"
    self.pub_key = persist_dir / "id_rsa.pub"
    self.dongle_id = persist_dir / "dongle_id"

  def _generate_keys(self):
    self.pub_key.touch()
    k = RSA.generate(2048)
    with open(self.priv_key, "wb") as f:
      f.write(k.export_key())
    with open(self.pub_key, "wb") as f:
      f.write(k.publickey().export_key())

  def test_valid_cache(self, mocker):
    # if all params are written, return the cached dongle id.
    # should work with a dongle ID on either /persist/ or normal params
    self._generate_keys()

    dongle = "DONGLE_ID_123"
    m = mocker.patch("openpilot.system.athena.registration.api_get", autospec=True)
    for persist, params in [(True, True), (True, False), (False, True)]:
      self.params.put("DongleId", dongle if params else "", block=True)
      with open(self.dongle_id, "w") as f:
        f.write(dongle if persist else "")
      assert register() == dongle
      assert not m.called

  def test_no_keys(self, mocker):
    # missing pubkey
    m = mocker.patch("openpilot.system.athena.registration.api_get", autospec=True)
    dongle = register()
    assert m.call_count == 0
    assert dongle == UNREGISTERED_DONGLE_ID
    assert self.params.get("DongleId") == dongle

  def test_missing_cache(self, mocker):
    # keys exist but no dongle id
    self._generate_keys()
    m = mocker.patch("openpilot.system.athena.registration.api_get", autospec=True)
    dongle = "DONGLE_ID_123"
    m.return_value = MockResponse(json.dumps({'dongle_id': dongle}), 200)
    assert register() == dongle
    assert m.call_count == 1

    # call again, shouldn't hit the API this time
    assert register() == dongle
    assert m.call_count == 1
    assert self.params.get("DongleId") == dongle

  def test_unregistered(self, mocker):
    # keys exist, but unregistered
    self._generate_keys()
    m = mocker.patch("openpilot.system.athena.registration.api_get", autospec=True)
    m.return_value = MockResponse(None, 402)
    dongle = register()
    assert m.call_count == 1
    assert dongle == UNREGISTERED_DONGLE_ID
    assert self.params.get("DongleId") == dongle

  def test_missing_imei_does_not_block_fresh_install(self, mocker):
    self._generate_keys()
    mocker.patch("openpilot.system.athena.registration.HARDWARE.get_imei", return_value=None)
    clock = mocker.patch("openpilot.system.athena.registration.time.monotonic", side_effect=range(0, 200, 5))
    mocker.patch("openpilot.system.athena.registration.time.sleep")
    api = mocker.patch("openpilot.system.athena.registration.api_get")
    api.return_value = MockResponse(json.dumps({'dongle_id': 'RECOVERED'}), 200)
    assert register() == 'RECOVERED'
    assert clock.call_count < 10

  def test_network_outage_is_bounded_without_spinner_and_retries_next_boot(self, mocker):
    self._generate_keys()
    mocker.patch("openpilot.system.athena.registration.HARDWARE.get_imei", return_value='')
    mocker.patch("openpilot.system.athena.registration.time.monotonic", side_effect=range(0, 1000, 10))
    mocker.patch("openpilot.system.athena.registration.time.sleep")
    api = mocker.patch("openpilot.system.athena.registration.api_get", side_effect=TimeoutError('offline'))
    assert register() == UNREGISTERED_DONGLE_ID
    assert api.call_count <= 3
    api.side_effect = None
    api.return_value = MockResponse(json.dumps({'dongle_id': 'RECOVERED'}), 200)
    assert register() == 'RECOVERED'

  def test_registration_timeout_always_closes_spinner(self, mocker):
    self._generate_keys()
    mocker.patch("openpilot.system.athena.registration.HARDWARE.get_imei", return_value='')
    mocker.patch("openpilot.system.athena.registration.time.monotonic", side_effect=range(0, 1000, 10))
    mocker.patch("openpilot.system.athena.registration.time.sleep")
    mocker.patch("openpilot.system.athena.registration.api_get", side_effect=TimeoutError('offline'))
    spinner = mocker.patch("openpilot.system.athena.registration.Spinner")
    assert register(show_spinner=True) == UNREGISTERED_DONGLE_ID
    spinner.return_value.close.assert_called_once()

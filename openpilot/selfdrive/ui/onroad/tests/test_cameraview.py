from unittest.mock import MagicMock

from openpilot.cereal.visionipc import VisionStreamType
import openpilot.selfdrive.ui.onroad.cameraview as cameraview


class TestCameraViewRecovery:
  def test_onroad_transition_replaces_stale_client_without_a_frame(self, monkeypatch):
    view = object.__new__(cameraview.CameraView)
    old_client = MagicMock()
    new_client = MagicMock()
    view.client = old_client
    view.frame = None
    view.available_streams = []
    view._name = "camerad"
    view._stream_type = VisionStreamType.VISION_STREAM_NARROW_ROAD
    view._clear_textures = MagicMock()
    view.close = MagicMock()
    monkeypatch.setattr(cameraview.ui_state, "is_onroad", lambda: True)
    client_ctor = MagicMock(return_value=new_client)
    monkeypatch.setattr(cameraview, "VisionIpcClient", client_ctor)

    view._offroad_transition()

    view._clear_textures.assert_called_once_with()
    client_ctor.assert_called_once_with("camerad", VisionStreamType.VISION_STREAM_NARROW_ROAD, conflate=True)
    assert view.client is new_client

  def test_invalid_frame_fd_reconnects_before_egl_import(self, monkeypatch):
    view = object.__new__(cameraview.CameraView)
    view.frame = MagicMock(fd=-1)
    view.egl_texture = MagicMock()
    view._reconnect = MagicMock()
    view.close = MagicMock()
    create_egl_image = MagicMock()
    monkeypatch.setattr(cameraview, "create_egl_image", create_egl_image)

    view._render_egl(MagicMock(), MagicMock())

    view._reconnect.assert_called_once_with()
    create_egl_image.assert_not_called()

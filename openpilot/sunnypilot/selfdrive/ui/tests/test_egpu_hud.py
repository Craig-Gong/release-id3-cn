#!/usr/bin/env python3
import unittest

from openpilot.sunnypilot.selfdrive.ui.egpu_hud import build_hud_egpu_view, dc_chip_from_label


def _hud(**kwargs):
  defaults = dict(
    engaged=True, connected=True, compiled=True, loading=False, active=True,
    model_alive=True, model_big=True, telemetry_valid=True, usb_speed_mbps=5000,
    model_fps=19.8, power_w=72.0, temp_c=61.0,
    memory_used_mb=6144, memory_total_mb=8192, dc_label="12V 开",
  )
  defaults.update(kwargs)
  return build_hud_egpu_view(**defaults)


class TestHudEgpuView(unittest.TestCase):
  def test_hidden_when_not_engaged(self):
    self.assertFalse(_hud(engaged=False).show)

  def test_running_uses_tiles_and_dc_chip(self):
    view = _hud()
    self.assertTrue(view.show)
    self.assertTrue(view.healthy)
    self.assertEqual(view.headline, "大模型")
    self.assertEqual(view.detail, "运行中")
    self.assertEqual(view.dc_text, "开")
    self.assertEqual(view.dc_kind, "on")
    self.assertEqual([m.unit for m in view.metrics], ["FPS", "W", "GPU", "GB"])
    self.assertEqual(view.metrics[0].value, "20")
    self.assertEqual(view.metrics[1].value, "72")
    self.assertEqual(view.metrics[2].value, "61°")
    self.assertEqual(view.metrics[3].value, "6.0/8.0")

  def test_disconnected_keeps_ecoflow_chip(self):
    view = _hud(connected=False, telemetry_valid=False)
    self.assertEqual(view.headline, "eGPU")
    self.assertEqual(view.detail, "未连接")
    self.assertEqual(view.dc_text, "开")
    self.assertEqual(view.metrics, ())
    self.assertEqual(view.severity, "muted")

  def test_fallback_and_loading(self):
    fallback = _hud(active=False, model_big=False, telemetry_valid=False)
    self.assertEqual(fallback.headline, "大模型")
    self.assertEqual(fallback.detail, "已回退小模型")
    self.assertEqual(fallback.severity, "danger")
    loading = _hud(loading=True, active=None, model_alive=False, model_big=False,
                   telemetry_valid=False, loading_progress=64)
    self.assertEqual(loading.detail, "加载 64%")

  def test_usb_degraded_still_shows_gpu_metrics_when_running(self):
    view = _hud(usb_speed_mbps=480)
    self.assertEqual(view.headline, "USB")
    self.assertEqual(view.detail, "未 SuperSpeed")
    self.assertFalse(view.healthy)
    self.assertEqual(view.metrics[0].value, "20")
    self.assertEqual(view.metrics[2].unit, "GPU")
    self.assertEqual(view.metrics[3].unit, "GB")

  def test_dc_chip_from_label(self):
    self.assertEqual(dc_chip_from_label("12V 开"), ("开", "on"))
    self.assertEqual(dc_chip_from_label("12V 关"), ("关", "off"))
    self.assertEqual(dc_chip_from_label("12V 未启用"), ("未启用", "idle"))
    self.assertEqual(dc_chip_from_label("12V 未知"), ("未知", "unknown"))


if __name__ == "__main__":
  unittest.main()

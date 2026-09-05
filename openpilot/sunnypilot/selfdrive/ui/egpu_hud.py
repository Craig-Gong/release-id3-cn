"""C3XL onroad eGPU strip copy. Keep this free of pyray so tests can import it."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class HudEgpuMetric:
  value: str
  unit: str


@dataclass(frozen=True)
class HudEgpuView:
  """Status header + 12V chip; metrics as labeled tiles when the big model is live."""
  show: bool = False
  healthy: bool = False
  severity: str = "muted"  # good | warning | danger | muted
  headline: str = ""
  detail: str = ""
  dc_text: str = "未知"
  dc_kind: str = "unknown"  # on | off | idle | unknown
  metrics: tuple[HudEgpuMetric, ...] = ()


def dc_chip_from_label(dc_label: str) -> tuple[str, str]:
  text = (dc_label or "").replace("12V", "").strip()
  if text == "开":
    return "开", "on"
  if text == "关":
    return "关", "off"
  if text == "未启用":
    return "未启用", "idle"
  return "未知", "unknown"


def build_hud_egpu_view(*, onroad: bool, connected: bool, compiled: bool, loading: bool,
                        active: bool | None, model_alive: bool, model_big: bool,
                        telemetry_valid: bool, loading_progress: int = 0,
                        usb_speed_mbps: int = 0, model_fps: float = 0.0,
                        power_w: float = 0.0, temp_c: float = 0.0,
                        memory_used_mb: int = 0, memory_total_mb: int = 0,
                        dc_label: str = "12V 未知") -> HudEgpuView:
  if not onroad:
    return HudEgpuView()

  degraded = connected and 0 < usb_speed_mbps < 5000
  running = bool(connected and compiled and not loading and active is True and model_alive and model_big)
  fallback = bool(connected and compiled and not loading and (
    active is False or (active is True and (not model_alive or not model_big))
  ))
  dc_text, dc_kind = dc_chip_from_label(dc_label)

  if not connected:
    headline, detail, severity, healthy = "eGPU", "未连接", "muted", False
  elif not compiled:
    headline, detail, severity, healthy = "大模型", "未编译", "warning", False
  elif loading:
    headline, detail, severity, healthy = "大模型", f"加载 {int(loading_progress)}%", "warning", False
  elif fallback:
    headline, detail, severity, healthy = "大模型", "已回退小模型", "danger", False
  elif active is not True:
    headline, detail, severity, healthy = "大模型", "等待启动", "warning", False
  elif degraded:
    headline, detail, severity, healthy = "USB", "未 SuperSpeed", "danger", False
  else:
    headline, detail, severity, healthy = "大模型", "运行中", "good", True

  metrics: tuple[HudEgpuMetric, ...] = ()
  if running and telemetry_valid:
    tiles = [
      HudEgpuMetric(f"{model_fps:.0f}", "FPS"),
      HudEgpuMetric(f"{power_w:.0f}", "W"),
      HudEgpuMetric(f"{temp_c:.0f}°", "GPU"),
    ]
    if memory_total_mb > 0:
      tiles.append(HudEgpuMetric(
        f"{memory_used_mb / 1024.0:.1f}/{memory_total_mb / 1024.0:.1f}", "GB",
      ))
    metrics = tuple(tiles)
  elif running:
    detail = "遥测暂无" if detail == "运行中" else f"{detail} · 遥测暂无"

  return HudEgpuView(
    show=True, healthy=healthy, severity=severity,
    headline=headline, detail=detail, dc_text=dc_text, dc_kind=dc_kind,
    metrics=metrics,
  )

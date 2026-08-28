"""Shared junction / traffic-light HUD state for camera overlay."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class JunctionHudSnapshot:
  active: bool
  light: str  # "red", "yellow", or "none"
  dist_m: float
  remain_s: float

  @property
  def headline(self) -> str:
    if self.light == "red":
      return "红灯"
    if self.light == "yellow":
      return "黄灯"
    return "前方停车"

  @property
  def detail(self) -> str:
    parts: list[str] = []
    if self.dist_m >= 1.0:
      parts.append(f"{int(round(self.dist_m))} 米")
    if self.remain_s >= 1.0:
      parts.append(f"{int(self.remain_s)} 秒")
    return " · ".join(parts)


def read_junction_snapshot(sm, *, engaged: bool) -> JunctionHudSnapshot:
  inactive = JunctionHudSnapshot(False, "none", 0.0, 0.0)
  if not engaged:
    return inactive
  try:
    if not bool(sm["iqPlan"].e2eAlerts.junctionStop):
      return inactive
  except Exception:
    return inactive

  light = "none"
  dist_m = 0.0
  remain_s = 0.0
  try:
    nav = sm["iqNavState"]
    token = str(getattr(nav, "trafficLight", "none") or "none").strip().lower()
    if token in ("red", "yellow"):
      light = token
    dist_m = float(getattr(nav, "trafficLightDistM", 0.0) or 0.0)
    remain_s = float(getattr(nav, "trafficLightRemainS", 0.0) or 0.0)
  except Exception:
    pass
  return JunctionHudSnapshot(True, light, dist_m, remain_s)


def junction_accent_rgb(light: str) -> tuple[int, int, int]:
  if light == "red":
    return (210, 48, 52)
  if light == "yellow":
    return (214, 168, 24)
  return (176, 186, 198)

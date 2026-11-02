#!/usr/bin/env python3
"""Static flash-readiness checks for the ID.3 `dev-sp` worktree (no cereal)."""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _read(rel: str) -> str:
  return (ROOT / rel).read_text(encoding="utf-8")


def main() -> int:
  fails: list[str] = []

  def ok(cond: bool, msg: str) -> None:
    if not cond:
      fails.append(msg)

  params = _read("openpilot/common/params_keys.h")
  ok(
    re.search(r'\{"LongitudinalPlannerMode".*"0"\}', params) is not None
    or '{"LongitudinalPlannerMode", {PERSISTENT | BACKUP, INT, "0"}}' in params,
    "LongitudinalPlannerMode default must be 0 (Official)",
  )

  official = _read("openpilot/selfdrive/controls/lib/longitudinal_planner.py")
  ok("enable_dec=False" in official, "Official planner must pass enable_dec=False")
  ok("apply_stop_helpers" in official, "Official planner must call apply_stop_helpers")
  ok("get_empty_road_max_accel" in official, "Official planner must keep empty-road soft cap")
  ok("AccelBoost" in official, "Official planner must keep AccelBoost")

  legacy = _read(
    "openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_backends/legacy_mpc/planner.py"
  )
  ok("apply_stop_helpers" in legacy, "Legacy planner must call apply_stop_helpers")

  session = _read(
    "openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_backends/session.py"
  )
  ok("_prefer_official" in session, "session must heal Experimental→Official")
  ok("BackendId.EXPERIMENTAL" in session, "heal must key off EXPERIMENTAL only")

  events = _read("openpilot/selfdrive/selfdrived/events.py")
  ok(
    "EventName.bigModelFailed" in events and "SOFT_DISABLE" not in events[
      events.index("EventName.bigModelFailed") : events.index("EventName.bigModelFailed") + 180
    ],
    "bigModelFailed must stay toast/permanent-only (no SOFT_DISABLE)",
  )

  pc = _read("openpilot/system/manager/process_config.py")
  ok('PythonProcess("iqlinkd"' in pc, "iqlinkd process missing")
  ok('PythonProcess("ecoflowd"' in pc, "ecoflowd process missing")
  ok("traffic_control_needed" in pc and "return False" in pc.split("def traffic_control_needed", 1)[1][:220],
     "trafficcontrold must stay disabled")

  modeld = _read("openpilot/sunnypilot/modeld_v2/modeld.py")
  ok("AM_POWER_LIMIT" in modeld and "'100'" in modeld, "C3XL PPT100 setdefault missing in modeld_v2")

  pandad = _read("openpilot/selfdrive/pandad/pandad.py")
  ok("skip DFU recover" in pandad, "pandad must skip DFU recover on internal panda")
  ok("PandaDFU.list()" in pandad and "skip recover on C3XL" in pandad,
     "pandad must not recover DFU pandas on C3XL")

  # tip once shipped a C3XL path that GPIO-reset every prepare and DFU-recovered
  # after empty-list timeout — that bricks the patched internal tres.
  panda_startup = _read("openpilot/sunnypilot/hardware/panda_startup.py")
  ok("C3XL_EMPTY_WAIT" in panda_startup, "panda_startup must keep C3XL empty-wait (no DFU)")
  ok(
    panda_startup.count("self.io.recover_internal()") == 1,
    "panda_startup must call recover_internal only on non-C3XL",
  )
  # C3XL body starts after the non-C3XL early-return; must not reset/recover there.
  c3xl_body = panda_startup.split("return PandaStartupResult.RECOVER", 1)[-1]
  ok(
    "self.io.recover_internal()" not in c3xl_body and "self.io.reset_internal()" not in c3xl_body,
    "C3XL panda_startup must not GPIO-reset or DFU-recover",
  )

  for rel in (
    "openpilot/sunnypilot/nav/iqlinkd.py",
    "openpilot/sunnypilot/nav/udp_nav.py",
    "openpilot/sunnypilot/selfdrive/controls/lib/helpers/unprotected_turn.py",
    "openpilot/selfdrive/ui/egpu_status.py",
    "openpilot/selfdrive/ui/sunnypilot/onroad/junction_hud.py",
    "opendbc_repo/opendbc/car/volkswagen/mebcan.py",
  ):
    ok((ROOT / rel).is_file(), f"missing required port file: {rel}")

  values = _read("opendbc_repo/opendbc/car/volkswagen/values.py")
  ok("VOLKSWAGEN_ID3_MK1" in values or "ID3" in values, "ID.3 platform missing in values.py")

  meb = _read("opendbc_repo/opendbc/car/volkswagen/mebcan.py")
  ok("meb_pid_hold_should_start" in meb, "meb_pid_hold_should_start missing")

  cruise_ext = _read("openpilot/sunnypilot/selfdrive/car/cruise_ext.py")
  ok("_apply_iqlink_nav_to_max" in cruise_ext, "IQ-link MAX follow missing in cruise_ext")
  ok("nav_lc_swallow_set" in cruise_ext, "Nav LC SET swallow missing in cruise_ext")

  car_events = _read("openpilot/selfdrive/car/car_events.py")
  ok("nav_lc_swallow_set" in car_events, "Nav LC SET swallow missing in car_events")

  sla = _read("openpilot/sunnypilot/selfdrive/controls/lib/speed_limit/speed_limit_assist.py")
  ok("nav_lc_swallow_set" in sla, "Nav LC SET swallow missing in SLA")

  cruise = _read("openpilot/selfdrive/car/cruise.py")
  ok("volkswagen_standby_set_speed" in cruise, "VW SET=current-speed path missing")
  ok("_sync_v_cruise_from_gas" in cruise, "Gas Sync missing in cruise.py")
  ok("mark_iqlink_cruise_override" in cruise, "IQ-link MAX override latch missing in cruise.py")

  params_py = _read("openpilot/common/params.py")
  ok("_PREBUILT_KEY_DEFAULTS" in params_py and "_FILE_BACKED_PARAMS" in params_py,
     "params.py prebuilt/file-backed rsync fallbacks missing")

  if fails:
    print("PREFLIGHT FAIL")
    for f in fails:
      print(" -", f)
    return 1
  print("PREFLIGHT OK")
  return 0


if __name__ == "__main__":
  sys.exit(main())

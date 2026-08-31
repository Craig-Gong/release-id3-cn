"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
import os
import shutil
import subprocess
import time
from pathlib import Path

import pytest

import iqpilot.cereal.messaging as messaging
from iqpilot.cereal import log

TOOLS_DIR = Path(__file__).parent
CABANA_BIN = TOOLS_DIR / "cabana" / "_cabana"
JOTPLUGGLER_BIN = TOOLS_DIR / "jotpluggler" / "jotpluggler"

DONGLE_ID = "0000000000000000"
TIMESTAMP = "2024-01-01--00-00-00"
ROUTE = f"{DONGLE_ID}|{TIMESTAMP}"


def write_rlog(path: Path, n_frames: int = 200):
  with open(path, "wb") as f:
    cp = messaging.new_message('carParams')
    cp.carParams.carFingerprint = "TOYOTA_RAV4_TSS2"
    cp.carParams.brand = "toyota"
    f.write(cp.to_bytes())

    for i in range(n_frames):
      msg = messaging.new_message('can', 2)
      msg.logMonoTime = int(i * 1e7)
      for j, addr in enumerate((0x1D2, 0x260)):
        msg.can[j].address = addr
        msg.can[j].src = 0
        msg.can[j].dat = bytes([i % 256] * 8)
      f.write(msg.to_bytes())


@pytest.fixture(scope="module")
def local_route(tmp_path_factory):
  data_dir = tmp_path_factory.mktemp("routes")
  for seg in range(2):
    seg_dir = data_dir / f"{DONGLE_ID}|{TIMESTAMP}--{seg}"
    seg_dir.mkdir()
    write_rlog(seg_dir / "rlog")
  return data_dir


def run(cmd, timeout=180):
  env = {**os.environ, "QT_QPA_PLATFORM": "offscreen"}
  return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=env,
                        cwd=TOOLS_DIR.parent)


def test_jotpluggler_renders_a_local_route(local_route, tmp_path):
  assert JOTPLUGGLER_BIN.exists(), "jotpluggler not built"
  out = tmp_path / "plot.png"
  result = run([str(JOTPLUGGLER_BIN), "--data-dir", str(local_route),
                "--sync-load", "--output", str(out), ROUTE])
  assert result.returncode == 0, result.stdout + result.stderr
  assert out.is_file(), result.stdout + result.stderr
  assert out.stat().st_size > 5000, f"suspiciously small render: {out.stat().st_size} bytes"


def test_cabana_loads_a_local_route(local_route):
  assert CABANA_BIN.exists(), "cabana not built"
  env = {**os.environ, "QT_QPA_PLATFORM": "offscreen"}
  proc = subprocess.Popen([str(CABANA_BIN), "--data_dir", str(local_route), "--no-vipc", ROUTE],
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                          env=env, cwd=TOOLS_DIR.parent)
  loaded = f"loaded route {ROUTE} with 2 valid segments"
  lines = []
  deadline = time.monotonic() + 60
  try:
    while time.monotonic() < deadline:
      line = proc.stdout.readline()
      if not line:
        break
      lines.append(line)
      if loaded in line:
        break
  finally:
    proc.kill()
    proc.wait()
    proc.stdout.close()

  out = "".join(lines)
  assert "failed to load route" not in out, out
  assert "invalid route format" not in out, out
  assert loaded in out, out


def test_replay_logreader_reports_load_stats(local_route):
  assert shutil.which("python3") is not None
  header = (TOOLS_DIR / "replay" / "logreader.h").read_text()
  for accessor in ("compressed_size", "decompressed_size", "download_seconds",
                   "decompress_seconds", "parse_seconds"):
    assert f"{accessor}() const" in header


def test_can_capnp_field_matches_extractor_codegen():
  assert 'busTimeDEPRECATED' in log.CanData.schema.fields
  gen = (TOOLS_DIR / "jotpluggler" / "generate_event_extractors.py").read_text()
  assert "getBusTimeDEPRECATED()" in gen

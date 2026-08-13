#!/bin/bash
set -e
cd /data/openpilot
echo '=== MEB hold-release ==='
PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 -m pytest \
  iqdbc_repo/iqdbc/car/volkswagen/tests/test_meb_acc_hold_release.py \
  iqdbc_repo/iqdbc/car/volkswagen/tests/test_meb_starting.py \
  -o addopts='' -v
echo '=== mapd bridge self-check ==='
PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 iqpilot/iq_maps/road_data/test_mapd_live_bridge_limits.py
echo '=== procs ==='
ps -ef | grep -E 'mapd|mapd_live|iqmapd' | grep -v grep || echo '(no mapd procs — expected offroad)'
echo '=== speed A probe ==='
PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 iqpilot/iqlink/tools/_probe_speed_a.py

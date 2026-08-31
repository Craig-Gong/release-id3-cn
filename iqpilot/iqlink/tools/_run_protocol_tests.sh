#!/bin/bash
set -e
cd /data/openpilot
echo '=== protocol tests ==='
PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 -m pytest \
  iqpilot/iqlink/tests/test_protocol.py -o addopts='' -v
echo '=== link ==='
PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 iqpilot/iqlink/tools/_link_state.py

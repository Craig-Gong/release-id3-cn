#!/usr/bin/env bash
# 2A: stash dirty tree, fetch iqlink from local bundle, checkout, restart manager.
set -euo pipefail
cd /data/openpilot

BUNDLE="${1:-/data/iqlink_2a.bundle}"
STASH_MSG="2a-pre-iqlink-$(date +%Y%m%d-%H%M%S)"

echo "== pre =="
echo "HEAD=$(git rev-parse --short HEAD) branch=$(git branch --show-current)"
git status -sb | head -5
test -f "$BUNDLE" || { echo "missing bundle: $BUNDLE"; exit 1; }
git cat-file -t f004daa32ef02432b740953602eece68e7767c03 >/dev/null

echo "== stash (incl untracked) =="
# Include untracked hot files; keep ignored (venv/models) alone.
git stash push -u -m "$STASH_MSG"
echo "stash_ok msg=$STASH_MSG"
git status -sb | head -5

echo "== fetch iqlink from bundle =="
git fetch "$BUNDLE" refs/heads/iqlink:refs/heads/iqlink
git rev-parse --short iqlink

echo "== checkout iqlink =="
git checkout iqlink
echo "HEAD=$(git rev-parse --short HEAD) branch=$(git branch --show-current)"
git status -sb | head -20

echo "== restart manager (comma.service) =="
# Soft relaunch so forked Python picks up new modules.
if systemctl is-active --quiet comma.service 2>/dev/null; then
  sudo systemctl restart comma.service
  echo "comma.service restarted"
else
  # Fallback: manager restart helper if present
  if [ -x /data/openpilot/system/manager/manager.py ] || [ -f /data/openpilot/system/manager/manager.py ]; then
    sudo systemctl restart comma 2>/dev/null || sudo service comma restart 2>/dev/null || true
  fi
  echo "restart_attempted"
fi

sleep 3
echo "== post =="
echo "HEAD=$(git rev-parse --short HEAD) branch=$(git branch --show-current)"
pgrep -af 'bridge|manager|selfdrive' | head -20 || true
echo "DONE"

#!/usr/bin/env bash
# Fetch Xiaoge lane.onnx (~13 MB) to the C3XL models dir. Do NOT commit the weight.
set -euo pipefail

DEST="${1:-/data/media/0/models/lane.onnx}"
# Nested openpilot/ path on ajouatom carrot-wip. Prefer jsDelivr (raw often times out in CN).
URLS=(
  "https://cdn.jsdelivr.net/gh/ajouatom/openpilot@carrot-wip/openpilot/selfdrive/carrot/xiaoge/assets/lane.onnx"
  "https://raw.githubusercontent.com/ajouatom/openpilot/carrot-wip/openpilot/selfdrive/carrot/xiaoge/assets/lane.onnx"
)

mkdir -p "$(dirname "$DEST")"
TMP="${DEST}.tmp.$$"

for url in "${URLS[@]}"; do
  echo "Trying: $url"
  if curl -fL --connect-timeout 20 --max-time 600 -o "$TMP" "$url"; then
    SIZE=$(wc -c <"$TMP" | tr -d ' ')
    if [[ "$SIZE" -lt 1000000 ]]; then
      echo "Too small ($SIZE bytes), skip"
      rm -f "$TMP"
      continue
    fi
    mv "$TMP" "$DEST"
    echo "OK → $DEST ($SIZE bytes)"
    exit 0
  fi
  rm -f "$TMP"
done

echo "Failed to download lane.onnx from all mirrors" >&2
exit 1

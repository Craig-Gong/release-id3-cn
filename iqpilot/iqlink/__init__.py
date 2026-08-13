"""iqlink: CP搭子 / Carrot-compatible nav bridge for IQ.Pilot (BLE-only)."""

# Deprecated WiFi ports (removed from runtime; see PROTOCOL.md):
# DISCOVERY_PORT=7705, NAVI_UDP_PORT=7706, NAVI_HTTP_PORT=7713

DEFAULT_WARN_TIMEOUT_S = 5.0
DEFAULT_CANCEL_TIMEOUT_S = 10.0  # no-write soft warn only (R1: do not clear exec snapshot)
# Aggressive lane-change desire window (CP-style early trigger).
AGGRESSIVE_LC_DISTANCE_M = 800.0

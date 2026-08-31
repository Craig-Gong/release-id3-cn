"""iqNavRenderState publish from iqlink bridge (navrenderd off)."""

from __future__ import annotations

from iqpilot.iqlink.bridge import IqlinkBridge


def _bridge() -> IqlinkBridge:
  b = IqlinkBridge.__new__(IqlinkBridge)
  b.params = None
  return b


def test_render_polyline_two_points_when_ego_and_dest_valid():
  b = _bridge()
  fields = {
    "active": True,
    "destinationValid": True,
    "destinationLatitude": 32.10,
    "destinationLongitude": 118.80,
    "nextManeuverValid": True,
    "nextManeuverType": "turn",
    "nextManeuverDirection": "left",
    "nextManeuverDistance": 250.0,
  }
  msg = b._fill_render_msg(fields, 32.03, 118.75, 90.0)
  rs = msg.iqNavRenderState
  assert rs.active is True
  assert len(rs.routePolyline) == 2
  assert len(rs.routePolylineSimplified) == 2
  assert abs(rs.routePolyline[0].latitude - 32.03) < 1e-6
  assert abs(rs.routePolyline[1].latitude - 32.10) < 1e-6
  assert rs.zoomHint == 16.0


def test_vppos_is_ego_not_destination():
  b = _bridge()
  dest_lat, dest_lon = 32.20, 118.95
  fields = {
    "active": True,
    "destinationValid": True,
    "destinationLatitude": dest_lat,
    "destinationLongitude": dest_lon,
    "nextManeuverValid": False,
    "nextManeuverType": "none",
    "nextManeuverDirection": "none",
    "nextManeuverDistance": 0.0,
  }
  raw = {"vpPosPointLat": 32.03, "vpPosPointLon": 118.90}
  msg = b._fill_render_msg(fields, 0.0, 0.0, 0.0, raw)
  rs = msg.iqNavRenderState
  assert abs(rs.currentLatitude - 32.03) < 1e-6
  assert abs(rs.destinationLatitude - dest_lat) < 1e-6
  assert abs(rs.destinationLatitude - rs.currentLatitude) > 0.01
  assert len(rs.routePolyline) == 2


def test_render_no_polyline_without_destination_coords():
  b = _bridge()
  fields = {
    "active": True,
    "destinationValid": True,
    "destinationLatitude": 0.0,
    "destinationLongitude": 0.0,
    "nextManeuverValid": True,
    "nextManeuverType": "turn",
    "nextManeuverDirection": "right",
    "nextManeuverDistance": 100.0,
  }
  msg = b._fill_render_msg(fields, 32.03, 118.75, 45.0)
  rs = msg.iqNavRenderState
  assert rs.active is True
  assert len(rs.routePolyline) == 0

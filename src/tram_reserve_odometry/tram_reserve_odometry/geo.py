from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple


# WGS-84 / UTM constants. MGRS uses UTM/UPS as its metric grid underneath.
_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_UTM_K0 = 0.9996


@dataclass(frozen=True)
class GridPoint:
    easting: float
    northing: float
    altitude: float
    zone: int
    northern: bool


def utm_zone_from_lon_lat(lon_deg: float, lat_deg: float) -> int:
    """Return the UTM zone, including the standard Norway/Svalbard exceptions."""
    zone = int((lon_deg + 180.0) / 6.0) + 1
    zone = max(1, min(60, zone))
    if 56.0 <= lat_deg < 64.0 and 3.0 <= lon_deg < 12.0:
        zone = 32
    if 72.0 <= lat_deg < 84.0:
        if 0.0 <= lon_deg < 9.0:
            zone = 31
        elif lon_deg < 21.0:
            zone = 33
        elif lon_deg < 33.0:
            zone = 35
        elif lon_deg < 42.0:
            zone = 37
    return zone


def wgs84_to_utm(
    lat_deg: float,
    lon_deg: float,
    altitude: float = 0.0,
    force_zone: Optional[int] = None,
) -> GridPoint:
    """
    Pure-Python WGS84 -> UTM conversion.

    x is UTM/MGRS-grid easting and y is northing in metres. Nothing is
    subtracted from the first GNSS point, deliberately: this preserves the
    grid convergence of the MGRS/UTM frame instead of silently switching to
    a local ENU tangent frame.
    """
    if not (-80.0 <= lat_deg <= 84.0):
        raise ValueError("UTM is defined here only for latitude [-80, 84] deg")
    zone = force_zone or utm_zone_from_lon_lat(lon_deg, lat_deg)
    northern = lat_deg >= 0.0

    a = _WGS84_A
    f = _WGS84_F
    e2 = f * (2.0 - f)
    ep2 = e2 / (1.0 - e2)

    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    tan_lat = math.tan(lat)
    n = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    t = tan_lat * tan_lat
    c = ep2 * cos_lat * cos_lat
    A = cos_lat * (lon - lon0)

    e4 = e2 * e2
    e6 = e4 * e2
    M = a * (
        (1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * lat
        - (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * math.sin(2 * lat)
        + (15 * e4 / 256 + 45 * e6 / 1024) * math.sin(4 * lat)
        - (35 * e6 / 3072) * math.sin(6 * lat)
    )

    easting = _UTM_K0 * n * (
        A
        + (1 - t + c) * A**3 / 6
        + (5 - 18 * t + t * t + 72 * c - 58 * ep2) * A**5 / 120
    ) + 500000.0

    northing = _UTM_K0 * (
        M
        + n * tan_lat * (
            A * A / 2
            + (5 - t + 9 * c + 4 * c * c) * A**4 / 24
            + (61 - 58 * t + t * t + 600 * c - 330 * ep2) * A**6 / 720
        )
    )
    if not northern:
        northing += 10000000.0
    return GridPoint(easting, northing, altitude, zone, northern)


def yaw_from_two_points(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.atan2(y1 - y0, x1 - x0)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    h = 0.5 * yaw
    return 0.0, 0.0, math.sin(h), math.cos(h)

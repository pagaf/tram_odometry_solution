from __future__ import annotations

import bisect
import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import List, Sequence, Tuple


@dataclass
class PathSample:
    x: float
    y: float
    z: float
    yaw: float


@dataclass
class PathDynamics:
    grade: float      # dz / horizontal ds
    curvature: float  # signed 1/R [1/m]


class PolylinePath:
    """Lightweight metric rail centreline parameterized by horizontal arc length."""

    def __init__(self, points: Sequence[Tuple[float, float, float]]):
        if len(points) < 2:
            raise ValueError("path needs at least two points")
        self.points = [(float(x), float(y), float(z)) for x, y, z in points]
        self.s = [0.0]
        for (x0, y0, _), (x1, y1, _) in zip(self.points[:-1], self.points[1:]):
            ds = math.hypot(x1 - x0, y1 - y0)
            self.s.append(self.s[-1] + max(ds, 1e-6))

    @classmethod
    def from_csv(cls, path: str) -> "PolylinePath":
        p = Path(path)
        with p.open("r", newline="") as f:
            rows = list(csv.reader(f))
        if not rows:
            raise ValueError(f"empty path file: {path}")

        def is_number(s: str) -> bool:
            try:
                float(s)
                return True
            except Exception:
                return False

        first = [c.strip().lower() for c in rows[0]]
        has_header = not all(is_number(c) for c in rows[0][:2])
        points: List[Tuple[float, float, float]] = []
        if has_header:
            idx = {name: i for i, name in enumerate(first)}
            xk = next((k for k in ("x", "easting", "east") if k in idx), None)
            yk = next((k for k in ("y", "northing", "north") if k in idx), None)
            zk = next((k for k in ("z", "altitude", "height") if k in idx), None)
            if xk is None or yk is None:
                raise ValueError("CSV header must contain x/easting and y/northing")
            for row in rows[1:]:
                if not row:
                    continue
                x = float(row[idx[xk]])
                y = float(row[idx[yk]])
                z = float(row[idx[zk]]) if zk is not None and idx[zk] < len(row) and row[idx[zk]] else 0.0
                points.append((x, y, z))
        else:
            for row in rows:
                if len(row) < 2:
                    continue
                points.append((float(row[0]), float(row[1]), float(row[2]) if len(row) > 2 and row[2] else 0.0))
        return cls(points)

    @property
    def length(self) -> float:
        return self.s[-1]

    def sample(self, s_query: float) -> PathSample:
        s_query = min(max(0.0, s_query), self.s[-1])
        i = max(0, min(len(self.s) - 2, bisect.bisect_right(self.s, s_query) - 1))
        s0, s1 = self.s[i], self.s[i + 1]
        t = 0.0 if s1 <= s0 else (s_query - s0) / (s1 - s0)
        p0, p1 = self.points[i], self.points[i + 1]
        x = p0[0] + t * (p1[0] - p0[0])
        y = p0[1] + t * (p1[1] - p0[1])
        z = p0[2] + t * (p1[2] - p0[2])
        yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        return PathSample(x, y, z, yaw)

    def project(self, x: float, y: float) -> Tuple[float, float]:
        """Return (s, lateral_distance) of the nearest point on the polyline."""
        best_d2 = float("inf")
        best_s = 0.0
        for i in range(len(self.points) - 1):
            x0, y0, _ = self.points[i]
            x1, y1, _ = self.points[i + 1]
            dx, dy = x1 - x0, y1 - y0
            l2 = dx * dx + dy * dy
            if l2 <= 1e-12:
                continue
            q = clamp01(((x - x0) * dx + (y - y0) * dy) / l2)
            px, py = x0 + q * dx, y0 + q * dy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_s = self.s[i] + q * (self.s[i + 1] - self.s[i])
        return best_s, math.sqrt(best_d2)

    def dynamics(self, s_query: float, window: float = 3.0) -> PathDynamics:
        """Estimate grade and curvature by centred finite differences.

        A few-metre window suppresses polyline quantization noise and is cheap enough
        to run for every filter prediction.
        """
        h = max(0.5, float(window))
        s0 = max(0.0, s_query - h)
        s1 = min(self.length, s_query + h)
        if s1 - s0 < 1e-4:
            return PathDynamics(0.0, 0.0)
        p0 = self.sample(s0)
        p1 = self.sample(s1)
        grade = (p1.z - p0.z) / (s1 - s0)
        dyaw = wrap_angle(p1.yaw - p0.yaw)
        curvature = dyaw / (s1 - s0)
        return PathDynamics(grade, curvature)

    def body_yaw(self, s_front: float, wheelbase: float) -> float:
        """Body yaw using the front/rear bogie chord constrained by wheelbase."""
        pf = self.sample(s_front)
        guess = max(0.0, s_front - wheelbase)
        lo = max(0.0, guess - 1.5)
        hi = min(s_front, guess + 1.5)
        best = None
        for j in range(31):
            sr = lo + (hi - lo) * j / 30.0
            pr = self.sample(sr)
            err = abs(math.hypot(pf.x - pr.x, pf.y - pr.y) - wheelbase)
            if best is None or err < best[0]:
                best = (err, pr)
        pr = best[1] if best else self.sample(guess)
        return math.atan2(pf.y - pr.y, pf.x - pr.x)


def clamp01(x: float) -> float:
    return min(1.0, max(0.0, x))


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

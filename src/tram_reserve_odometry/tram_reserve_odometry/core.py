from __future__ import annotations

from dataclasses import dataclass, field, fields
import math
from typing import Callable, List, Optional, Tuple

from .filter import RobustTramFilter
from .geo import wgs84_to_utm
from .path_model import PolylinePath


DEFAULT_DYNAMICS_COEFFS = [
    -0.10134737355232307,
    2.247224149328625,
    -0.7227761121566878,
    -2.418307339106011,
    1.7315138159232895,
    0.014344865261432287,
    -0.0006659416778320035,
    -0.07946674873014506,
    -0.01741991290433522,
]


@dataclass
class CoreConfig:
    """All estimator parameters; mirrors config/odom_params.yaml one-to-one."""

    wheel_units: str = "kmph"
    wheelbase_m: float = 7.55
    master_x_m: float = -9.873
    rover_x_m: float = 2.563
    antenna_z_m: float = 3.0
    gnss_init_seconds: float = 5.0
    gnss_pair_tolerance_s: float = 0.20
    # Output/map frame = UTM(zone) - grid offset. Organizer pathgraph uses 300000/6100000.
    utm_zone: int = 37
    grid_offset_easting: float = 300000.0
    grid_offset_northing: float = 6100000.0
    path_z_is_rail_height: bool = True
    # Starts off the route (a parallel terminal stub, a siding) keep the initial GNSS
    # offset from the route and fade it out over join_length_m of travel.
    max_path_init_lateral_m: float = 100.0
    join_length_m: float = 30.0
    # Platform stop anchoring: after a confirmed standstill of stop_anchor_min_s near a
    # surveyed stopping point, pull the along-track position towards it.
    stop_anchor_min_s: float = 8.0
    stop_anchor_window_m: float = 25.0
    stop_anchor_gain: float = 0.8
    manual_initialization: bool = False
    initial_easting: float = 0.0
    initial_northing: float = 0.0
    initial_z: float = 0.0
    initial_yaw: float = 0.0
    tau_accel: float = 0.35
    wheel_sigma: float = 0.10
    process_accel_sigma: float = 1.5
    max_speed_mps: float = 25.0
    bias_adapt_gain: float = 0.006
    grade_gain: float = 0.0
    curve_accel_per_curvature: float = 0.0
    nis_gate: float = 9.0
    stop_speed_mps: float = 0.06
    stop_confirm_s: float = 0.35
    common_slip_weight: float = 1.0
    consensus_tol_mps: float = 0.15
    wheel_fresh_s: float = 0.22
    publish_max_rate_hz: float = 50.0
    vehicle_mass_kg: float = 0.0
    wheel_radius_m: float = 0.0
    gear_ratio: float = 0.0
    drive_efficiency: float = 1.0
    driven_equivalent_count: float = 1.0
    dynamics_coeffs: List[float] = field(default_factory=lambda: list(DEFAULT_DYNAMICS_COEFFS))

    @classmethod
    def names(cls) -> List[str]:
        return [f.name for f in fields(cls)]


class ReserveOdometryCore:
    """ROS-free estimator core shared by the ROS node and offline replay tools.

    Inputs are plain numbers with message header stamps; outputs are the state and
    pose in the configured metric map frame. Keeping ROS out of this class lets
    tools/replay_eval.py validate exactly the code that runs in the node.
    """

    def __init__(
        self,
        cfg: CoreConfig,
        path: Optional[PolylinePath] = None,
        log: Optional[Callable[[str], None]] = None,
    ):
        self.cfg = cfg
        self.path = path
        self.log = log or (lambda _msg: None)
        self.baseline = cfg.rover_x_m - cfg.master_x_m

        self.filter = RobustTramFilter(
            coeffs=list(cfg.dynamics_coeffs),
            tau_accel=cfg.tau_accel,
            wheel_sigma=cfg.wheel_sigma,
            process_accel_sigma=cfg.process_accel_sigma,
            max_speed=cfg.max_speed_mps,
            bias_adapt_gain=cfg.bias_adapt_gain,
            grade_gain=cfg.grade_gain,
            curve_accel_per_curvature=cfg.curve_accel_per_curvature,
            nis_gate=cfg.nis_gate,
            stop_speed=cfg.stop_speed_mps,
            stop_confirm_s=cfg.stop_confirm_s,
            common_slip_weight=cfg.common_slip_weight,
            consensus_tol_mps=cfg.consensus_tol_mps,
            vehicle_mass_kg=cfg.vehicle_mass_kg,
            wheel_radius_m=cfg.wheel_radius_m,
            gear_ratio=cfg.gear_ratio,
            drive_efficiency=cfg.drive_efficiency,
            driven_equivalent_count=cfg.driven_equivalent_count,
        )
        self.notch = 0
        self.front: Optional[Tuple[float, float]] = None
        self.rear: Optional[Tuple[float, float]] = None
        self.last_used_front_t: Optional[float] = None
        self.last_used_rear_t: Optional[float] = None
        self.last_publish_t: Optional[float] = None
        self.publish_min_dt = 1.0 / max(1.0, float(cfg.publish_max_rate_hz))

        # Position anchor. distance=filter.distance; path_s = path_offset + direction*distance.
        self.position_ready = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_z = 0.0
        self.anchor_yaw = 0.0
        self.path_offset = 0.0
        self.path_direction = 1.0
        self.path_valid = path is not None
        self.join_dx = 0.0
        self.join_dy = 0.0
        self.join_d0 = 0.0
        self.stop_anchored = False
        self.stop_anchor_count = 0

        self.master_fix: Optional[Tuple[float, float, float, float]] = None
        self.rover_fix: Optional[Tuple[float, float, float, float]] = None
        self.first_gnss_t: Optional[float] = None
        self.gnss_frozen = False
        self.heading_cos = 0.0
        self.heading_sin = 0.0
        self.heading_samples = 0

        if cfg.manual_initialization:
            self._set_anchor_from_current_pose(
                cfg.initial_easting, cfg.initial_northing, cfg.initial_z, cfg.initial_yaw
            )

    # ------------------------------------------------------------------ inputs
    def _wheel_to_mps(self, value: float) -> float:
        if self.cfg.wheel_units.lower() in ("kmph", "km/h", "kph"):
            return float(value) / 3.6
        return float(value)

    def _advance(self, t: float) -> None:
        grade = 0.0
        curvature = 0.0
        if self.path_valid and self.position_ready:
            dyn = self.path.dynamics(self.path_s())
            # Reverse path direction changes the sign of grade/curvature as seen by motion.
            grade = self.path_direction * dyn.grade
            curvature = self.path_direction * dyn.curvature
        self.filter.predict_to(t, self.notch, grade=grade, curvature=curvature)

    def on_front(self, t: float, raw_velocity: float) -> None:
        self._advance(t)
        self.front = (t, self._wheel_to_mps(raw_velocity))
        self._use_new_wheels(t)

    def on_rear(self, t: float, raw_velocity: float) -> None:
        self._advance(t)
        self.rear = (t, self._wheel_to_mps(raw_velocity))
        self._use_new_wheels(t)

    def on_cmd(self, t: float, position: int) -> None:
        self._advance(t)
        self.notch = int(position)

    def _use_new_wheels(self, t: float) -> None:
        # Use both fresh values as slip-detection context, but assimilate each physical
        # sample only once in the Kalman update.
        fresh = self.cfg.wheel_fresh_s
        fv = self.front[1] if self.front is not None and abs(t - self.front[0]) <= fresh else None
        rv = self.rear[1] if self.rear is not None and abs(t - self.rear[0]) <= fresh else None
        uf = fv is not None and self.front[0] != self.last_used_front_t
        ur = rv is not None and self.rear[0] != self.last_used_rear_t
        if not uf and not ur:
            return
        self.filter.update_wheels(t, self.notch, fv, rv, update_front=uf, update_rear=ur)
        if uf:
            self.last_used_front_t = self.front[0]
        if ur:
            self.last_used_rear_t = self.rear[0]
        self._maybe_anchor_to_stop(t)

    def _maybe_anchor_to_stop(self, t: float) -> None:
        """Map-matching on platform stops: the only absolute along-track cue left
        without GNSS. Wheel scale errors (+-1.5 % between runs) otherwise accumulate."""
        since = self.filter.stop_since
        if since is None:
            self.stop_anchored = False
            return
        if (
            self.stop_anchored
            or not (self.path_valid and self.position_ready)
            or self.cfg.stop_anchor_gain <= 0.0
            or t - since < self.cfg.stop_anchor_min_s
        ):
            return
        self.stop_anchored = True
        s_now = self.path_s()
        s_stop = self.path.nearest_stop(s_now)
        if s_stop is None or abs(s_stop - s_now) > self.cfg.stop_anchor_window_m:
            return
        self.path_offset += self.cfg.stop_anchor_gain * (s_stop - s_now)
        self.stop_anchor_count += 1

    def should_publish(self, t: float, force: bool = False) -> bool:
        if self.last_publish_t is not None:
            if t < self.last_publish_t - 0.10:
                self.last_publish_t = None
            elif not force and t - self.last_publish_t < self.publish_min_dt:
                return False
        self.last_publish_t = t
        return True

    # -------------------------------------------------------------- GNSS init
    def on_gnss_fix(self, antenna: str, t: float, lat: float, lon: float, alt: float, status: int = 0) -> None:
        """Consume a GNSS fix during the initialization window only."""
        if self.gnss_frozen:
            return
        if status < 0 or not (math.isfinite(lat) and math.isfinite(lon) and math.isfinite(alt)):
            return
        if antenna == "master":
            self.master_fix = (t, lat, lon, alt)
        else:
            self.rover_fix = (t, lat, lon, alt)
        self._try_gnss_pair(t)

    def to_map(self, lat: float, lon: float) -> Tuple[float, float]:
        g = wgs84_to_utm(lat, lon, 0.0, self.cfg.utm_zone)
        return g.easting - self.cfg.grid_offset_easting, g.northing - self.cfg.grid_offset_northing

    def _try_gnss_pair(self, t_now: float) -> None:
        if self.master_fix is None or self.rover_fix is None:
            return
        tm, mlat, mlon, malt = self.master_fix
        tr, rlat, rlon, ralt = self.rover_fix
        if abs(tm - tr) > self.cfg.gnss_pair_tolerance_s:
            return
        if self.first_gnss_t is None:
            self.first_gnss_t = min(tm, tr)
        if t_now - self.first_gnss_t > self.cfg.gnss_init_seconds:
            self.gnss_frozen = True
            self.log("GNSS initialization window closed; continuing from wheels + model only")
            return

        mx, my = self.to_map(mlat, mlon)
        rx, ry = self.to_map(rlat, rlon)
        dx, dy = rx - mx, ry - my
        observed_baseline = math.hypot(dx, dy)
        if not (0.55 * self.baseline <= observed_baseline <= 1.45 * self.baseline):
            return
        yaw = math.atan2(dy, dx)
        self.heading_cos += math.cos(yaw)
        self.heading_sin += math.sin(yaw)
        self.heading_samples += 1
        yaw_f = math.atan2(self.heading_sin, self.heading_cos)
        hx, hy = math.cos(yaw_f), math.sin(yaw_f)

        # p_ant = p_base + R(yaw) * [x_ant, 0]; solve p_base from both antennas.
        bx = 0.5 * ((mx - self.cfg.master_x_m * hx) + (rx - self.cfg.rover_x_m * hx))
        by = 0.5 * ((my - self.cfg.master_x_m * hy) + (ry - self.cfg.rover_x_m * hy))
        bz = 0.5 * (malt + ralt) - self.cfg.antenna_z_m
        self._set_anchor_from_current_pose(bx, by, bz, yaw_f)

    def _set_anchor_from_current_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        d = self.filter.distance
        self.origin_z = z
        self.anchor_yaw = yaw
        self.origin_x = x - d * math.cos(yaw)
        self.origin_y = y - d * math.sin(yaw)
        if self.path is not None:
            s_now, lateral = self.path.project(x, y, yaw)
            self.path_valid = lateral <= self.cfg.max_path_init_lateral_m
            if self.path_valid:
                ps = self.path.sample(s_now)
                self.path_direction = 1.0 if math.cos(ps.yaw - yaw) >= 0.0 else -1.0
                self.path_offset = s_now - self.path_direction * d
                self.join_dx = x - ps.x
                self.join_dy = y - ps.y
                self.join_d0 = d
            elif not self.position_ready:
                self.log(f"GNSS base_link is {lateral:.1f} m from configured path; straight-line fallback")
        self.position_ready = True

    # ----------------------------------------------------------------- output
    @property
    def velocity(self) -> float:
        return self.filter.velocity

    def path_s(self) -> float:
        return self.path_offset + self.path_direction * self.filter.distance

    def pose(self) -> Tuple[float, float, float, float]:
        """Pose in the map frame; callers must not publish it before position_ready."""
        d = self.filter.distance
        if not self.position_ready:
            # Relative odometry until GNSS/manual initialization appears.
            return d, 0.0, 0.0, 0.0
        if not self.path_valid:
            return (
                self.origin_x + d * math.cos(self.anchor_yaw),
                self.origin_y + d * math.sin(self.anchor_yaw),
                self.origin_z,
                self.anchor_yaw,
            )
        s_front = self.path_s()
        ps = self.path.sample(s_front)
        yaw = self.path.body_yaw(s_front, self.cfg.wheelbase_m)
        if self.path_direction < 0.0:
            yaw = math.atan2(-math.sin(yaw), -math.cos(yaw))
        fade = math.exp(-max(0.0, d - self.join_d0) / max(1e-3, self.cfg.join_length_m))
        x = ps.x + fade * self.join_dx
        y = ps.y + fade * self.join_dy
        z = ps.z if self.cfg.path_z_is_rail_height else ps.z - self.cfg.antenna_z_m
        # A path without meaningful z often contains zeros; in that case keep initialized rail height.
        if abs(ps.z) < 1e-9 and abs(self.origin_z) > 1e-9:
            z = self.origin_z
        return x, y, z, yaw

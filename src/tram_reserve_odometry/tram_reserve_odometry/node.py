from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from tram_vehicle_msgs.msg import VelocitySensor, DriverControllerCommand

from .filter import RobustTramFilter
from .geo import quat_from_yaw, wgs84_to_utm
from .path_model import PolylinePath


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


class ReserveOdometryNode(Node):
    def __init__(self):
        super().__init__("tram_reserve_odometry")

        self.declare_parameter("wheel_units", "kmph")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("wheelbase_m", 7.55)
        self.declare_parameter("master_x_m", -9.873)
        self.declare_parameter("rover_x_m", 2.563)
        self.declare_parameter("antenna_z_m", 3.0)
        self.declare_parameter("gnss_init_seconds", 5.0)
        self.declare_parameter("gnss_pair_tolerance_s", 0.20)
        self.declare_parameter("path_csv", "")
        self.declare_parameter("path_z_is_rail_height", True)
        self.declare_parameter("manual_initialization", False)
        self.declare_parameter("initial_easting", 0.0)
        self.declare_parameter("initial_northing", 0.0)
        self.declare_parameter("initial_z", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("tau_accel", 0.35)
        self.declare_parameter("wheel_sigma", 0.10)
        self.declare_parameter("process_accel_sigma", 0.55)
        self.declare_parameter("max_speed_mps", 25.0)
        self.declare_parameter("bias_adapt_gain", 0.01)
        self.declare_parameter("grade_gain", 1.0)
        self.declare_parameter("curve_accel_per_curvature", 0.0)
        self.declare_parameter("nis_gate", 9.0)
        self.declare_parameter("stop_speed_mps", 0.06)
        self.declare_parameter("stop_confirm_s", 0.35)
        self.declare_parameter("publish_max_rate_hz", 50.0)
        self.declare_parameter("vehicle_mass_kg", 0.0)
        self.declare_parameter("wheel_radius_m", 0.0)
        self.declare_parameter("gear_ratio", 0.0)
        self.declare_parameter("drive_efficiency", 1.0)
        self.declare_parameter("driven_equivalent_count", 1.0)
        self.declare_parameter("dynamics_coeffs", [
            0.000140604885,
            1.85266353,
            -0.268904638,
            -2.43778005,
            1.57779757,
            -0.00808316467,
            0.000480035941,
            -0.0745746855,
            0.0115504134,
        ])

        p = lambda name: self.get_parameter(name).value
        self.wheel_units = str(p("wheel_units"))
        self.frame_id = str(p("frame_id"))
        self.child_frame_id = str(p("child_frame_id"))
        self.wheelbase = float(p("wheelbase_m"))
        self.master_x = float(p("master_x_m"))
        self.rover_x = float(p("rover_x_m"))
        self.antenna_z = float(p("antenna_z_m"))
        self.baseline = self.rover_x - self.master_x
        self.gnss_init_seconds = float(p("gnss_init_seconds"))
        self.gnss_pair_tol = float(p("gnss_pair_tolerance_s"))
        self.path_z_is_rail = bool(p("path_z_is_rail_height"))

        self.filter = RobustTramFilter(
            coeffs=list(p("dynamics_coeffs")),
            tau_accel=float(p("tau_accel")),
            wheel_sigma=float(p("wheel_sigma")),
            process_accel_sigma=float(p("process_accel_sigma")),
            max_speed=float(p("max_speed_mps")),
            bias_adapt_gain=float(p("bias_adapt_gain")),
            grade_gain=float(p("grade_gain")),
            curve_accel_per_curvature=float(p("curve_accel_per_curvature")),
            nis_gate=float(p("nis_gate")),
            stop_speed=float(p("stop_speed_mps")),
            stop_confirm_s=float(p("stop_confirm_s")),
            vehicle_mass_kg=float(p("vehicle_mass_kg")),
            wheel_radius_m=float(p("wheel_radius_m")),
            gear_ratio=float(p("gear_ratio")),
            drive_efficiency=float(p("drive_efficiency")),
            driven_equivalent_count=float(p("driven_equivalent_count")),
        )
        self.notch = 0
        self.front: Optional[Tuple[float, float]] = None
        self.rear: Optional[Tuple[float, float]] = None
        self.last_used_front_t: Optional[float] = None
        self.last_used_rear_t: Optional[float] = None
        self.last_publish_t: Optional[float] = None
        self.publish_min_dt = 1.0 / max(1.0, float(p("publish_max_rate_hz")))

        path_csv = str(p("path_csv"))
        self.path: Optional[PolylinePath] = None
        if path_csv:
            self.path = PolylinePath.from_csv(path_csv)
            self.get_logger().info(f"Loaded path: {path_csv}, length={self.path.length:.1f} m")

        # Position anchor. distance=filter.distance; path_s = path_offset + direction*distance.
        self.position_ready = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_z = 0.0
        self.anchor_yaw = 0.0
        self.path_offset = 0.0
        self.path_direction = 1.0
        self.utm_zone: Optional[int] = None
        self.utm_northern: Optional[bool] = None

        if bool(p("manual_initialization")):
            self.origin_x = float(p("initial_easting"))
            self.origin_y = float(p("initial_northing"))
            self.origin_z = float(p("initial_z"))
            self.anchor_yaw = float(p("initial_yaw"))
            self._set_anchor_from_current_pose(self.origin_x, self.origin_y, self.origin_z, self.anchor_yaw)

        self.master_fix: Optional[Tuple[float, NavSatFix]] = None
        self.rover_fix: Optional[Tuple[float, NavSatFix]] = None
        self.master_vel: Optional[Tuple[float, TwistStamped]] = None
        self.rover_vel: Optional[Tuple[float, TwistStamped]] = None
        self.first_gnss_t: Optional[float] = None
        self.gnss_frozen = False
        self.heading_cos = 0.0
        self.heading_sin = 0.0
        self.heading_samples = 0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Inputs
        self.create_subscription(VelocitySensor, "/vehicle/front_bogie_velocity", self.on_front, qos)
        self.create_subscription(VelocitySensor, "/vehicle/rear_bogie_velocity", self.on_rear, qos)
        self.create_subscription(DriverControllerCommand, "/vehicle/driver_position_cmd", self.on_cmd, qos)
        # GNSS is consumed only during the short initialization window.
        self.create_subscription(NavSatFix, "/sensing/gnss/master/fix", self.on_master_fix, qos)
        self.create_subscription(NavSatFix, "/sensing/gnss/rover/fix", self.on_rover_fix, qos)
        self.create_subscription(TwistStamped, "/sensing/gnss/master/vel", self.on_master_vel, qos)
        self.create_subscription(TwistStamped, "/sensing/gnss/rover/vel", self.on_rover_vel, qos)

        self.vel_pub = self.create_publisher(VelocitySensor, "/result/velocity", 10)
        self.odom_pub = self.create_publisher(Odometry, "/result/position", 10)
        self.diag_pub = self.create_publisher(TwistStamped, "/result/diagnostics", 10)
        self.latency_pub = self.create_publisher(Float64, "/result/latency_ms", 10)
        self.torque_pub = self.create_publisher(Float64, "/result/model_motor_torque_nm", 10)
        self.get_logger().info("Reserve odometry ready; wheel speed is interpreted as km/h and converted to m/s")

    def _wheel_to_mps(self, value: float) -> float:
        if self.wheel_units.lower() in ("kmph", "km/h", "kph"):
            return float(value) / 3.6
        return float(value)

    def _advance(self, t: float) -> None:
        grade = 0.0
        curvature = 0.0
        if self.path is not None and self.position_ready:
            s_front = self.path_offset + self.path_direction * self.filter.distance
            dyn = self.path.dynamics(s_front)
            # Reverse path direction changes the sign of grade/curvature as seen by motion.
            grade = self.path_direction * dyn.grade
            curvature = self.path_direction * dyn.curvature
        self.filter.predict_to(t, self.notch, grade=grade, curvature=curvature)

    def _maybe_publish(self, stamp, t: float, started: Optional[float] = None, force: bool = False) -> None:
        if self.last_publish_t is not None:
            if t < self.last_publish_t - 0.10:
                self.last_publish_t = None
            elif not force and t - self.last_publish_t < self.publish_min_dt:
                return
        self.publish_result(stamp)
        self.last_publish_t = t
        if started is not None:
            lm = Float64()
            lm.data = 1000.0 * (time.perf_counter() - started)
            self.latency_pub.publish(lm)

    def on_front(self, msg: VelocitySensor) -> None:
        started = time.perf_counter()
        t = stamp_to_sec(msg.header.stamp)
        self._advance(t)
        self.front = (t, self._wheel_to_mps(msg.velocity))
        self._use_new_wheels(t)
        self._maybe_publish(msg.header.stamp, t, started)

    def on_rear(self, msg: VelocitySensor) -> None:
        started = time.perf_counter()
        t = stamp_to_sec(msg.header.stamp)
        self._advance(t)
        self.rear = (t, self._wheel_to_mps(msg.velocity))
        self._use_new_wheels(t)
        self._maybe_publish(msg.header.stamp, t, started)

    def _use_new_wheels(self, t: float) -> None:
        # Use both fresh values as slip-detection context, but assimilate each physical
        # sample only once in the Kalman update.
        fresh = 0.22
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

    def on_cmd(self, msg: DriverControllerCommand) -> None:
        started = time.perf_counter()
        t = stamp_to_sec(msg.header.stamp)
        self._advance(t)
        self.notch = int(msg.position)
        # Publish on whichever allowed input arrives; throttle keeps output <= configured rate.
        self._maybe_publish(msg.header.stamp, t, started)

    def on_master_vel(self, msg: TwistStamped) -> None:
        if self.gnss_frozen:
            return
        self.master_vel = (stamp_to_sec(msg.header.stamp), msg)

    def on_rover_vel(self, msg: TwistStamped) -> None:
        if self.gnss_frozen:
            return
        self.rover_vel = (stamp_to_sec(msg.header.stamp), msg)

    def on_master_fix(self, msg: NavSatFix) -> None:
        if self.gnss_frozen:
            return
        t = stamp_to_sec(msg.header.stamp)
        self.master_fix = (t, msg)
        self._try_gnss_pair(t)

    def on_rover_fix(self, msg: NavSatFix) -> None:
        if self.gnss_frozen:
            return
        t = stamp_to_sec(msg.header.stamp)
        self.rover_fix = (t, msg)
        self._try_gnss_pair(t)

    def _valid_fix(self, msg: NavSatFix) -> bool:
        return math.isfinite(msg.latitude) and math.isfinite(msg.longitude) and math.isfinite(msg.altitude)

    def _try_gnss_pair(self, t_now: float) -> None:
        if self.master_fix is None or self.rover_fix is None:
            return
        tm, m = self.master_fix
        tr, r = self.rover_fix
        if abs(tm - tr) > self.gnss_pair_tol or not self._valid_fix(m) or not self._valid_fix(r):
            return
        if self.first_gnss_t is None:
            self.first_gnss_t = min(tm, tr)
        if t_now - self.first_gnss_t > self.gnss_init_seconds:
            self.gnss_frozen = True
            self.get_logger().info("GNSS initialization window closed; continuing from wheels + model only")
            return

        if self.utm_zone is None:
            gp0 = wgs84_to_utm(m.latitude, m.longitude, m.altitude)
            self.utm_zone, self.utm_northern = gp0.zone, gp0.northern
        gm = wgs84_to_utm(m.latitude, m.longitude, m.altitude, self.utm_zone)
        gr = wgs84_to_utm(r.latitude, r.longitude, r.altitude, self.utm_zone)
        dx, dy = gr.easting - gm.easting, gr.northing - gm.northing
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
        bx_m = gm.easting - self.master_x * hx
        by_m = gm.northing - self.master_x * hy
        bx_r = gr.easting - self.rover_x * hx
        by_r = gr.northing - self.rover_x * hy
        bx = 0.5 * (bx_m + bx_r)
        by = 0.5 * (by_m + by_r)
        bz = 0.5 * (gm.altitude + gr.altitude) - self.antenna_z
        self._set_anchor_from_current_pose(bx, by, bz, yaw_f)

    def _set_anchor_from_current_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        d = self.filter.distance
        self.origin_z = z
        self.anchor_yaw = yaw
        if self.path is not None:
            s_now, lateral = self.path.project(x, y)
            tangent = self.path.sample(s_now).yaw
            self.path_direction = 1.0 if math.cos(tangent - yaw) >= 0.0 else -1.0
            self.path_offset = s_now - self.path_direction * d
            if lateral > 8.0:
                self.get_logger().warn(f"GNSS base_link is {lateral:.1f} m from configured path")
        else:
            self.origin_x = x - d * math.cos(yaw)
            self.origin_y = y - d * math.sin(yaw)
        self.position_ready = True

    def _pose(self) -> Tuple[float, float, float, float]:
        d = self.filter.distance
        if not self.position_ready:
            # Relative fallback until GNSS/manual initialization appears.
            return d, 0.0, 0.0, 0.0
        if self.path is None:
            return (
                self.origin_x + d * math.cos(self.anchor_yaw),
                self.origin_y + d * math.sin(self.anchor_yaw),
                self.origin_z,
                self.anchor_yaw,
            )
        s_front = self.path_offset + self.path_direction * d
        ps = self.path.sample(s_front)
        yaw = self.path.body_yaw(s_front, self.wheelbase)
        if self.path_direction < 0.0:
            yaw = math.atan2(-math.sin(yaw), -math.cos(yaw))
        z = ps.z if self.path_z_is_rail else ps.z - self.antenna_z
        # A path without meaningful z often contains zeros; in that case keep initialized rail height.
        if abs(ps.z) < 1e-9 and abs(self.origin_z) > 1e-9:
            z = self.origin_z
        return ps.x, ps.y, z, yaw

    def publish_result(self, stamp) -> None:
        vmsg = VelocitySensor()
        vmsg.header.stamp = stamp
        vmsg.header.frame_id = self.child_frame_id
        vmsg.velocity = float(self.filter.velocity)  # required output is m/s
        self.vel_pub.publish(vmsg)

        x, y, z, yaw = self._pose()
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.frame_id
        o.child_frame_id = self.child_frame_id
        o.pose.pose.position.x = float(x)
        o.pose.pose.position.y = float(y)
        o.pose.pose.position.z = float(z)
        qx, qy, qz, qw = quat_from_yaw(yaw)
        o.pose.pose.orientation.x = qx
        o.pose.pose.orientation.y = qy
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = float(self.filter.velocity)
        # Conservative covariance: grows with along-track uncertainty; map constrains cross-track.
        pos_var = max(0.04, self.filter.P[0][0])
        cross_var = 0.25 if self.path is not None else max(1.0, pos_var)
        o.pose.covariance[0] = pos_var
        o.pose.covariance[7] = cross_var
        o.pose.covariance[14] = 1.0
        o.pose.covariance[35] = 0.04 if self.path is not None else 0.25
        o.twist.covariance[0] = max(1e-4, self.filter.P[1][1])
        self.odom_pub.publish(o)

        # Optional diagnostics: x=slip score, y=front weight, z=rear weight; angular.x=model accel.
        d = TwistStamped()
        d.header.stamp = stamp
        d.header.frame_id = self.child_frame_id
        d.twist.linear.x = float(self.filter.diag.slip_score)
        d.twist.linear.y = float(self.filter.diag.front_weight)
        d.twist.linear.z = float(self.filter.diag.rear_weight)
        d.twist.angular.x = float(self.filter.diag.model_accel)
        d.twist.angular.y = float(self.filter.diag.adaptive_accel_bias)
        d.twist.angular.z = float(self.filter.diag.common_slip_prob)
        self.diag_pub.publish(d)

        tq = self.filter.diag.equivalent_motor_torque_nm
        if math.isfinite(tq):
            tm = Float64()
            tm.data = float(tq)
            self.torque_pub.publish(tm)


def main(args=None):
    rclpy.init(args=args)
    node = ReserveOdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

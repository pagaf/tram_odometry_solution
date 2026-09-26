from __future__ import annotations

import math
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from tram_vehicle_msgs.msg import VelocitySensor, DriverControllerCommand

from .core import CoreConfig, ReserveOdometryCore
from .geo import quat_from_yaw
from .path_model import PolylinePath


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def resolve_path_file(value: str) -> str:
    """Absolute path, or a path relative to this package's share directory."""
    if not value or os.path.isabs(value):
        return value
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("tram_reserve_odometry"), value)
    except Exception:
        return value


class ReserveOdometryNode(Node):
    """Thin ROS 2 wrapper: messages in, ReserveOdometryCore, messages out."""

    def __init__(self):
        super().__init__("tram_reserve_odometry")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("path_file", "config/route.json")
        defaults = CoreConfig()
        for name in CoreConfig.names():
            self.declare_parameter(name, getattr(defaults, name))

        p = lambda name: self.get_parameter(name).value
        cfg = CoreConfig(**{name: p(name) for name in CoreConfig.names()})
        cfg.dynamics_coeffs = [float(x) for x in cfg.dynamics_coeffs]
        self.frame_id = str(p("frame_id"))
        self.child_frame_id = str(p("child_frame_id"))

        path: Optional[PolylinePath] = None
        path_file = resolve_path_file(str(p("path_file")))
        if path_file:
            path = PolylinePath.from_file(path_file)
            self.get_logger().info(f"Loaded path: {path_file}, length={path.length:.1f} m")
        self.core = ReserveOdometryCore(cfg, path, log=self.get_logger().info)

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

        self.vel_pub = self.create_publisher(VelocitySensor, "/result/velocity", 10)
        self.odom_pub = self.create_publisher(Odometry, "/result/position", 10)
        self.diag_pub = self.create_publisher(TwistStamped, "/result/diagnostics", 10)
        self.latency_pub = self.create_publisher(Float64, "/result/latency_ms", 10)
        self.torque_pub = self.create_publisher(Float64, "/result/model_motor_torque_nm", 10)
        self.get_logger().info("Reserve odometry ready; wheel speed is interpreted as km/h and converted to m/s")

    def _maybe_publish(self, stamp, t: float, started: float) -> None:
        if not self.core.should_publish(t):
            return
        self.publish_result(stamp)
        lm = Float64()
        lm.data = 1000.0 * (time.perf_counter() - started)
        self.latency_pub.publish(lm)

    def on_front(self, msg: VelocitySensor) -> None:
        started = time.perf_counter()
        t = stamp_to_sec(msg.header.stamp)
        self.core.on_front(t, msg.velocity)
        self._maybe_publish(msg.header.stamp, t, started)

    def on_rear(self, msg: VelocitySensor) -> None:
        started = time.perf_counter()
        t = stamp_to_sec(msg.header.stamp)
        self.core.on_rear(t, msg.velocity)
        self._maybe_publish(msg.header.stamp, t, started)

    def on_cmd(self, msg: DriverControllerCommand) -> None:
        started = time.perf_counter()
        t = stamp_to_sec(msg.header.stamp)
        self.core.on_cmd(t, msg.position)
        # Publish on whichever allowed input arrives; throttle keeps output <= configured rate.
        self._maybe_publish(msg.header.stamp, t, started)

    def on_master_fix(self, msg: NavSatFix) -> None:
        self._on_fix("master", msg)

    def on_rover_fix(self, msg: NavSatFix) -> None:
        self._on_fix("rover", msg)

    def _on_fix(self, antenna: str, msg: NavSatFix) -> None:
        if self.core.gnss_frozen:
            return
        self.core.on_gnss_fix(
            antenna, stamp_to_sec(msg.header.stamp), msg.latitude, msg.longitude, msg.altitude, msg.status.status
        )

    def publish_result(self, stamp) -> None:
        f = self.core.filter
        vmsg = VelocitySensor()
        vmsg.header.stamp = stamp
        vmsg.header.frame_id = self.child_frame_id
        vmsg.velocity = float(f.velocity)  # required output is m/s
        self.vel_pub.publish(vmsg)

        # Without an absolute anchor the pose would be meaningless in the map frame.
        if self.core.position_ready:
            x, y, z, yaw = self.core.pose()
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
            o.twist.twist.linear.x = float(f.velocity)
            # Conservative covariance: grows with along-track uncertainty; map constrains cross-track.
            on_path = self.core.path_valid
            pos_var = max(0.04, f.P[0][0])
            cross_var = 0.25 if on_path else max(1.0, pos_var)
            o.pose.covariance[0] = pos_var
            o.pose.covariance[7] = cross_var
            o.pose.covariance[14] = 1.0
            o.pose.covariance[35] = 0.04 if on_path else 0.25
            o.twist.covariance[0] = max(1e-4, f.P[1][1])
            self.odom_pub.publish(o)

        # Optional diagnostics: x=slip score, y=front weight, z=rear weight; angular.x=model accel.
        d = TwistStamped()
        d.header.stamp = stamp
        d.header.frame_id = self.child_frame_id
        d.twist.linear.x = float(f.diag.slip_score)
        d.twist.linear.y = float(f.diag.front_weight)
        d.twist.linear.z = float(f.diag.rear_weight)
        d.twist.angular.x = float(f.diag.model_accel)
        d.twist.angular.y = float(f.diag.adaptive_accel_bias)
        d.twist.angular.z = float(f.diag.common_slip_prob)
        self.diag_pub.publish(d)

        tq = f.diag.equivalent_motor_torque_nm
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

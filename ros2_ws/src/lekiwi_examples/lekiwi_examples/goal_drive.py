"""RViz の 2D Goal Pose (/goal_pose) へ、Nav2 なしで前向きに寄る。

    # ベース + LiDAR が起動している別ターミナルで
    ros2 run lekiwi_examples goal_drive

★ Nav2 と同時に起動しないこと。両方 /cmd_vel を書いて取り合う。
★ 後退しない。ゴールが後ろならその場旋回で正面へ入れてから進む。
  (この機体の後方スキャンは胴体しか見えない)

入力:
    /goal_pose   PoseStamped   RViz "2D Goal Pose"
    /scan        LaserScan     障害物
    TF           goal frame -> base_footprint

出力:
    /cmd_vel              Twist
    /goal_drive/status    String
    /goal_drive/marker    Marker
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from lekiwi_examples.goal_drive_logic import (
    Limits,
    decide,
    sector_min,
    yaw_from_quat,
)


def _transform_xy_yaw(
    tf: TransformStamped, x: float, y: float, yaw: float
) -> tuple[float, float, float]:
    """平面上で point + yaw を child へ写す (lookup は parent<-child ではない)。

    ``lookup_transform(target, source)`` の結果は source 上の点を target へ運ぶ。
    """
    t = tf.transform.translation
    q = tf.transform.rotation
    yaw_tf = yaw_from_quat(q.x, q.y, q.z, q.w)
    c, s = math.cos(yaw_tf), math.sin(yaw_tf)
    return t.x + c * x - s * y, t.y + s * x + c * y, yaw_tf + yaw


class GoalDrive(Node):
    def __init__(self) -> None:
        super().__init__("goal_drive")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("cruise", 0.07)
        self.declare_parameter("creep", 0.035)
        self.declare_parameter("turn", 0.40)
        self.declare_parameter("stop_r", 0.40)
        self.declare_parameter("slow_r", 0.75)
        self.declare_parameter("arrive_xy", 0.20)

        p = self.get_parameter
        self._base_frame = str(p("base_frame").value)
        self._limits = Limits(
            cruise=float(p("cruise").value),
            creep=float(p("creep").value),
            turn=float(p("turn").value),
            stop_r=float(p("stop_r").value),
            slow_r=float(p("slow_r").value),
            arrive_xy=float(p("arrive_xy").value),
        )

        self._goal: PoseStamped | None = None
        self._scan: LaserScan | None = None
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        self._cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self._status = self.create_publisher(String, "/goal_drive/status", 10)
        marker_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._marker = self.create_publisher(Marker, "/goal_drive/marker", marker_qos)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            "goal_drive 起動。RViz の 2D Goal Pose を待つ "
            "(Nav2 と同時に使わないこと)"
        )

    def _on_goal(self, msg: PoseStamped) -> None:
        if not msg.header.frame_id:
            self.get_logger().warn("/goal_pose に frame_id が無い。無視する")
            return
        self._goal = msg
        self.get_logger().info(
            f"goal {msg.header.frame_id} "
            f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )
        self._publish_marker(msg)

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _tick(self) -> None:
        twist = Twist()
        if self._goal is None:
            self._send(twist, "idle (2D Goal Pose を待っている)")
            return
        if self._scan is None:
            self._send(twist, "waiting for /scan")
            return

        try:
            tf = self._tf.lookup_transform(
                self._base_frame,
                self._goal.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            self._send(twist, f"tf fail: {exc}")
            return

        q = self._goal.pose.orientation
        gx, gy, gyaw = _transform_xy_yaw(
            tf,
            self._goal.pose.position.x,
            self._goal.pose.position.y,
            yaw_from_quat(q.x, q.y, q.z, q.w),
        )
        scan = self._scan
        front = sector_min(
            list(scan.ranges), scan.angle_min, scan.angle_increment,
            -35.0, 35.0, self._limits.self_r, scan.range_min, scan.range_max,
        )
        fl = sector_min(
            list(scan.ranges), scan.angle_min, scan.angle_increment,
            20.0, 70.0, self._limits.self_r, scan.range_min, scan.range_max,
        )
        fr = sector_min(
            list(scan.ranges), scan.angle_min, scan.angle_increment,
            -70.0, -20.0, self._limits.self_r, scan.range_min, scan.range_max,
        )
        cmd = decide(front, fl, fr, gx, gy, gyaw, self._limits)
        twist.linear.x = max(0.0, cmd.vx)
        twist.angular.z = cmd.wz
        self._send(
            twist,
            f"{cmd.mode} goal=({gx:.2f},{gy:.2f}) front={front:.2f} "
            f"vx={twist.linear.x:.3f} wz={twist.angular.z:.2f}",
        )

    def _send(self, twist: Twist, text: str) -> None:
        self._cmd.publish(twist)
        msg = String()
        msg.data = text
        self._status.publish(msg)

    def _publish_marker(self, goal: PoseStamped) -> None:
        marker = Marker()
        marker.header = goal.header
        marker.ns = "goal_drive"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = goal.pose
        marker.scale.x = 0.35
        marker.scale.y = 0.06
        marker.scale.z = 0.06
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.1
        marker.color.a = 1.0
        self._marker.publish(marker)


def main() -> None:
    rclpy.init()
    node = GoalDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

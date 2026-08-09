"""アームを動かして、前へ進む例。

    ros2 run lekiwi_examples example_sequence

`robot.launch.py` が動いていることが前提です。

    1. アームを stow（収納）姿勢へ
    2. アームを上げる
    3. アームを stow へ戻す
    4. 前方 50cm へナビゲーション

アームは FollowJointTrajectory アクション、ナビゲーションは NavigateToPose
アクションで動かします。どちらも `send_goal_async()` が返す Future を
`spin_until_future_complete()` で待ちます。スレッドは使いません。

画像の保存は別ノード（`image_saver`）が受け持ちます。

★ **干渉チェックはありません。** アーム周辺に人と物が無いことを確かめてから
  動かしてください。4 で実際に走るので、車輪を浮かせるか前方 1m を空けること。
"""

from __future__ import annotations

import math

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = (
    "arm_shoulder_pan_joint",
    "arm_shoulder_lift_joint",
    "arm_elbow_flex_joint",
    "arm_wrist_flex_joint",
    "arm_wrist_roll_joint",
)

# stow（収納）姿勢。手先はおよそ (0.072, 0.000, 0.219) [m]。
STOW = (0.0322, -1.7253, 1.5200, -1.5800, 1.3709)

# 手先がいちばん高くなる姿勢。およそ (0.067, 0.000, 0.429) [m] でほぼ真上。
RAISED = (0.0, -0.2570, -1.2900, 0.0, 0.0)

FORWARD_DISTANCE = 0.5   # 前へ進む距離 [m]
GLOBAL_FRAME = "map"     # ナビゲーション目標を置くフレーム
MOVE_SECONDS = 4.0       # アームが 1 つの姿勢へ移動するのにかける時間 [s]

TRAJECTORY_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"
NAVIGATE_ACTION = "/navigate_to_pose"


class ExampleSequence(Node):
    def __init__(self) -> None:
        super().__init__("example_sequence")

        self._tf = Buffer()
        TransformListener(self._tf, self)

        self._arm = ActionClient(self, FollowJointTrajectory, TRAJECTORY_ACTION)
        self._nav = ActionClient(self, NavigateToPose, NAVIGATE_ACTION)

    def _await(self, future):
        """Future が終わるまで spin する。"""
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def _send_goal(self, client: ActionClient, goal, what: str):
        """アクションのゴールを送り、結果が返るまで待つ。"""
        handle = self._await(client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError(f"{what} のゴールが拒否された")
        return self._await(handle.get_result_async())

    def move_arm(self, positions, label: str) -> None:
        """5 関節を指定の角度へ動かす。"""
        self.get_logger().info(f"アーム -> {label}")
        if not self._arm.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("JTC のアクションサーバが居ない")

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.velocities = [0.0] * len(positions)
        point.time_from_start = Duration(sec=int(MOVE_SECONDS))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        goal.trajectory.points = [point]

        result = self._send_goal(self._arm, goal, f"アーム({label})")
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f"アームが失敗: {result.result.error_string}")

    def move_forward(self) -> None:
        """いま向いている方向へ FORWARD_DISTANCE だけ進む。"""
        self.get_logger().info(f"前方 {FORWARD_DISTANCE:.2f} m へナビゲーション")
        if not self._nav.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Nav2 のアクションサーバが居ない")

        # いまの自分の位置と向きを TF から取る。
        now = rclpy.time.Time()
        self._await(self._tf.wait_for_transform_async(GLOBAL_FRAME, "base_link", now))
        transform = self._tf.lookup_transform(GLOBAL_FRAME, "base_link", now).transform
        rotation = transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2),
        )

        # ★ 目標は map（固定フレーム）で計算する。frame_id を base_link にして
        #   「前へ 0.5m」と書くと、目標が自分に付いて回って Nav2 が収束しない。
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = GLOBAL_FRAME
        goal.pose.pose.position.x = (
            transform.translation.x + FORWARD_DISTANCE * math.cos(yaw)
        )
        goal.pose.pose.position.y = (
            transform.translation.y + FORWARD_DISTANCE * math.sin(yaw)
        )
        goal.pose.pose.orientation = rotation  # 向きは変えない

        # 目標に着いたかどうかは Nav2 の判定に従う。nav2.yaml の
        # xy_goal_tolerance が 0.12 なので、ぴったり 0.5m では止まらない。
        result = self._send_goal(self._nav, goal, "ナビゲーション")
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"ナビゲーションが失敗: status={result.status}")


def main() -> None:
    rclpy.init()
    node = ExampleSequence()
    try:
        node.move_arm(STOW, "stow")
        node.move_arm(RAISED, "上げる")
        node.move_arm(STOW, "stow")
        node.move_forward()
        node.get_logger().info("完了")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""最小構成の例。アームを動かして、前へ進む。

    ros2 run lekiwi_examples example_sequence

★ `robot.launch.py` が動いていることが前提。ハードウェアには直接触らず、
  ROS のインターフェース（アクションと TF）だけを使う。

順番に次をやる。

    1. アームを stow（収納）姿勢へ
    2. アームを上げる
    3. アームを stow へ戻す
    4. 前方 50cm へナビゲーション

★ **画像の保存はここではやらない。** 別ノードのサービスに分けてある。

      ros2 run lekiwi_examples image_saver                          # 常駐
      ros2 service call /image_saver/save std_srvs/srv/Trigger      # 保存

  保存は「撮りたいタイミングで呼ぶ」ものなので、この一直線の手順に混ぜると
  手順を足すたびに保存の位置を考えることになる。呼ぶ側に判断を残した。

────────────────────────────────────────────────────────────────────────
★ スレッドを使わない
────────────────────────────────────────────────────────────────────────
アクションは `send_goal_async()` + `spin_until_future_complete()` で待つ。
**別スレッドで spin しない。** 手順が上から下へ一直線に読め、
どこで待っているかがコードの見た目と一致する。

その代わり **spin していない間は何も受信しない**。TF バッファの更新も
spin の中でしか進まないので、TF も async 版 (`wait_for_transform_async`) を
使う。**待ち方は `_await()` の 1 つだけ**にしてある。

★ ブロックする API（`ActionClient.send_goal()`、`Buffer.lookup_transform()` の
  `timeout` 付き）は**別スレッドが spin していないと永久に待つ**。
  スレッドを使わないなら、それらを呼んではいけない。
  例外は `ActionClient.wait_for_server()` で、これはグラフを直接見る。

────────────────────────────────────────────────────────────────────────
★ 安全上の注意
────────────────────────────────────────────────────────────────────────
* **干渉チェックは一切無い。** 関節空間で補間するので、肘が天板や LiDAR を
  通る経路を取りうる。アーム周辺に人と物が無いことを確かめてから動かすこと。
* ナビゲーションは Nav2 を通すので `collision_monitor` が効く。
  一方 4 の前に**アームを stow へ戻している**のは、伸ばしたまま走ると
  `nav2.yaml` の `robot_radius: 0.17`（収納状態の前提）が嘘になるため。
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

# ★ stow は reach.yaml の実測値を**可動域の内側へ丸めた**もの。
#   実測値そのもの（lift -1.795 など）は URDF の可動域をわずかに外れており、
#   reach ノードも同じように丸めている（「stow_positions を制限内へ丸めた」）。
#   手先はおよそ (0.072, 0.000, 0.219) [m]。
STOW = (0.0322, -1.7253, 1.5200, -1.5800, 1.3709)

# ★ 手先がいちばん高くなる姿勢（可動域内を総当たりして選んだ）。
#   手先はおよそ (0.067, 0.000, 0.429) [m] で、ほぼ真上を向く。
RAISED = (0.0, -0.2570, -1.2900, 0.0, 0.0)

#: 前へ進む距離 [m]
FORWARD_DISTANCE = 0.5
#: ナビゲーション目標を置くフレーム。Nav2 の大域フレームに合わせる。
GLOBAL_FRAME = "map"
#: アームが 1 つの姿勢へ移動するのにかける時間 [s]
MOVE_SECONDS = 4.0

TRAJECTORY_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"
NAVIGATE_ACTION = "/navigate_to_pose"

# ★ ROS パラメータにしていないのは、これが**最小構成の例**だから。
#   実際に運用するノード（reach_to_point / base_driver / teleop_keyboard）は
#   YAML + declare_parameter を使う。理由は docs/development.md。
#   なお --symlink-install なので、ここを書き換えれば再ビルド無しで効く。


class ExampleSequence(Node):
    def __init__(self) -> None:
        super().__init__("example_sequence")

        self._tf = Buffer()
        TransformListener(self._tf, self)

        self._arm = ActionClient(self, FollowJointTrajectory, TRAJECTORY_ACTION)
        self._nav = ActionClient(self, NavigateToPose, NAVIGATE_ACTION)

    # ── 待つ ──────────────────────────────────────────────────────────

    def _await(self, future):
        """future が終わるまで spin する。待ち方はこれ 1 つだけ。"""
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def _send_goal(self, client: ActionClient, goal, what: str):
        """アクションを送り、結果が返るまで待つ。"""
        handle = self._await(client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError(f"{what} のゴールが拒否された")
        return self._await(handle.get_result_async())

    # ── 1〜3. アーム ──────────────────────────────────────────────────

    def move_arm(self, positions, label: str) -> None:
        self.get_logger().info(f"アーム -> {label}")
        # ★ wait_for_server は graph を直接見るので spin 不要（例外的）。
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
            raise RuntimeError(f"アームが失敗: {result.result.error_string}")

    # ── 4. ナビゲーション ─────────────────────────────────────────────

    def move_forward(self) -> None:
        self.get_logger().info(f"前方 {FORWARD_DISTANCE:.2f} m へナビゲーション")
        if not self._nav.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Nav2 のアクションサーバが居ない")

        # ★ `lookup_transform(..., timeout=...)` は使わない。あれは内部で
        #   sleep して待つだけなので、別スレッドが spin していないと
        #   バッファが埋まらず必ずタイムアウトする。async 版を待つ。
        now = rclpy.time.Time()
        self._await(self._tf.wait_for_transform_async(GLOBAL_FRAME, "base_link", now))
        transform = self._tf.lookup_transform(GLOBAL_FRAME, "base_link", now).transform
        rotation = transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2),
        )

        # ★ 目標は**固定フレーム（map）で自分で計算する**。
        #   `frame_id: base_link` に「前へ 0.5m」と書いてはいけない。
        #   目標が自分に付いて回るので Nav2 が収束しない。同じ 0.5m を
        #   モックで比べた実測:
        #       base_link 指定 -> ABORTED  224 秒で 0.123m しか進まない
        #       map で自前計算 -> SUCCEEDED
        #
        #   なお `xy_goal_tolerance: 0.12`（nav2.yaml）なので、
        #   **目標ちょうどには止まらない**。0.5m 指令に対し実測 0.37〜0.39m。
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

        result = self._send_goal(self._nav, goal, "ナビゲーション")
        # ★ 結果を必ず見る。Nav2 は「行けなかった」も返す。
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"ナビゲーションが失敗: status={result.status}")


def main() -> None:
    rclpy.init()
    node = ExampleSequence()

    # ★ executor もスレッドも作らない。待つのは全部 node 側の spin。
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

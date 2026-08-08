"""最小構成の例。アームを動かし、前へ進み、手首カメラの画像を保存する。

    ros2 run lekiwi_examples example_sequence

★ `robot.launch.py` が動いていることが前提。ハードウェアには直接触らず、
  ROS のインターフェース（アクションとトピック）だけを使う。

順番に次をやる。

    1. アームを stow（収納）姿勢へ
    2. アームを上げる
    3. アームを stow へ戻す
    4. 前方 50cm へナビゲーション
    5. 手首カメラの画像を保存（RGB は PNG、depth はグレースケールの JPG）

★ 画像は**購読しっぱなしで最新の 1 枚だけ持つ**。変換と保存は 5 の
  タイミングでまとめてやる。撮りたい瞬間に同期を取る必要が無く、
  「届いていなければ保存しない」も素直に書ける。

★ **`cv_bridge` は使わない。** numpy 2 系では `imgmsg_to_cv2()` が SIGSEGV する
  （import は通るので気付きにくい）。理由と代替は `docs/development.md`。

────────────────────────────────────────────────────────────────────────
★ スレッドを使わない
────────────────────────────────────────────────────────────────────────
アクションは `send_goal_async()` + `spin_until_future_complete()` で待つ。
**別スレッドで spin しない。** 手順が上から下へ一直線に読め、
どこで待っているかがコードの見た目と一致する。

その代わり **spin していない間は何も受信しない**。購読とバッファ更新は
すべて spin の中で進むので、

* TF は `_spin_until()` で `can_transform` になるまで自分で回す
* 画像も同じく、届くまで回してから保存する

★ ブロックする API（`ActionClient.send_goal()`、`Buffer.lookup_transform()` の
  `timeout` 付き）は**別スレッドが spin していないと永久に待つ**。
  スレッドを使わないなら、それらを呼んではいけない。

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
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
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

# sensor_msgs/Image のエンコーディング -> (numpy 型, チャンネル数)
ENCODINGS = {
    "bgr8": (np.uint8, 3),
    "rgb8": (np.uint8, 3),
    "mono8": (np.uint8, 1),
    "mono16": (np.uint16, 1),
    "16UC1": (np.uint16, 1),
    "32FC1": (np.float32, 1),
}


def imgmsg_to_np(message: Image) -> np.ndarray:
    """sensor_msgs/Image -> numpy（OpenCV と同じ BGR 並び）。

    ★ `cv_bridge` の代わり。やっていることはエンコーディングを見て
      `bytes` を numpy へ整形するだけなので、依存なしで書ける。
    """
    dtype, channels = ENCODINGS[message.encoding]
    array = np.frombuffer(message.data, dtype=dtype)
    if channels > 1:
        array = array.reshape(message.height, message.width, channels)
    else:
        array = array.reshape(message.height, message.width)
    # cv2 は BGR を期待する。rgb8 のときだけ入れ替える。
    return array[..., ::-1] if message.encoding == "rgb8" else array


class ExampleSequence(Node):
    def __init__(self) -> None:
        super().__init__("example_sequence")

        defaults = {
            "output_dir": "/captured_images",
            "forward_distance": 0.5,
            # ナビゲーション目標を置くフレーム。Nav2 の大域フレームに合わせる。
            "global_frame": "map",
            "move_seconds": 4.0,
            "color_topic": "/wrist_camera/wrist_camera/color/image_raw",
            "depth_topic": "/wrist_camera/wrist_camera/depth/image_rect_raw",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def param(name):
            return self.get_parameter(name).value

        self._output = Path(str(param("output_dir")))
        self._distance = float(param("forward_distance"))
        self._frame = str(param("global_frame"))
        self._seconds = float(param("move_seconds"))

        self._tf = Buffer()
        TransformListener(self._tf, self)

        self._arm = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self._nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # ★ 最新の 1 枚だけ持つ。保存は最後にまとめてやる。
        self._color: Image | None = None
        self._depth: Image | None = None
        # ★ SENSOR_DATA (BEST_EFFORT)。publisher が RELIABLE でも繋がる。
        self.create_subscription(
            Image, str(param("color_topic")),
            lambda m: setattr(self, "_color", m), qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, str(param("depth_topic")),
            lambda m: setattr(self, "_depth", m), qos_profile_sensor_data,
        )

    # ── 待つ（spin は全部ここに集める）──────────────────────────────

    def _spin_until(self, ready, what: str, timeout: float = 30.0) -> None:
        """`ready()` が True になるまで spin する。

        ★ 別スレッドを使わないので、**購読が進むのは spin の中だけ**。
          「届くまで待つ」は自分で回すしかない。
        """
        limit = time.monotonic() + timeout
        while not ready():
            if time.monotonic() > limit:
                raise RuntimeError(f"{what} を {timeout:.0f} 秒待っても揃わない")
            rclpy.spin_once(self, timeout_sec=0.1)

    def _send_goal(self, client: ActionClient, goal, what: str, timeout: float):
        """アクションを送り、結果が返るまで spin する。

        ★ `send_goal()`（同期版）は使わない。あれは別スレッドが spin して
          いないと永久に待つ。`send_goal_async()` なら自分で回せる。
        """
        accepted = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, accepted, timeout_sec=timeout)
        if not accepted.done():
            raise RuntimeError(f"{what} のゴールに応答が無い")
        handle = accepted.result()
        if not handle.accepted:
            raise RuntimeError(f"{what} のゴールが拒否された")

        finished = handle.get_result_async()
        rclpy.spin_until_future_complete(self, finished, timeout_sec=timeout)
        if not finished.done():
            raise RuntimeError(f"{what} が {timeout:.0f} 秒で終わらない")
        return finished.result()

    # ── 1〜3. アーム ──────────────────────────────────────────────────

    def move_arm(self, positions, label: str) -> None:
        self.get_logger().info(f"アーム -> {label}")
        # ★ wait_for_server は graph を直接見るので spin 不要（例外的）。
        if not self._arm.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("JTC のアクションサーバが居ない")

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.velocities = [0.0] * len(positions)
        point.time_from_start = Duration(sec=int(self._seconds))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        goal.trajectory.points = [point]

        result = self._send_goal(self._arm, goal, f"アーム({label})",
                                 self._seconds + 20.0)
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f"アームが失敗: {result.result.error_string}")

    # ── 4. ナビゲーション ─────────────────────────────────────────────

    def move_forward(self) -> None:
        self.get_logger().info(f"前方 {self._distance:.2f} m へナビゲーション")
        if not self._nav.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Nav2 のアクションサーバが居ない")

        # ★ 目標は**固定フレーム（map）で自分で計算する**。
        #   `frame_id: base_link` に「前へ 0.5m」と書いてはいけない。
        #   目標が自分に付いて回るので Nav2 が収束しない。同じ 0.5m を
        #   モックで比べた実測:
        #       base_link 指定 -> ABORTED  224 秒で 0.123m しか進まない
        #       map で自前計算 -> SUCCEEDED
        #
        #   なお `xy_goal_tolerance: 0.12`（nav2.yaml）なので、
        #   **目標ちょうどには止まらない**。0.5m 指令に対し実測 0.37〜0.39m。
        # ★ `lookup_transform(..., timeout=...)` は使わない。あれは内部で
        #   sleep して待つだけなので、別スレッドが spin していないと
        #   バッファが埋まらず必ずタイムアウトする。自分で回して待つ。
        self._spin_until(
            lambda: self._tf.can_transform(self._frame, "base_link", rclpy.time.Time()),
            f"TF {self._frame} -> base_link",
        )
        transform = self._tf.lookup_transform(
            self._frame, "base_link", rclpy.time.Time()
        ).transform
        rotation = transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2),
        )

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._frame
        goal.pose.pose.position.x = (
            transform.translation.x + self._distance * math.cos(yaw)
        )
        goal.pose.pose.position.y = (
            transform.translation.y + self._distance * math.sin(yaw)
        )
        goal.pose.pose.orientation = rotation  # 向きは変えない

        result = self._send_goal(self._nav, goal, "ナビゲーション", 300.0)
        # ★ 結果を必ず見る。Nav2 は「行けなかった」も返す。
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"ナビゲーションが失敗: status={result.status}")

    # ── 5. 画像 ──────────────────────────────────────────────────────

    def save_images(self) -> None:
        # ここまでの spin で届いているはずだが、届いていなければ待つ。
        self._spin_until(
            lambda: self._color is not None and self._depth is not None,
            "手首カメラの画像",
        )
        self._output.mkdir(parents=True, exist_ok=True)

        color = imgmsg_to_np(self._color)
        # ★ depth は 16UC1 [mm]（0 は無効値）。そのままでは真っ黒なので
        #   最小-最大で 0-255 へ引き伸ばしてグレースケールにする。
        depth = cv2.normalize(
            imgmsg_to_np(self._depth), None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )

        for path, image in (
            (self._output / "example_rgb.png", color),
            (self._output / "example_depth.jpg", depth),
        ):
            if not cv2.imwrite(str(path), image):
                raise RuntimeError(f"保存に失敗: {path}")
            self.get_logger().info(f"保存: {path} {image.shape}")


def main() -> None:
    rclpy.init()
    node = ExampleSequence()

    # ★ executor もスレッドも作らない。待つのは全部 node 側の spin。
    try:
        node.move_arm(STOW, "stow")
        node.move_arm(RAISED, "上げる")
        node.move_arm(STOW, "stow")
        node.move_forward()
        node.save_images()
        node.get_logger().info("完了")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

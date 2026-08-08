"""example_sequence の ROS に依存しない部分を検証する。

★ 検証の主眼は `imgmsg_to_np`。`cv_bridge` を使わない理由
  （numpy 2 で `imgmsg_to_cv2()` が SIGSEGV する。docs/development.md）が
  ある以上、代替が正しいことは自前で担保しないといけない。
"""

import sys
import types

import numpy as np
import pytest

# example_sequence は rclpy 系を import する。実機もコンテナも無しで回すため、
# ダミーの型だけ置いて import を通す（検証するのは純粋な関数だけ）。
for name in (
    "rclpy", "rclpy.action", "rclpy.duration", "rclpy.executors", "rclpy.node",
    "rclpy.qos", "action_msgs.msg", "builtin_interfaces.msg", "control_msgs.action",
    "geometry_msgs.msg", "nav2_msgs.action", "sensor_msgs.msg", "tf2_ros",
    "trajectory_msgs.msg",
):
    sys.modules.setdefault(name, types.ModuleType(name))

sys.modules["rclpy.node"].Node = object
sys.modules["rclpy.action"].ActionClient = object
sys.modules["rclpy.duration"].Duration = object
sys.modules["rclpy.executors"].ExternalShutdownException = type("E", (Exception,), {})
sys.modules["rclpy.executors"].MultiThreadedExecutor = object
sys.modules["rclpy.qos"].qos_profile_sensor_data = object()
for module, attributes in {
    "action_msgs.msg": ("GoalStatus",),
    "builtin_interfaces.msg": ("Duration",),
    "control_msgs.action": ("FollowJointTrajectory",),
    "geometry_msgs.msg": ("PoseStamped",),
    "nav2_msgs.action": ("NavigateToPose",),
    "sensor_msgs.msg": ("Image",),
    "tf2_ros": ("Buffer", "TransformListener"),
    "trajectory_msgs.msg": ("JointTrajectory", "JointTrajectoryPoint"),
}.items():
    for attribute in attributes:
        setattr(sys.modules[module], attribute, object)

from lekiwi_examples.cartesian_math import joint_limits_from_urdf  # noqa: E402
from lekiwi_examples.example_sequence import (  # noqa: E402
    ARM_JOINTS,
    RAISED,
    STOW,
    imgmsg_to_np,
)


class FakeImage:
    def __init__(self, array, encoding):
        self.height, self.width = array.shape[:2]
        self.encoding = encoding
        self.data = array.tobytes()


def test_bgr8はそのまま通す():
    source = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    assert np.array_equal(imgmsg_to_np(FakeImage(source, "bgr8")), source)


def test_rgb8はBGRへ入れ替える():
    """★ cv2.imwrite は BGR 前提。ここを飛ばすと赤と青が入れ替わる。"""
    source = np.zeros((2, 2, 3), np.uint8)
    source[..., 0] = 200  # R
    source[..., 2] = 10   # B
    result = imgmsg_to_np(FakeImage(source, "rgb8"))
    assert result[0, 0, 0] == 10, "B が先頭に来ていない"
    assert result[0, 0, 2] == 200, "R が末尾に来ていない"


def test_16UC1のdepthは2次元で返る():
    """RealSense の depth は 16UC1 [mm]。チャンネル次元を付けない。"""
    source = np.array([[0, 1000], [2000, 65535]], np.uint16)
    result = imgmsg_to_np(FakeImage(source, "16UC1"))
    assert result.shape == (2, 2)
    assert result.dtype == np.uint16
    assert np.array_equal(result, source)


def test_32FC1のdepthも扱える():
    source = np.array([[0.0, 1.5]], np.float32)
    result = imgmsg_to_np(FakeImage(source, "32FC1"))
    assert result.dtype == np.float32
    assert np.array_equal(result, source)


def test_知らないエンコーディングは黙って通さない():
    """★ 黙って壊れた配列を返すより、その場で落ちるほうがよい。"""
    with pytest.raises(KeyError):
        imgmsg_to_np(FakeImage(np.zeros((1, 1), np.uint8), "yuv422"))


def test_姿勢は可動域の内側にある(combined_urdf):
    """★ STOW と RAISED を送ってもコントローラに弾かれないこと。

    reach.yaml の stow 実測値は URDF の可動域をわずかに外れており、
    そのまま送ると危ない。example_sequence の STOW は丸めた値を持つ。
    """
    limits = joint_limits_from_urdf(combined_urdf)
    for label, positions in (("STOW", STOW), ("RAISED", RAISED)):
        assert len(positions) == len(ARM_JOINTS)
        for name, value in zip(ARM_JOINTS, positions):
            lower, upper = limits[name]
            assert lower <= value <= upper, (
                f"{label} の {name}={value} が可動域 [{lower}, {upper}] の外"
            )

"""example_sequence の ROS に依存しない部分を検証する。

★ 検証するのは STOW / RAISED が URDF の可動域に収まっていること。
  外れた値を送るとコントローラに弾かれるか、勝手に丸められる。
"""

import sys
import types


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
from lekiwi_examples.example_sequence import ARM_JOINTS, RAISED, STOW  # noqa: E402


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

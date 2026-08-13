"""Nav2 なしで RViz の 2D Goal Pose へ寄る。

    # ① ベース + LiDAR (Nav2 は起動しない)
    ros2 launch lekiwi_base_bringup nav.launch.py start_nav2:=false start_slam:=false

    # ② こちら
    ros2 launch lekiwi_examples goal_drive.launch.py

★ Nav2 と同時に起動しないこと。/cmd_vel を取り合う。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lekiwi_examples",
            executable="goal_drive",
            name="goal_drive",
            output="screen",
        ),
    ])

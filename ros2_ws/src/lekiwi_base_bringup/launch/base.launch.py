"""LeKiwi ベースの起動。

    ros2 launch lekiwi_base_bringup base.launch.py
    ros2 launch lekiwi_base_bringup base.launch.py dry_run:=true      # 実機なし検証
    ros2 launch lekiwi_base_bringup base.launch.py port:=/dev/ttyACM0

キーボード操作はこの launch には含まれない。teleop_twist_keyboard は
`ros2 launch` 配下では stdin を取れないため、別ターミナルから起動すること:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -p speed:=0.1 -p turn:=0.5
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    hardware_backend = LaunchConfiguration("hardware_backend")
    dry_run = LaunchConfiguration("dry_run")
    start_rviz = LaunchConfiguration("start_rviz")
    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")
    use_mesh = LaunchConfiguration("use_mesh")
    config_file = LaunchConfiguration("config_file")
    rviz_config = LaunchConfiguration("rviz_config")

    bringup_share = Path(get_package_share_directory("lekiwi_base_bringup"))
    description_share = Path(get_package_share_directory("lekiwi_description"))
    xacro_file = description_share / "urdf" / "lekiwi_base.urdf.xacro"

    # ParameterValue(..., value_type=str) は Jazzy では必須。
    # 無いと Command の出力が文字列として扱われず robot_state_publisher が落ちる。
    robot_description = ParameterValue(
        Command(["xacro ", str(xacro_file), " use_mesh:=", use_mesh]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/lekiwi"),
        DeclareLaunchArgument("hardware_backend", default_value="serial"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        # /robot_description は TRANSIENT_LOCAL / depth 1 なので、publisher が
        # 2 つあると後から繋いだ購読者がどちらの latch を掴むか非決定になる
        # (CLAUDE.md の「RViz に別のロボットが出る」症状)。アームを載せた
        # 合成 launch は自分でロボット全体の RSP を持つので false にする。
        DeclareLaunchArgument("start_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument("use_mesh", default_value="false"),
        DeclareLaunchArgument(
            "config_file",
            default_value=str(bringup_share / "config" / "base.yaml"),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(bringup_share / "rviz" / "base.rviz"),
            description="RViz config file (absolute path).",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            condition=IfCondition(start_robot_state_publisher),
        ),

        Node(
            package="lekiwi_base_bringup",
            executable="base_driver",
            name="lekiwi_base_driver",
            output="screen",
            parameters=[
                config_file,
                {
                    "port": port,
                    "dry_run": dry_run,
                    "hardware_backend": hardware_backend,
                },
            ],
        ),

        # sllidar_ros2 は frame_id="laser" で publish する。URDF の laser_link と
        # 同一位置を指すため、ゼロオフセットの static transform で接続する。
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="laser_frame_bridge",
            arguments=["--frame-id", "laser_link", "--child-frame-id", "laser"],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(start_rviz),
            output="screen",
        ),
    ])

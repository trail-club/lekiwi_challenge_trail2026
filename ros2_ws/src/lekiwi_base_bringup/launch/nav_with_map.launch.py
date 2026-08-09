"""保存済みの地図を使って AMCL 自己位置推定 + Nav2 ナビゲーションを動かす launch。

    ros2 launch lekiwi_base_bringup nav_with_map.launch.py map_file:=/maps/my_room.yaml

■ 使い方の流れ

  1. nav.launch.py (SLAM モード) で地図を作る
  2. 地図を保存する:
       ros2 run nav2_map_server map_saver_cli -f /maps/my_room
       → /maps/my_room.yaml と /maps/my_room.pgm が生成される
  3. nav.launch.py を停止する
  4. この launch を起動する:
       ros2 launch lekiwi_base_bringup nav_with_map.launch.py map_file:=/maps/my_room.yaml
  5. RViz の "2D Pose Estimate" で初期自己位置を与える
  6. RViz の "Nav2 Goal" でゴールを与えるとナビゲーションが始まる

■ nav.launch.py との違い
  * slam_toolbox の代わりに nav2_map_server (地図配信) + amcl (自己位置推定) を使う
  * 既知の地図内を繰り返しナビゲーションするときに向いている
  * 地図は更新されない (SLAM モードに戻すには nav.launch.py を使う)

■ AMCL の初期収束について
  初期自己位置が大きくずれていると AMCL が収束しない。
  RViz の "2D Pose Estimate" で大まかな位置と向きを与えてから動かすこと。
  収束後は走り回るにつれて自動的に精度が上がる。

■ start_lidar フラグ (nav.launch.py と共通)
  * start_lidar:=true  (既定): このプロセス内で sllidar_node を起動する
  * start_lidar:=false       : /scan を外部 (rplidar コンテナ等) に任せる
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("lekiwi_base_bringup"))
    description_share = Path(get_package_share_directory("lekiwi_description"))
    nav2_bringup_share = Path(get_package_share_directory("nav2_bringup"))

    xacro_file = description_share / "urdf" / "lekiwi_base.urdf.xacro"

    port = LaunchConfiguration("port")
    hardware_backend = LaunchConfiguration("hardware_backend")
    serial_port = LaunchConfiguration("serial_port")
    map_file = LaunchConfiguration("map_file")
    start_rviz = LaunchConfiguration("start_rviz")
    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")
    start_lidar = LaunchConfiguration("start_lidar")
    base_params = LaunchConfiguration("base_params_file")
    nav2_params = LaunchConfiguration("nav2_params_file")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description = ParameterValue(
        Command(["xacro ", str(xacro_file), " use_mesh:=false"]), value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/lekiwi",
                              description="LeKiwiベースのシリアルポート"),
        DeclareLaunchArgument("hardware_backend", default_value="serial"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0",
                              description="RPLIDAR A1 のシリアルポート (start_lidar:=true 時のみ使用)"),
        DeclareLaunchArgument(
            "map_file",
            description="地図 YAML ファイルのパス。nav.launch.py + map_saver_cli で生成したもの。",
        ),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        # /robot_description は TRANSIENT_LOCAL / depth 1 なので、publisher が
        # 2 つあると後から繋いだ購読者がどちらの latch を掴むか非決定になる
        # (CLAUDE.md の「RViz に別のロボットが出る」症状)。アームを載せた
        # 合成 launch は自分でロボット全体の RSP を持つので false にする。
        DeclareLaunchArgument("start_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument(
            "start_lidar", default_value="true",
            description="true: このプロセス内で sllidar_node を起動する。"
                        "false: /scan を外部 (rplidar コンテナ等) に任せる。",
        ),
        DeclareLaunchArgument(
            "base_params_file",
            default_value=str(bringup_share / "config" / "base.yaml"),
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=str(bringup_share / "config" / "nav2.yaml"),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(nav2_bringup_share / "rviz" / "nav2_default_view.rviz"),
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
                base_params,
                {"port": port, "hardware_backend": hardware_backend},
            ],
        ),

        # /scan を -60°〜+60° (右車輪〜左車輪の前方アーク) にフィルタリングして
        # /scan_filtered として配信する。
        Node(
            package="lekiwi_base_bringup",
            executable="scan_filter",
            name="scan_angular_filter",
            output="screen",
        ),

        # nav.launch.py と同じ: frame_id は laser_link 固定
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            output="screen",
            parameters=[{
                "channel_type": "serial",
                "serial_port": serial_port,
                "serial_baudrate": 115200,
                "frame_id": "laser_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard",
            }],
            condition=IfCondition(start_lidar),
        ),

        # bringup_launch.py は map_server + amcl + navigation 全ノードを起動する。
        # nav.launch.py が「slam_toolbox + navigation_launch.py」の組み合わせで
        # SLAM を担当したのに対し、ここでは既存の地図と amcl で自己位置を推定する。
        #
        # ★ bringup_launch.py が起動する amcl は nav2.yaml の amcl セクションを使う。
        #   robot_model_type: "nav2_amcl::OmniMotionModel" がオムニ向けに設定済み。
        #
        # ★ amcl は起動直後には自己位置が未確定。RViz の "2D Pose Estimate" で
        #   初期位置を与えるまでナビゲーションを開始しないこと。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(nav2_bringup_share / "launch" / "bringup_launch.py")
            ),
            launch_arguments={
                "map": map_file,
                "params_file": nav2_params,
                "use_sim_time": "false",
                "autostart": "true",
            }.items(),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(start_rviz),
        ),
    ])

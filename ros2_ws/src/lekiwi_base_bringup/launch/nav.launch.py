"""実機で SLAM + Nav2 を動かす launch。

    ros2 launch lekiwi_base_bringup nav.launch.py
    ros2 launch lekiwi_base_bringup nav.launch.py port:=/dev/ttyACM0

構成:

    base_driver (実機)  ← /cmd_vel ← Nav2 controller
        ↓ odom, TF (odom → base_footprint)
    sllidar_node  → /scan (frame_id: laser_link)  ← start_lidar:=true の場合のみ
        ↓
    slam_toolbox  → TF (map → odom) と /map
        ↓
    Nav2 (planner + MPPI controller)  → /cmd_vel

■ sim_nav.launch.py との違い
  * base_driver は dry_run ではなく実機のシリアルポートへ繋ぐ
  * fake_scan の代わりに実機 RPLIDAR A1 を使う
  * SLAM / Nav2 の設定ファイルは共通 (config/slam_toolbox.yaml, config/nav2.yaml)

■ start_lidar フラグ
  * start_lidar:=true (既定) : このプロセス内で sllidar_node を起動する。
    ネイティブ ROS 環境や単体テスト向け。sllidar_ros2 パッケージが必要。
  * start_lidar:=false       : /scan の提供を外部に任せる。
    Docker Compose (compose.nav.yaml) では rplidar コンテナが担当するためこちら。

■ 起動前チェック (Phase D 完了が前提)
  * base.yaml の wheel_direction_signs / wheel_angle_offset_deg が
    実機合わせ済みであること。未確定のままだと Nav2 が正しい /cmd_vel を
    出しても機体が意図しない方向へ走る。
  * URDF の laser_link 位置が実測値に更新されていること。
    TBD の仮値 (z=0.09) のままだと地図とコストマップが歪む。

■ ゴールの与え方
  RViz の "2D Goal Pose" ツール、または:

    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\
      '{header: {frame_id: map}, pose: {position: {x: 1.5, y: 1.0}, \\
        orientation: {w: 1.0}}}'

■ 地図の保存
  start_slam:=true のときは map_saver_server + 専用 lifecycle_manager を
  自動起動しているので、以下のどちらかで保存できる:

    ros2 run lekiwi_base_bringup save_map [名前]   # /map_saver/save_map を呼ぶ
    ros2 run nav2_map_server map_saver_cli -f ~/map/my_room \\
      --ros-args -p save_map_timeout:=10.0
      # ★ 既定の save_map_timeout (2.0s) だとこの環境では discovery が
      #   間に合わず "Failed to spin map subscription" で失敗することがある
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
    start_rviz = LaunchConfiguration("start_rviz")
    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")
    start_slam = LaunchConfiguration("start_slam")
    start_nav2 = LaunchConfiguration("start_nav2")
    start_lidar = LaunchConfiguration("start_lidar")
    base_params = LaunchConfiguration("base_params_file")
    slam_params = LaunchConfiguration("slam_params_file")
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
        DeclareLaunchArgument("start_rviz", default_value="true"),
        # /robot_description は TRANSIENT_LOCAL / depth 1 なので、publisher が
        # 2 つあると後から繋いだ購読者がどちらの latch を掴むか非決定になる
        # (CLAUDE.md の「RViz に別のロボットが出る」症状)。アームを載せた
        # 合成 launch は自分でロボット全体の RSP を持つので false にする。
        DeclareLaunchArgument("start_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_nav2", default_value="true"),
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
            "slam_params_file",
            default_value=str(bringup_share / "config" / "slam_toolbox.yaml"),
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=str(bringup_share / "config" / "nav2.yaml"),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(nav2_bringup_share / "rviz" / "nav2_default_view.rviz"),
            description="RViz 設定ファイル。Nav2 公式設定を既定にしている。",
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
        # /scan_filtered として配信する。コストマップ・slam_toolbox・collision_monitor
        # がこちらを購読する。後方車輪やボディが /scan に映ると地図に幻の壁が焼き付く
        # ため、SLAM も含めてフィルタ済みスキャンを使う。
        Node(
            package="lekiwi_base_bringup",
            executable="scan_filter",
            name="scan_angular_filter",
            output="screen",
        ),

        # RPLIDAR A1 を直接起動する (start_lidar:=true の場合のみ)。
        # sllidar_ros2 パッケージに直接依存し、rplidar_bringup には依存しない。
        # これにより Docker イメージに rplidar_bringup が入っていなくても
        # compose.nav.yaml (start_lidar:=false) で起動できる。
        #
        # ★ frame_id は laser_link 固定。これが URDF のリンク名と一致することで
        #   /scan → TF ツリー → costmap の経路が繋がる。"laser" のままだと
        #   base.launch.py の static_transform_publisher が無い限り costmap が空になる。
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

        # ★ slam_toolbox は Jazzy では LifecycleNode。ふつうの Node として起動すると
        #   unconfigured のまま止まり、map → odom の TF が出ない。
        #   configure → activate のイベント発行は公式 launch が持っているので、
        #   自前で書かずに include する。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    Path(get_package_share_directory("slam_toolbox"))
                    / "launch"
                    / "online_async_launch.py"
                )
            ),
            launch_arguments={
                "slam_params_file": slam_params,
                "use_sim_time": "false",
                "autostart": "true",
                "use_lifecycle_manager": "false",
            }.items(),
            condition=IfCondition(start_slam),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(nav2_bringup_share / "launch" / "navigation_launch.py")
            ),
            launch_arguments={
                "params_file": nav2_params,
                "use_sim_time": "false",
                "autostart": "true",
            }.items(),
            condition=IfCondition(start_nav2),
        ),

        # ★ navigation_launch.py は /map_saver/save_map サービスを提供しない
        #   (controller/planner/behavior 系のみ)。地図保存には nav2_map_server の
        #   map_saver_server を別途起動する必要がある。専用の lifecycle_manager で
        #   configure/activate しないと "サービスが見つかりません" のまま止まる。
        Node(
            package="nav2_map_server",
            executable="map_saver_server",
            name="map_saver",
            output="screen",
            parameters=[{
                "save_map_timeout": 10.0,
                "free_thresh_default": 0.25,
                "occupied_thresh_default": 0.65,
            }],
            condition=IfCondition(start_slam),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map_saver",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_saver"],
            }],
            condition=IfCondition(start_slam),
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

"""実機なしで SLAM + Nav2 を閉ループで回す launch。

    ros2 launch lekiwi_base_bringup sim_nav.launch.py

構成:

    base_driver (dry_run)  ← /cmd_vel ← Nav2 controller
        ↓ odom, TF (odom → base_footprint)
    fake_scan  ← TF を読んで仮想の部屋へレイキャスト
        ↓ /scan
    slam_toolbox  → TF (map → odom) と /map
        ↓
    Nav2 (planner + MPPI controller)  → /cmd_vel

シリアルも LiDAR も使わないので、実機が無い状態で「走らせる → 地図ができる →
ゴールを与えると経路を追従する」までを検証できる。

■ 実機との違い (ここで検証できないこと)

  * dry_run の odom は指令値の積分なので **スリップも外乱も無い**。
    既定では odom が真値そのものなので、スキャンマッチングは自明に成功する。
    意味のある検証にするには odom に系統誤差を入れる:

        ros2 launch lekiwi_base_bringup sim_nav.launch.py odom_trans_scale:=1.03

  * 実機の Phase D (車輪の回転方向・前方向・鏡像の確定) は代替できない。
    ここが未確定のままだと、Nav2 が正しい /cmd_vel を出しても機体は違う方向へ走る。

  * laser_link の取付位置は未実測 (URDF の仮値 z=0.09)。実機では実測値に
    差し替えること。ここがずれると地図が歪む。

■ ゴールの与え方

  RViz の "2D Goal Pose" ツール、または:

    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\
      '{header: {frame_id: map}, pose: {position: {x: 1.5, y: 1.0}, \\
        orientation: {w: 1.0}}}'
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

    start_rviz = LaunchConfiguration("start_rviz")
    hardware_backend = LaunchConfiguration("hardware_backend")
    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")
    start_slam = LaunchConfiguration("start_slam")
    start_nav2 = LaunchConfiguration("start_nav2")
    base_params = LaunchConfiguration("base_params_file")
    scan_params = LaunchConfiguration("fake_scan_params_file")
    slam_params = LaunchConfiguration("slam_params_file")
    nav2_params = LaunchConfiguration("nav2_params_file")
    rviz_config = LaunchConfiguration("rviz_config")

    # ParameterValue(..., value_type=...) は Jazzy では必須。
    # 無いと launch 引数の文字列が double として渡らずノードが落ちる。
    odom_trans_scale = ParameterValue(
        LaunchConfiguration("odom_trans_scale"), value_type=float
    )
    odom_yaw_scale = ParameterValue(
        LaunchConfiguration("odom_yaw_scale"), value_type=float
    )
    robot_description = ParameterValue(
        Command(["xacro ", str(xacro_file), " use_mesh:=false"]), value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("hardware_backend", default_value="serial"),
        # /robot_description は TRANSIENT_LOCAL / depth 1 なので、publisher が
        # 2 つあると後から繋いだ購読者がどちらの latch を掴むか非決定になる
        # (CLAUDE.md の「RViz に別のロボットが出る」症状)。アームを載せた
        # 合成 launch は自分でロボット全体の RSP を持つので false にする。
        DeclareLaunchArgument("start_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_nav2", default_value="true"),
        DeclareLaunchArgument(
            "base_params_file", default_value=str(bringup_share / "config" / "base.yaml")
        ),
        DeclareLaunchArgument(
            "fake_scan_params_file",
            default_value=str(bringup_share / "config" / "fake_scan.yaml"),
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
            # Nav2 公式の RViz 設定を流用する。地図・costmap・経路・ゴールツールが
            # 揃っており、手書きの .rviz を保守するより確実。
            default_value=str(
                nav2_bringup_share / "rviz" / "nav2_default_view.rviz"
            ),
        ),
        # odom の系統誤差。1.0 = 誤差なし (= SLAM が自明に成功する)。
        DeclareLaunchArgument("odom_trans_scale", default_value="1.0"),
        DeclareLaunchArgument("odom_yaw_scale", default_value="1.0"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            condition=IfCondition(start_robot_state_publisher),
        ),

        # ★ dry_run を固定する。この launch はシリアルを一切触らない。
        Node(
            package="lekiwi_base_bringup",
            executable="base_driver",
            name="lekiwi_base_driver",
            output="screen",
            parameters=[
                base_params,
                {"dry_run": True, "hardware_backend": hardware_backend},
            ],
        ),

        Node(
            package="lekiwi_base_bringup",
            executable="fake_scan",
            name="fake_scan",
            output="screen",
            parameters=[
                scan_params,
                {
                    "odom_trans_scale": odom_trans_scale,
                    "odom_yaw_scale": odom_yaw_scale,
                },
            ],
        ),

        # ★ scan_filter が無いと SLAM も costmap も一切データを受け取れない。
        #   fake_scan が出すのは /scan だが、slam_toolbox.yaml の scan_topic も
        #   nav2.yaml の costmap も /scan_filtered を読む。この 1 ノードが
        #   欠けていると /scan_filtered は publisher 0 / subscriber 3 になり、
        #   map -> odom が永遠に出ず、Nav2 は "Invalid frame ID map" を
        #   INFO で吐き続ける (エラーではないので気付きにくい)。
        #   nav.launch.py と nav_with_map.launch.py には元からある。
        Node(
            package="lekiwi_base_bringup",
            executable="scan_filter",
            name="scan_angular_filter",
            output="screen",
        ),

        # ★ slam_toolbox は Jazzy では LifecycleNode。ふつうの Node として起動すると
        #   unconfigured のまま止まり、map → odom の TF が出ないので Nav2 が
        #   "Invalid frame ID map" を吐き続ける (エラーではなく INFO なので気付きにくい)。
        #   configure → activate のイベント発行は公式 launch が持っているので、
        #   自前で書かずにそれを include する。
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
                # ★ 公式 launch の既定は true。Gazebo を使わないので false 必須。
                "use_sim_time": "false",
                "autostart": "true",
                "use_lifecycle_manager": "false",
            }.items(),
            condition=IfCondition(start_slam),
        ),

        # Nav2 本体は公式の navigation_launch.py に任せる。
        # lifecycle manager の起動順序を自前で書くと壊しやすい。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(nav2_bringup_share / "launch" / "navigation_launch.py")
            ),
            launch_arguments={
                "params_file": nav2_params,
                # ★ Gazebo を使わないので system clock。true にすると
                #   /clock を待って全ノードが固まる。
                "use_sim_time": "false",
                "autostart": "true",
            }.items(),
            condition=IfCondition(start_nav2),
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

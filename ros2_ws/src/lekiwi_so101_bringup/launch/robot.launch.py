"""ロボット全体を 1 プロセスで起動する launch。

    # 実機 (docker/robot のコンテナ内で)
    ros2 launch lekiwi_so101_bringup robot.launch.py \
      motor_bus_mode:=split backend:=lerobot robot_id:=my_follower

    # 実機なし (Mac でも動く)
    ros2 launch lekiwi_so101_bringup robot.launch.py motor_bus_mode:=split sim:=true

    # 保存済み地図 + AMCL
    ros2 launch lekiwi_so101_bringup robot.launch.py motor_bus_mode:=split \\
      backend:=lerobot robot_id:=my_follower use_saved_map:=true \\
      map_file:=/maps/my_room.yaml

────────────────────────────────────────────────────────────────────────
起動するもの
────────────────────────────────────────────────────────────────────────
    robot_state_publisher (結合 URDF。システム全体でこれ 1 つ)  ┐ arm.launch.py
    LeRobot ブリッジ + ros2_control + spawner                   │ を include
    RViz (システム全体でこれ 1 つ)                              ┘

    base_driver + scan_filter                                   ┐ nav.launch.py /
    sllidar_node (sim:=true なら fake_scan)                     │ sim_nav.launch.py /
    slam_toolbox または map_server+amcl / Nav2 / map_saver      ┘ nav_with_map.launch.py

    realsense2_camera (手首カメラ)                                d435i.launch.py

★ arm.launch.py との違い
  arm.launch.py は**アーム側だけ** (RSP + ブリッジ + ros2_control + RViz)。
  この launch がそれを include したうえでベース・LiDAR・カメラも起動する。
  アームだけを切り分けて確認したいときは arm.launch.py を直接使える。

★ リーチと逆運動学は起動しない。lekiwi_examples へ移した。
  この launch はロボットを「動かせる状態」にするところまでで、その上で
  何をするかはアプリケーション側の責任にする。別ターミナルで:

      ros2 launch lekiwi_examples reach.launch.py      # map 上の点へリーチ
      ros2 run lekiwi_examples teleop_keyboard         # キーボード操作

★ 停止
  この launch を Ctrl+C する。アームのトルクが切れて落ちるので、
  lekiwi_examples の reach.launch.py も起動している場合は、先に
  `ros2 service call /so101/stow std_srvs/srv/Trigger {}` で畳んでおくこと。
  SIGKILL された場合の復帰は `ros2 run lekiwi_so101_bringup release_all`。

★ include を GroupAction で包む理由
  launch の設定値は既定で共有スコープに入る。included な launch が同名の引数
  (start_rviz, rviz_config, serial_port ...) を宣言すると、その既定値が
  こちらの設定を**上書きする**。GroupAction (scoped=True) で閉じ込める。
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PythonExpression,
)

MOUNT_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


def _validate_motor_bus_mode(context):
    mode = LaunchConfiguration("motor_bus_mode").perform(context)
    if mode not in ("split", "shared"):
        raise RuntimeError("motor_bus_mode must be explicitly set to split or shared")
    return []


def generate_launch_description():
    share = Path(get_package_share_directory("lekiwi_so101_bringup"))
    base_launch_dir = Path(get_package_share_directory("lekiwi_base_bringup")) / "launch"
    camera_launch_dir = Path(get_package_share_directory("realsense_bringup")) / "launch"

    sim = LaunchConfiguration("sim")
    use_saved_map = LaunchConfiguration("use_saved_map")
    map_file = LaunchConfiguration("map_file")
    lekiwi_port = LaunchConfiguration("lekiwi_port")
    lidar_port = LaunchConfiguration("lidar_port")
    start_lidar = LaunchConfiguration("start_lidar")
    start_base = LaunchConfiguration("start_base")
    start_camera = LaunchConfiguration("start_camera")
    start_rviz = LaunchConfiguration("start_rviz")

    backend = LaunchConfiguration("backend")
    motor_bus_mode = LaunchConfiguration("motor_bus_mode")
    robot_id = LaunchConfiguration("robot_id")
    usb_port = LaunchConfiguration("usb_port")
    split_calibration_dir = LaunchConfiguration("split_calibration_dir")
    shared_calibration_dir = LaunchConfiguration("shared_calibration_dir")
    joint_prefix = LaunchConfiguration("joint_prefix")

    arm_port = PythonExpression([
        "'", lekiwi_port, "' if '", motor_bus_mode,
        "' == 'shared' else '", usb_port, "'",
    ])
    calibration_dir = PythonExpression([
        "'", shared_calibration_dir, "' if '", motor_bus_mode,
        "' == 'shared' else '", split_calibration_dir, "'",
    ])
    hardware_backend = PythonExpression([
        "'bridge' if '", motor_bus_mode,
        "' == 'shared' else 'serial'",
    ])

    wrist_camera = LaunchConfiguration("wrist_camera")
    wrist_camera_name = LaunchConfiguration("wrist_camera_name")
    wrist_camera_parent = LaunchConfiguration("wrist_camera_parent")
    wrist_camera_fps = LaunchConfiguration("wrist_camera_fps")
    wrist_camera_decimation = LaunchConfiguration("wrist_camera_decimation")
    wrist_camera_reconnect_timeout = LaunchConfiguration(
        "wrist_camera_reconnect_timeout"
    )
    mock_wrist_camera_optical = LaunchConfiguration("mock_wrist_camera_optical")

    # ベース側 3 通りの排他条件。
    #   sim:=true                          -> sim_nav.launch.py   (dry_run + fake_scan)
    #   sim:=false かつ use_saved_map:=false -> nav.launch.py       (SLAM)
    #   sim:=false かつ use_saved_map:=true  -> nav_with_map.launch.py (AMCL)
    real = NotSubstitution(sim)
    use_slam = AndSubstitution(real, NotSubstitution(use_saved_map))
    use_amcl = AndSubstitution(real, use_saved_map)

    return LaunchDescription([
        # ───────── 全体 ─────────
        DeclareLaunchArgument(
            "sim", default_value="false",
            description="true: シリアルも LiDAR も開かない (base_driver は dry_run、"
                        "スキャンは fake_scan、アームは backend:=mock を推奨)"),
        DeclareLaunchArgument("start_base", default_value="true",
                              description="false にするとベースとナビを起動しない"),
        DeclareLaunchArgument("start_camera", default_value="true",
                              description="realsense2_camera を起動するか。"
                                          "★ sim:=true では自動的に起動しない"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument(
            "motor_bus_mode",
            description="REQUIRED: split (two ports) or shared (one canonical ID 1-9 bus)",
        ),
        OpaqueFunction(function=_validate_motor_bus_mode),

        # ───────── ベース ─────────
        DeclareLaunchArgument("lekiwi_port", default_value="/dev/lekiwi"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/rplidar"),
        DeclareLaunchArgument(
            "start_lidar", default_value="true",
            description="実機の sllidar_node を起動するか。sim:=true では無視される"),
        DeclareLaunchArgument(
            "use_saved_map", default_value="false",
            description="true: slam_toolbox の代わりに map_server + AMCL を使う"),
        DeclareLaunchArgument(
            "map_file", default_value="",
            description="use_saved_map:=true のとき必須。例 /maps/my_room.yaml"),

        # ───────── アーム ─────────
        DeclareLaunchArgument(
            "backend", default_value="mock",
            description="mock: シリアルを開かない / lerobot: 実機の SO-101"),
        DeclareLaunchArgument(
            "robot_id", default_value="",
            description="backend:=lerobot では必須の LeRobot 較正 ID"),
        DeclareLaunchArgument("usb_port", default_value="/dev/so101_follower"),
        DeclareLaunchArgument(
            "arm_torque", default_value="true",
            description="false: アームのトルクを入れず指令も書かない。"
                        "手で動かして /joint_states を読むとき"),
        DeclareLaunchArgument(
            "split_calibration_dir",
            default_value="/root/.cache/huggingface/lerobot/calibration/robots/so_follower"),
        DeclareLaunchArgument(
            "shared_calibration_dir",
            default_value="/root/.cache/huggingface/lerobot/calibration/robots/lekiwi"),
        DeclareLaunchArgument("joint_prefix", default_value="arm_"),

        # ───────── 手首カメラ ─────────
        # ★ WRIST_CAMERA_* は「URDF にリンクを生やすか」と「その取付姿勢」。
        #   既定は空文字 = 指定なしで、URDF 側の既定値が唯一の情報源になる。
        DeclareLaunchArgument(
            "wrist_camera", default_value=os.environ.get("WRIST_CAMERA", "true")),
        DeclareLaunchArgument(
            "wrist_camera_name",
            default_value=os.environ.get("WRIST_CAMERA_NAME", "wrist_camera"),
            description="★ realsense の camera_name と URDF の両方に効く。"
                        "この launch は同じ値を両方へ渡すので不一致は起きない"),
        DeclareLaunchArgument(
            "wrist_camera_parent",
            default_value=os.environ.get("WRIST_CAMERA_PARENT", "gripper_link"),
            description="接頭辞なしで書く (joint_prefix が前置される)"),
        *(DeclareLaunchArgument(
            f"wrist_camera_{key}",
            default_value=os.environ.get(f"WRIST_CAMERA_{key.upper()}", ""),
            description="空なら URDF の既定値を使う")
          for key in MOUNT_KEYS),
        DeclareLaunchArgument(
            "wrist_camera_fps", default_value=os.environ.get("WRIST_CAMERA_FPS", "6")),
        DeclareLaunchArgument(
            "wrist_camera_decimation",
            default_value=os.environ.get("WRIST_CAMERA_DECIMATION", "2")),
        DeclareLaunchArgument(
            "wrist_camera_reconnect_timeout",
            default_value=os.environ.get(
                "WRIST_CAMERA_RECONNECT_TIMEOUT", "6.0"
            ),
            description="RealSense切断後の再接続試行間隔 [s]",
        ),
        # ★ 実機では realsense2_camera がこの TF を出すので起動しないこと。
        #   sim:=true でカメラ実機が無いときだけ true にする。
        DeclareLaunchArgument("mock_wrist_camera_optical", default_value="false"),

        # ───────── アーム + RSP + リーチ + RViz ─────────
        # ★ arm.launch.py がシステム全体で唯一の robot_state_publisher と
        #   RViz を持つ。ベース側の include には
        #   start_robot_state_publisher:=false / start_rviz:=false を渡すこと。
        GroupAction(actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / "launch" / "arm.launch.py")),
                launch_arguments=[
                    ("backend", backend),
                    ("motor_bus_mode", motor_bus_mode),
                    ("robot_id", robot_id),
                    ("usb_port", arm_port),
                    # robot.launch.py 側は arm_torque。ベースにもトルクがあるので
                    # どちらの話か分かる名前にしてある。下位は torque。
                    ("torque", LaunchConfiguration("arm_torque")),
                    ("calibration_dir", calibration_dir),
                    ("joint_prefix", joint_prefix),
                    ("start_rviz", start_rviz),
                    ("wrist_camera", wrist_camera),
                    ("wrist_camera_name", wrist_camera_name),
                    ("wrist_camera_parent", wrist_camera_parent),
                    ("mock_wrist_camera_optical", mock_wrist_camera_optical),
                    *((f"wrist_camera_{key}", LaunchConfiguration(f"wrist_camera_{key}"))
                      for key in MOUNT_KEYS),
                ],
            ),
        ]),

        # ───────── ベース: 実機 + SLAM ─────────
        GroupAction(
            condition=IfCondition(AndSubstitution(start_base, use_slam)),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(base_launch_dir / "nav.launch.py")),
                    launch_arguments=[
                        ("port", lekiwi_port),
                        ("hardware_backend", hardware_backend),
                        ("serial_port", lidar_port),
                        ("start_lidar", start_lidar),
                        ("start_robot_state_publisher", "false"),
                        ("start_rviz", "false"),
                    ],
                ),
            ],
        ),

        # ───────── ベース: 実機 + 保存済み地図 + AMCL ─────────
        GroupAction(
            condition=IfCondition(AndSubstitution(start_base, use_amcl)),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(base_launch_dir / "nav_with_map.launch.py")),
                    launch_arguments=[
                        ("port", lekiwi_port),
                        ("hardware_backend", hardware_backend),
                        ("serial_port", lidar_port),
                        ("map_file", map_file),
                        ("start_lidar", start_lidar),
                        ("start_robot_state_publisher", "false"),
                        ("start_rviz", "false"),
                    ],
                ),
            ],
        ),

        # ───────── ベース: 実機なし ─────────
        # ★ base_driver は dry_run 固定、スキャンは fake_scan。
        #   シリアルも LiDAR も一切開かないので Mac でも動く。
        GroupAction(
            condition=IfCondition(AndSubstitution(start_base, sim)),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(base_launch_dir / "sim_nav.launch.py")),
                    launch_arguments=[
                        ("hardware_backend", hardware_backend),
                        ("start_robot_state_publisher", "false"),
                        ("start_rviz", "false"),
                    ],
                ),
            ],
        ),

        # ───────── 手首カメラ ─────────
        # ★ sim:=true では起動しない。USB カメラが無い環境で
        #   realsense2_camera を上げても "No RealSense devices were found" を
        #   吐き続けるだけで、模擬光学フレームは mock_wrist_camera_optical が担う。
        GroupAction(
            condition=IfCondition(AndSubstitution(start_camera, NotSubstitution(sim))),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(camera_launch_dir / "d435i.launch.py")),
                    launch_arguments=[
                        # ★ camera_name は TF のフレーム名にも効く。
                        #   結合 URDF の wrist_camera_name と同じ値をここから渡すので、
                        #   4 コンテナ構成で起きていた「compose の env と URDF の
                        #   不一致」は原理的に起きない。
                        ("camera_name", wrist_camera_name),
                        ("camera_namespace", wrist_camera_name),
                        ("enable_pointcloud", "true"),
                        ("depth_fps", wrist_camera_fps),
                        ("color_fps", wrist_camera_fps),
                        ("decimation", wrist_camera_decimation),
                        ("reconnect_timeout", wrist_camera_reconnect_timeout),
                        ("start_rviz", "false"),
                    ],
                ),
            ],
        ),

        # ★ sim:=true では start_camera:=true を無視する。黙って無視すると
        #   「カメラを上げたつもり」になるので、ログに理由を出す。
        LogInfo(
            condition=IfCondition(AndSubstitution(start_camera, sim)),
            msg="sim:=true なので realsense2_camera は起動しません。"
                "光学フレームだけ要るなら mock_wrist_camera_optical:=true。",
        ),
    ])

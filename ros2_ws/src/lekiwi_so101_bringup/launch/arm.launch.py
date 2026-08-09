"""結合ロボット (LeKiwi ベース + SO-101 アーム) のアーム側 launch。

この launch が起動するもの:

  1. **結合ロボット全体**の robot_state_publisher (システム全体でこれ 1 つだけ)
  2. アーム側 (LeRobot ブリッジ + ros2_control + spawner) — follower.launch.py を include
  3. RViz (システム全体でこれ 1 つだけ)

★ リーチと逆運動学はここでは起動しない。lekiwi_examples へ移した。
  この launch (と robot.launch.py) はロボットを動かせる状態にするところまでで、
  その上で何をするかはアプリケーション側の責任にする。

      ros2 launch lekiwi_examples reach.launch.py     # 別ターミナルで

★ ベース側 (base_driver / scan_filter / slam_toolbox / Nav2) はここでは起動しない。

  ★ **ふだんは robot.launch.py を使うこと。** あちらがこの launch を include した
    うえでベースとカメラも起動するので、ロボット全体が 1 プロセスで上がる
    (docker/robot の統合イメージには nav2 も lekiwi_base_bringup も入っている)。

  この launch を単体で使うのは、**アームだけを切り分けて確認したいとき**。
  ベース側が要るなら robot.launch.py を使うか、別途

      ros2 launch lekiwi_base_bringup nav.launch.py \
        start_robot_state_publisher:=false start_rviz:=false

  を起動する (RSP はこの launch が唯一持つので false が要る)。

★ なぜ RSP をこちらが持つのか
  結合 URDF は lekiwi_description と so_arm101_description の両方を必要とし、
  両方を $(find) できるのはこのイメージだけ。xacro の $(find) はローカルの
  ファイルシステムを見るので、DDS では橋渡しできない。
  /robot_description は TRANSIENT_LOCAL / depth 1 なので publisher が 2 つあると
  後から繋いだ購読者がどちらの latch を掴むか非決定になる
  (CLAUDE.md の「RViz に別のロボットが出る」症状)。
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from so101_bringup.calibration_limits import build_robot_description


def generate_launch_description():
    share = Path(get_package_share_directory("lekiwi_so101_bringup"))
    arm_launch_dir = Path(get_package_share_directory("so101_bringup")) / "launch"
    xacro_file = share / "urdf" / "lekiwi_so101.urdf.xacro"

    backend = LaunchConfiguration("backend")
    torque = LaunchConfiguration("torque")
    joint_prefix = LaunchConfiguration("joint_prefix")
    usb_port = LaunchConfiguration("usb_port")
    robot_id = LaunchConfiguration("robot_id")
    calibration_dir = LaunchConfiguration("calibration_dir")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    controllers_file = LaunchConfiguration("controllers_file")

    mount_keys = ("x", "y", "z", "roll", "pitch", "yaw")
    wrist_camera = LaunchConfiguration("wrist_camera")
    wrist_camera_name = LaunchConfiguration("wrist_camera_name")
    wrist_camera_parent = LaunchConfiguration("wrist_camera_parent")
    mock_wrist_camera_optical = LaunchConfiguration("mock_wrist_camera_optical")

    def setup_robot_state_publisher(context, *, xacro_file: Path):
        prefix = joint_prefix.perform(context)
        mappings = {
            "use_mesh": "false",
            "prefix": prefix,
            "usb_port": usb_port.perform(context),
        }
        for key in mount_keys:
            mappings[f"arm_mount_{key}"] = LaunchConfiguration(
                f"arm_mount_{key}"
            ).perform(context)
        mappings.update(
            {
                "wrist_camera": wrist_camera.perform(context),
                "wrist_camera_name": wrist_camera_name.perform(context),
                "wrist_camera_parent": wrist_camera_parent.perform(context),
            }
        )
        # ★ 空文字は「指定なし」の意味で、xacro へ渡さない。
        #   常に渡していると、環境変数が無いときのフォールバック値が
        #   **URDF の既定値を上書きしてしまう**。以前ここが "0.0" だったため、
        #   .env が無いとカメラが gripper_link 原点に重なり、
        #   幾何計算で入れた初期値がまったく効いていなかった。
        #   渡さなければ URDF の既定値が使われる（＝ URDF が唯一の情報源）。
        for key in mount_keys:
            value = LaunchConfiguration(f"wrist_camera_{key}").perform(context)
            if value != "":
                mappings[f"wrist_camera_{key}"] = value

        robot_description = build_robot_description(
            xacro_file,
            mappings=mappings,
            calibration_dir=calibration_dir.perform(context),
            robot_id=robot_id.perform(context),
            backend=backend.perform(context),
        )
        return [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="mock",
                description="mock: シリアルを開かない / lerobot: 実機の SO-101",
            ),
            DeclareLaunchArgument(
                "joint_prefix",
                default_value="arm_",
                description=(
                    "結合 URDF・controllers_file・リーチノードで必ず一致させること。"
                    "上流マクロはリンク名だけでなく関節名にも適用する"
                ),
            ),
            DeclareLaunchArgument("usb_port", default_value="/dev/so101_follower"),
            DeclareLaunchArgument(
                "torque", default_value="true",
                description="false: トルクを入れず指令も書かない。"
                            "手でアームを動かして /joint_states を読むとき"),
            DeclareLaunchArgument(
                "robot_id",
                default_value="",
                description="backend:=lerobot では必須の LeRobot 較正 ID",
            ),
            DeclareLaunchArgument(
                "calibration_dir",
                default_value=(
                    "/root/.cache/huggingface/lerobot/calibration/robots/so_follower"
                ),
                description="LeRobot calibration JSON directory",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=str(share / "config" / "ros2_controllers.yaml"),
            ),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config", default_value=str(share / "rviz" / "reach.rviz")
            ),
            # ★ 手首カメラ (RealSense)。取付は arm_gripper_link。
            #   点群を map 上に置くのに必要なのは URDF のこの繋ぎだけで、
            #   外部キャリブレーションは要らない (カメラが剛体固定されているため)。
            #   既定値は環境変数から読む (.env -> compose の environment -> ここ)。
            DeclareLaunchArgument(
                "wrist_camera",
                default_value=os.environ.get("WRIST_CAMERA", "true"),
                description="URDF に手首カメラのリンクを生やすか"),
            DeclareLaunchArgument(
                "wrist_camera_name",
                default_value=os.environ.get("WRIST_CAMERA_NAME", "wrist_camera"),
                description="★ realsense の camera_name と必ず一致させること"),
            DeclareLaunchArgument(
                "wrist_camera_parent",
                default_value=os.environ.get("WRIST_CAMERA_PARENT", "gripper_link"),
                description="接頭辞なしで書く (joint_prefix が前置される)"),
            # ★ 実機では realsense2_camera がこの TF を /tf_static へ出すので
            #   起動しないこと (二重定義になる)。カメラ実機が無い Mac で
            #   map -> ..._depth_optical_frame の疎通と軸の向きを検証するため。
            DeclareLaunchArgument(
                "mock_wrist_camera_optical", default_value="false"),
            # ★ 取付姿勢は未実測。RViz でマゼンタのマーカーとして見える。
            # ★ 既定は空文字＝「指定なし」。URDF の既定値（幾何計算による初期値）が
            #   そのまま効く。.env で WRIST_CAMERA_* を設定したときだけ上書きする。
            #   ここに数値を書くと URDF と二重管理になり、必ず片方が古くなる。
            *(DeclareLaunchArgument(
                f"wrist_camera_{key}",
                default_value=os.environ.get(f"WRIST_CAMERA_{key.upper()}", ""),
                description="空なら URDF の既定値を使う")
              for key in mount_keys),
            # arm_mount_link から見たアーム基部の姿勢。既定の 0 は「補正なし」。
            #   ★ arm_mount_link 自体は**実測済み** (2026-08-07)。
            #     base_link から (0.08, 0.00, 0.057), rpy=0。
            #     CAD の y=-0.04 は誤りで、実測は y=0 だった (docs/agent/report.md)。
            *(
                DeclareLaunchArgument(f"arm_mount_{key}", default_value="0.0")
                for key in mount_keys
            ),
            # ---- システム全体で唯一の robot_state_publisher ----
            OpaqueFunction(
                function=setup_robot_state_publisher,
                kwargs={"xacro_file": xacro_file},
            ),
            # ---- アーム側 ----
            # follower.launch.py の起動順序 (bridge 待ち -> control_node ->
            # 直列 spawner -> bridge 終了で shutdown) をそのまま再利用する。
            # follower.launch.py also declares `start_rviz`. Keep its
            # start_rviz:=false local to the include; otherwise it overwrites
            # the composed launch's RViz setting in the shared launch context.
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            str(arm_launch_dir / "follower.launch.py")
                        ),
                        launch_arguments=[
                            ("start_robot_state_publisher", "false"),
                            ("start_rviz", "false"),
                            # ★ ブリッジが落ちてもこの launch service は止めない。
                            #   止めると同居している唯一の robot_state_publisher も死に、
                            #   別コンテナの slam_toolbox / Nav2 が
                            #   base_footprint -> laser_link を失って測位できなくなる。
                            #   アームの故障をアームだけに閉じ込める。
                            ("shutdown_on_bridge_exit", "false"),
                            ("backend", backend),
                            ("torque", torque),
                            ("usb_port", usb_port),
                            ("robot_id", robot_id),
                            ("calibration_dir", calibration_dir),
                            ("description_file", str(xacro_file)),
                            ("joint_prefix", joint_prefix),
                            ("controllers_file", controllers_file),
                        ],
                    )
                ]
            ),
            # ★ Mac 検証用の模擬光学フレーム。REP-103 のボディ座標 (x前 y左 z上)
            #   から光学座標 (x右 y下 z前) への変換は rpy = (-pi/2, 0, -pi/2)。
            #   realsense2_camera が出すものと同じ。
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="wrist_camera_optical_tf_mock",
                output="screen",
                condition=IfCondition(mock_wrist_camera_optical),
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "-1.5707963", "--pitch", "0", "--yaw", "-1.5707963",
                    "--frame-id", [wrist_camera_name, "_link"],
                    "--child-frame-id", [wrist_camera_name, "_depth_optical_frame"],
                ],
            ),
            # ---- システム全体で唯一の RViz ----
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(start_rviz),
                output="screen",
            ),
        ]
    )

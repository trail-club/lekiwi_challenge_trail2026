"""Launch the SO-101 with ros2_control over a thin LeRobot bridge."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from so101_bringup.calibration_limits import build_robot_description


def _setup_actions(context, *, bringup_share: Path):
    backend = LaunchConfiguration("backend").perform(context)
    motor_bus_mode = LaunchConfiguration("motor_bus_mode").perform(context)
    usb_port = LaunchConfiguration("usb_port").perform(context)
    robot_id = LaunchConfiguration("robot_id").perform(context)
    calibration_dir = LaunchConfiguration("calibration_dir").perform(context)
    controllers_file = LaunchConfiguration("controllers_file").perform(context)
    base_params_file = LaunchConfiguration("base_params_file").perform(context)
    description_file = LaunchConfiguration("description_file").perform(context)
    joint_prefix = LaunchConfiguration("joint_prefix").perform(context)
    torque = LaunchConfiguration("torque").perform(context).lower() == "true"
    control_file = bringup_share / "control" / "so101_follower.ros2_control.xacro"

    wheel_safety_params = {}
    if motor_bus_mode == "shared":
        if not base_params_file:
            base_params_file = str(
                Path(get_package_share_directory("lekiwi_base_bringup"))
                / "config"
                / "base.yaml"
            )
        try:
            base_data = yaml.safe_load(Path(base_params_file).read_text())
            base_params = base_data["/**"]["ros__parameters"]
            wheel_safety_params = {
                name: base_params[name]
                for name in (
                    "motor_ids",
                    "max_ticks",
                    "cmd_vel_timeout_s",
                    "servo_acceleration",
                    "diagnostics_period_s",
                )
            }
        except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
            raise RuntimeError(
                f"invalid shared wheel safety parameter file: {base_params_file}"
            ) from exc

    # The upstream description still requires legacy xacro argument names. The
    # selected control file ignores their values and always uses the ROS topic
    # hardware adapter; backend selection belongs only to the bridge.
    #
    # Expand and calibrate the description before creating any node.  This keeps
    # RSP, ros2_control, and the bridge on the same calibration-derived model.
    robot_description = ParameterValue(
        build_robot_description(
            description_file,
            mappings={
                "ros2_control_hardware_type": "real",
                "usb_port": usb_port,
                "ros2_control_file": str(control_file),
                "prefix": joint_prefix,
            },
            calibration_dir=calibration_dir,
            robot_id=robot_id,
            backend=backend,
            motor_bus_mode=motor_bus_mode,
        ),
        value_type=str,
    )

    bridge = Node(
        package="so101_bringup",
        executable="so101_lerobot_bridge",
        name="lerobot_bridge",
        output="screen",
        parameters=[
            {
                **wheel_safety_params,
                "backend": backend,
                "motor_bus_mode": motor_bus_mode,
                "torque": torque,
                "usb_port": usb_port,
                "robot_id": robot_id,
                "calibration_dir": calibration_dir,
                "robot_description": robot_description,
                "joint_prefix": joint_prefix,
                "update_rate": 50.0,
                "command_timeout": 0.5,
            }
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_file],
        remappings=[("~/robot_description", "/robot_description")],
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    jsb = spawner("joint_state_broadcaster")
    jtc = spawner("joint_trajectory_controller")
    gripper = spawner("gripper_controller")

    # `ros2 service call` waits for the service to appear. The service is
    # created only after calibration validation and backend connection finish.
    wait_for_bridge = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/so101/lerobot_bridge/ready",
            "std_srvs/srv/Trigger",
            "{}",
        ],
        output="screen",
    )

    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")
    shutdown_on_bridge_exit = LaunchConfiguration("shutdown_on_bridge_exit")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            condition=IfCondition(start_robot_state_publisher),
        ),
        bridge,
        wait_for_bridge,
        RegisterEventHandler(
            OnProcessExit(target_action=wait_for_bridge, on_exit=[control_node])
        ),
        # Spawners wait for controller_manager, then are serialized to avoid
        # competing load/configure requests during startup.
        jsb,
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[jtc])),
        RegisterEventHandler(OnProcessExit(target_action=jtc, on_exit=[gripper])),
        RegisterEventHandler(
            OnProcessExit(
                target_action=bridge,
                on_exit=[EmitEvent(event=Shutdown(reason="LeRobot bridge exited"))],
            ),
            # ★ Shutdown は launch service 全体 -- include 元も含めて -- を落とす。
            #   アーム単体ならそれでよいが、合成構成では同じ launch service に
            #   **結合ロボット唯一の robot_state_publisher** が居る。そのままだと
            #   シリアルが一度こけてブリッジが fault した瞬間に
            #   base_footprint -> base_link -> laser_link の TF が消え、
            #   別コンテナの slam_toolbox と Nav2 が測位を失う
            #   (ベースのコンテナは restart:"no" なので自力復帰しない)。
            #   アームの故障がロボット全体の故障に波及しないよう、
            #   合成側は false にして RSP を生かしたままにする。
            condition=IfCondition(shutdown_on_bridge_exit),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(start_rviz),
            output="screen",
        ),
    ]


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("so101_bringup"))
    description_share = Path(get_package_share_directory("so_arm101_description"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="mock",
                description="mock: no serial access / lerobot: physical SO-101",
            ),
            DeclareLaunchArgument(
                "motor_bus_mode",
                default_value="split",
                description="split: separate arm/base ports; shared: one nine-motor bus",
            ),
            DeclareLaunchArgument("usb_port", default_value="/dev/so101_follower"),
            DeclareLaunchArgument(
                "torque", default_value="true",
                description="false: トルクを入れず指令も書かない。"
                            "手でアームを動かして /joint_states を読むとき"),
            DeclareLaunchArgument(
                "robot_id",
                default_value="",
                description="LeRobot calibration ID; required for backend:=lerobot",
            ),
            DeclareLaunchArgument(
                "calibration_dir",
                default_value=(
                    "/root/.cache/huggingface/lerobot/calibration/robots/so_follower"
                ),
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=str(bringup_share / "config" / "ros2_controllers.yaml"),
            ),
            DeclareLaunchArgument(
                "base_params_file",
                default_value="",
                description="Shared wheel safety settings (max ticks/watchdog/acceleration)",
            ),
            DeclareLaunchArgument(
                "description_file",
                default_value=str(
                    description_share / "urdf" / "so_arm101.urdf.xacro"
                ),
                description="Top-level xacro; override to launch a composed robot",
            ),
            DeclareLaunchArgument(
                "joint_prefix",
                default_value="",
                description=(
                    "Prefix applied to every link AND joint name by the upstream "
                    "macro; must match the controllers file"
                ),
            ),
            # /robot_description is TRANSIENT_LOCAL with depth 1. A second
            # publisher makes late-joining subscribers latch either sample
            # non-deterministically, which is the "RViz shows a different robot"
            # symptom recorded in CLAUDE.md. A composed launch starts its own
            # publisher for the whole robot and sets this to false.
            DeclareLaunchArgument(
                "start_robot_state_publisher", default_value="true"
            ),
            # false にするとブリッジが落ちても launch 全体は止めない。
            # 合成構成 (lekiwi_so101_bringup) で使う。理由は下の条件を参照。
            DeclareLaunchArgument(
                "shutdown_on_bridge_exit", default_value="true"
            ),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(description_share / "rviz" / "config.rviz"),
            ),
            OpaqueFunction(
                function=_setup_actions,
                kwargs={
                    "bringup_share": bringup_share,
                },
            ),
        ]
    )

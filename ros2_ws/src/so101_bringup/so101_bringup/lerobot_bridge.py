"""Thin ROS 2 bridge between ros2_control JointState topics and LeRobot."""

from __future__ import annotations

import sys
import threading
import time

import rclpy
from lekiwi_hardware_interfaces.msg import WheelCommand
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .bridge_core import (
    ROS_JOINTS,
    CommandWatchdog,
    InvalidJointCommand,
    VelocityEstimator,
    gripper_limit_from_urdf,
    lerobot_to_ros,
    ordered_ros_positions,
    ros_to_lerobot,
)
from .lerobot_backend import (
    LeRobotLeKiwiBackend,
    LeRobotSO101Backend,
    MockLeKiwiBackend,
    MockSO101Backend,
)


class SO101LeRobotBridge(Node):
    def __init__(self) -> None:
        super().__init__("lerobot_bridge")
        self.declare_parameter("backend", "mock")
        self.declare_parameter("motor_bus_mode", "split")
        self.declare_parameter("usb_port", "/dev/so101_follower")
        self.declare_parameter("robot_id", "")
        self.declare_parameter(
            "calibration_dir",
            "/root/.cache/huggingface/lerobot/calibration/robots/so_follower",
        )
        # false: 起動時にトルクを入れず、指令も書かない。手でアームを動かして
        # /joint_states で角度を読むための姿勢。読み出しはトルクに依存しない。
        self.declare_parameter("torque", True)
        self.declare_parameter("update_rate", 50.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("robot_description", "")
        # The upstream xacro macro applies `prefix` to joint names as well as
        # link names, so a composed robot (arm on the LeKiwi base) sends
        # e.g. arm_shoulder_pan_joint. Everything below the ROS boundary uses
        # unprefixed canonical names, so strip on input and reapply on output.
        self.declare_parameter("joint_prefix", "")
        # These names match base.yaml so the same parameter file remains the
        # only source for wheel limits and recovery behavior.
        self.declare_parameter("motor_ids", [7, 8, 9])
        self.declare_parameter("max_ticks", 3000)
        self.declare_parameter("cmd_vel_timeout_s", 0.5)
        self.declare_parameter("servo_acceleration", 20)
        self.declare_parameter("diagnostics_period_s", 5.0)

        update_rate = float(self.get_parameter("update_rate").value)
        if update_rate <= 0:
            raise ValueError("update_rate must be positive")
        robot_description = str(self.get_parameter("robot_description").value)
        self._joint_prefix = str(self.get_parameter("joint_prefix").value)
        self._gripper_limit = gripper_limit_from_urdf(
            robot_description, self._joint_prefix
        )
        self._watchdog = CommandWatchdog(
            float(self.get_parameter("command_timeout").value)
        )
        self._wheel_watchdog = CommandWatchdog(
            float(self.get_parameter("cmd_vel_timeout_s").value)
        )
        self._velocity = VelocityEstimator()
        self._command_lock = threading.Lock()
        self._bus_lock = threading.Lock()
        self._command: dict[str, float] | None = None
        self._wheel_command = (0, 0, 0)
        self._wheel_timed_out = False
        self._last_cycle_duration = 0.0
        self._fault: Exception | None = None
        self._closed = False
        self._shutdown_requested = False

        torque = bool(self.get_parameter("torque").value)
        if not torque:
            self.get_logger().warn(
                "torque:=false でアームのトルクを入れません。指令も書きません "
                "(手で動かして /joint_states を読むための姿勢です)"
            )

        backend_name = str(self.get_parameter("backend").value)
        self._motor_bus_mode = str(self.get_parameter("motor_bus_mode").value)
        if self._motor_bus_mode not in ("split", "shared"):
            raise ValueError("motor_bus_mode must be either 'split' or 'shared'")
        motor_ids = [int(value) for value in self.get_parameter("motor_ids").value]
        if self._motor_bus_mode == "shared" and motor_ids != [7, 8, 9]:
            raise ValueError("shared motor bus requires canonical wheel IDs [7, 8, 9]")
        self._wheel_max_ticks = int(self.get_parameter("max_ticks").value)
        if self._wheel_max_ticks <= 0:
            raise ValueError("max_ticks must be positive")
        self._shared_bus = self._motor_bus_mode == "shared"
        if backend_name == "mock":
            backend_type = MockLeKiwiBackend if self._shared_bus else MockSO101Backend
            self._backend = backend_type(torque=torque)
        elif backend_name == "lerobot":
            backend_type = LeRobotLeKiwiBackend if self._shared_bus else LeRobotSO101Backend
            kwargs = {
                "port": str(self.get_parameter("usb_port").value),
                "robot_id": str(self.get_parameter("robot_id").value),
                "calibration_dir": str(self.get_parameter("calibration_dir").value),
                "torque": torque,
            }
            if self._shared_bus:
                kwargs["wheel_acceleration"] = int(
                    self.get_parameter("servo_acceleration").value
                )
            self._backend = backend_type(**kwargs)
        else:
            raise ValueError("backend must be either 'mock' or 'lerobot'")

        self._backend.connect()

        # ★ タイマ (シリアル I/O) とサブスクリプションを別グループに置き、
        #   MultiThreadedExecutor で回す。
        #
        #   両方をノード既定の MutuallyExclusiveCallbackGroup に入れると、
        #   _control_cycle がシリアル I/O でブロックしている間
        #   _command_callback が一切呼ばれない。lerobot のパケットタイムアウトは
        #   1パケット約 50ms、sync_read は6モータを逐次読むので、通信が一度こけると
        #   1サイクルで 300ms 前後ブロックしうる。それが2回続くと、指令は正常に
        #   届いているのに watchdog が期限切れになる (誤発火)。
        #   グループを分ければ、I/O 中でも指令の受信と mark() が進む。
        self._io_group = MutuallyExclusiveCallbackGroup()
        self._command_group = MutuallyExclusiveCallbackGroup()

        self._state_publisher = self.create_publisher(
            JointState, "/so101/hardware_states", qos_profile_sensor_data
        )
        self.create_subscription(
            JointState,
            "/so101/hardware_commands",
            self._command_callback,
            1,
            callback_group=self._command_group,
        )
        if self._shared_bus:
            self.create_subscription(
                WheelCommand,
                "/lekiwi/hardware_wheel_commands",
                self._wheel_command_callback,
                1,
                callback_group=self._command_group,
            )
            self.create_service(
                Trigger,
                "/lekiwi_base_driver/recover",
                self._recover_wheels,
                callback_group=self._io_group,
            )
        self.create_service(
            Trigger,
            "/so101/lerobot_bridge/ready",
            self._ready,
            callback_group=self._command_group,
        )
        self.create_service(
            Trigger,
            "/so101/lerobot_bridge/shutdown",
            self._shutdown,
            callback_group=self._command_group,
        )
        self.create_timer(
            1.0 / update_rate, self._control_cycle, callback_group=self._io_group
        )
        diagnostics_period = float(self.get_parameter("diagnostics_period_s").value)
        if self._shared_bus and diagnostics_period > 0.0:
            self.create_timer(
                diagnostics_period,
                self._diagnostics,
                callback_group=self._io_group,
            )
        self._period = 1.0 / update_rate
        self.get_logger().info(f"SO-101 bridge ready with backend={backend_name}")

    @property
    def faulted(self) -> bool:
        return self._fault is not None

    def _ready(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = not self.faulted
        response.message = "ready" if response.success else str(self._fault)
        return response

    def _shutdown(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Close LeRobot after the Trigger response has been sent.

        ``docker compose down`` only signals the container's PID 1; the
        launch process started by ``docker compose exec`` is a separate
        process.  Exposing shutdown as a ROS service lets the Makefile request
        the same backend disconnect path as Ctrl+C, including torque-off.
        The short timer gives the service server time to return its response
        before the context is shut down.
        """
        response.success = True
        response.message = "shutting down"
        if not self._shutdown_requested:
            self._shutdown_requested = True
            self.create_timer(
                0.1,
                self._shutdown_after_response,
                callback_group=self._io_group,
            )
        return response

    def _shutdown_after_response(self) -> None:
        self.close()
        if rclpy.ok():
            rclpy.shutdown()

    def _strip_prefix(self, names) -> list[str]:
        """Drop the configured prefix; leave unprefixed names for the validator.

        A name that lacks the prefix is passed through untouched so
        ordered_ros_positions reports it as unknown, which names the offending
        joint instead of failing with a confusing count mismatch.
        """
        if not self._joint_prefix:
            return list(names)
        width = len(self._joint_prefix)
        return [
            name[width:] if name.startswith(self._joint_prefix) else name
            for name in names
        ]

    def _command_callback(self, message: JointState) -> None:
        try:
            ordered = ordered_ros_positions(
                self._strip_prefix(message.name), message.position
            )
            command = ros_to_lerobot(ordered, self._gripper_limit)
        except InvalidJointCommand as exc:
            # ros2_control initializes command interfaces to NaN until the
            # controllers have received the first finite hardware state.  It
            # is safe to discard that startup sample (nothing is sent to the
            # servos), but logging it at ERROR for every 50 Hz sample obscures
            # the real startup result. Other malformed commands remain errors.
            if str(exc) == "joint positions must all be finite":
                self.get_logger().warning(
                    "Ignoring non-finite startup hardware command",
                    throttle_duration_sec=1.0,
                )
            else:
                self.get_logger().error(f"Rejected hardware command: {exc}")
            return
        with self._command_lock:
            self._command = command
        self._watchdog.mark()

    def _wheel_command_callback(self, message: WheelCommand) -> None:
        ticks = tuple(int(value) for value in message.ticks)
        if len(ticks) != 3 or any(abs(value) > self._wheel_max_ticks for value in ticks):
            self.get_logger().error(
                f"Rejected wheel command {ticks}; expected three values within "
                f"±{self._wheel_max_ticks}"
            )
            return
        with self._command_lock:
            self._wheel_command = ticks
        self._wheel_watchdog.mark()
        self._wheel_timed_out = False

    def _recover_wheels(self, _request: Trigger.Request, response: Trigger.Response):
        try:
            with self._bus_lock:
                self._backend.recover_wheels()
            with self._command_lock:
                self._wheel_command = (0, 0, 0)
            response.success = True
            response.message = "wheel velocity mode and torque restored"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
        return response

    def _diagnostics(self) -> None:
        try:
            with self._bus_lock:
                diagnostics = self._backend.read_wheel_diagnostics()
            for motor, values in diagnostics.items():
                if "error" in values:
                    self.get_logger().warning(f"{motor}: {values['error']}")
                    continue
                message = (
                    f"{motor}: {values.get('voltage', 0.0):.1f}V "
                    f"{values.get('temperature', 0.0):.0f}C "
                    f"load={values.get('load', 0.0)}"
                )
                if (
                    values.get("voltage", 12.0) < 9.0
                    or values.get("temperature", 0.0) > 60.0
                ):
                    self.get_logger().warning(message)
                else:
                    self.get_logger().debug(message)
        except Exception as exc:  # diagnostics must not interrupt control
            self.get_logger().warning(f"shared bus diagnostics failed: {exc}")

    def _control_cycle(self) -> None:
        started = time.monotonic()
        try:
            if self._watchdog.expired():
                # 誤発火の切り分けに必要なので、原因の手掛かりを必ず残す。
                raise TimeoutError(
                    "hardware command watchdog expired "
                    f"(timeout={self._watchdog.timeout:.3f}s, "
                    f"last cycle={self._last_cycle_duration * 1e3:.1f}ms, "
                    f"period={self._period * 1e3:.1f}ms)"
                )
            with self._command_lock:
                command = self._command
                wheel_command = self._wheel_command
            if self._shared_bus and self._wheel_watchdog.expired():
                wheel_command = (0, 0, 0)
                if not self._wheel_timed_out:
                    self.get_logger().warning(
                        "wheel command watchdog expired; sending zero velocity"
                    )
                    self._wheel_timed_out = True
            with self._bus_lock:
                if command is not None:
                    self._backend.write_positions(command)
                if self._shared_bus:
                    self._backend.write_wheel_ticks(wheel_command)
                ros_positions = lerobot_to_ros(
                    self._backend.read_positions(), self._gripper_limit
                )
            velocities = self._velocity.update(ros_positions, time.monotonic())
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = [f"{self._joint_prefix}{name}" for name in ROS_JOINTS]
            message.position = [ros_positions[name] for name in ROS_JOINTS]
            message.velocity = [velocities[name] for name in ROS_JOINTS]
            self._state_publisher.publish(message)
            self._last_cycle_duration = time.monotonic() - started
            # 周期を超えたサイクルは executor を圧迫する。watchdog 誤発火の
            # 前兆なので記録する (throttle 付きでログを溢れさせない)。
            if self._last_cycle_duration > self._period:
                self.get_logger().warning(
                    f"control cycle overran: {self._last_cycle_duration * 1e3:.1f}ms "
                    f"> period {self._period * 1e3:.1f}ms",
                    throttle_duration_sec=1.0,
                )
        except Exception as exc:  # noqa: BLE001 - any I/O error must safe-stop
            # Ctrl+C shuts down the launch process and its worker threads at
            # nearly the same time. If the main thread has already closed the
            # backend, do not turn that normal teardown race into a hardware
            # fault or a misleading FATAL log.
            if self._closed or not rclpy.ok():
                return
            self._fault = exc
            self.get_logger().fatal(f"SO-101 bridge fault: {exc}")
            self.close()
            if rclpy.ok():
                rclpy.shutdown()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        with self._bus_lock:
            self._backend.disconnect()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = SO101LeRobotBridge()
        # ★ SingleThreadedExecutor だとシリアル I/O 中に指令を受け取れず、
        #   watchdog が誤発火する。グループを分けた意味が出るのは複数スレッド時のみ。
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.remove_node(node)
        if node.faulted:
            exit_code = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        # ★ Ctrl+C は正常終了。ここで捕まえないと KeyboardInterrupt が
        #   そのまま抜けて、停止のたびにトレースバックと
        #   "process has died [exit code -2]" がログに出る。
        #   トルク OFF は下の finally が担うので安全性には影響しないが、
        #   **正常に切れたのか異常終了したのかを操作者が判断できなくなる**のが問題。
        #   base_driver.py と cartesian_jog.py と同じ扱いに揃える。
        #
        # ★ ExternalShutdownException も一緒に拾う。これは Exception の
        #   サブクラスなので、下の except Exception に落ちると事実と違う
        #   "Failed to start" を出して exit 1 になる。このノードは fault 時に
        #   自分で rclpy.shutdown() を呼ぶ (制御ループの例外処理) ので、
        #   spin() がこれを投げる経路は実在する。
        #   ★ 実際に fault していたかは node.faulted で別途判定する。
        if node is not None and node.faulted:
            exit_code = 1
    except Exception as exc:  # noqa: BLE001 - process boundary reports startup faults
        print(f"Failed to start SO-101 LeRobot bridge: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()

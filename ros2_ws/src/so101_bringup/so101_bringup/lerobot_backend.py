"""LeRobot and in-memory backends used by the ROS bridge."""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

from .bridge_core import LEROBOT_JOINTS

LOGGER = logging.getLogger(__name__)

LEKIWI_ARM_NAMES = tuple(f"arm_{name}" for name in LEROBOT_JOINTS)
LEKIWI_WHEEL_NAMES = ("base_left_wheel", "base_back_wheel", "base_right_wheel")
LEKIWI_NAMES = LEKIWI_ARM_NAMES + LEKIWI_WHEEL_NAMES


class SO101Backend(Protocol):
    def connect(self) -> None: ...
    def read_positions(self) -> dict[str, float]: ...
    def write_positions(self, positions: dict[str, float]) -> None: ...
    def disconnect(self) -> None: ...


class SharedLeKiwiBackend(SO101Backend, Protocol):
    def write_wheel_ticks(self, ticks: tuple[int, int, int]) -> None: ...
    def recover_wheels(self) -> None: ...
    def read_wheel_diagnostics(self) -> dict[str, dict[str, float]]: ...


@dataclass
class MockSO101Backend:
    """Position-following backend that never imports ROS or opens a serial port."""

    positions: dict[str, float] = field(
        default_factory=lambda: dict.fromkeys(LEROBOT_JOINTS, 0.0)
    )
    connected: bool = False
    fault: Exception | None = None
    # 実機と同じ意味。torque=False では指令を受け取っても姿勢が変わらない。
    torque: bool = True

    def connect(self) -> None:
        self.connected = True

    def read_positions(self) -> dict[str, float]:
        if not self.connected:
            raise RuntimeError("mock backend is not connected")
        if self.fault is not None:
            raise self.fault
        return dict(self.positions)

    def write_positions(self, positions: dict[str, float]) -> None:
        if not self.connected:
            raise RuntimeError("mock backend is not connected")
        if self.fault is not None:
            raise self.fault
        if set(positions) != set(LEROBOT_JOINTS):
            raise ValueError("mock backend requires all SO-101 joints")
        if not self.torque:
            return
        self.positions = {name: float(positions[name]) for name in LEROBOT_JOINTS}

    def inject_fault(self, fault: Exception) -> None:
        """Make subsequent I/O fail for ROS-independent safety-path tests."""
        self.fault = fault

    def disconnect(self) -> None:
        self.connected = False


@dataclass
class MockLeKiwiBackend(MockSO101Backend):
    """Nine-motor mock used to exercise the shared-bus ROS path."""

    wheel_ticks: tuple[int, int, int] = (0, 0, 0)
    recover_count: int = 0

    def write_wheel_ticks(self, ticks: tuple[int, int, int]) -> None:
        if not self.connected:
            raise RuntimeError("mock backend is not connected")
        if self.fault is not None:
            raise self.fault
        if len(ticks) != 3:
            raise ValueError("shared mock requires exactly three wheel ticks")
        self.wheel_ticks = tuple(int(value) for value in ticks)

    def recover_wheels(self) -> None:
        if not self.connected:
            raise RuntimeError("mock backend is not connected")
        self.wheel_ticks = (0, 0, 0)
        self.recover_count += 1

    def read_wheel_diagnostics(self) -> dict[str, dict[str, float]]:
        return {
            name: {"voltage": 12.0, "temperature": 25.0, "load": 0.0}
            for name in LEKIWI_WHEEL_NAMES
        }

    def disconnect(self) -> None:
        self.wheel_ticks = (0, 0, 0)
        super().disconnect()


class LeRobotSO101Backend:
    """Safe, non-interactive wrapper around LeRobot's SO101Follower."""

    def __init__(
        self,
        port: str,
        robot_id: str,
        calibration_dir: str,
        torque: bool = True,
    ) -> None:
        if not robot_id:
            raise ValueError("robot_id is required for the lerobot backend")
        self.port = port
        self.robot_id = robot_id
        self.calibration_dir = Path(calibration_dir).expanduser()
        self.calibration_file = self.calibration_dir / f"{robot_id}.json"
        # torque=False は「手でアームを動かして /joint_states で読む」ための姿勢。
        # ★ トルクを切るだけでは足りない。write_positions を止めないと 50Hz で
        #   Goal_Position を書き続け、何かの拍子にトルクが戻った瞬間に
        #   最後の指令値へアームが飛ぶ。切るのと書かないのは必ず対で扱う。
        self.torque = torque
        self._validate_calibration_file()
        self._robot = None

    def _validate_calibration_file(self) -> None:
        if not self.calibration_file.is_file():
            raise FileNotFoundError(
                f"LeRobot calibration file not found: {self.calibration_file}. "
                "Run lerobot-calibrate on the Linux host first."
            )
        try:
            data = json.loads(self.calibration_file.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"invalid calibration JSON: {self.calibration_file}") from exc
        if set(data) != set(LEROBOT_JOINTS):
            raise ValueError("calibration JSON must contain exactly the six SO-101 motors")
        required = {"id", "drive_mode", "homing_offset", "range_min", "range_max"}
        for expected_id, name in enumerate(LEROBOT_JOINTS, start=1):
            entry = data[name]
            if not isinstance(entry, dict) or not required.issubset(entry):
                raise ValueError(f"calibration entry for {name} is incomplete")
            if int(entry["id"]) != expected_id:
                raise ValueError(
                    f"calibration motor {name} must use ID {expected_id}, got {entry['id']}"
                )

    def connect(self) -> None:
        # Delayed import keeps mock/unit tests usable on macOS without LeRobot's
        # Linux serial runtime or ROS installation.
        # Both classes are exported by the package in LeRobot 0.5.1. The
        # config class is intentionally not defined in so_follower.py itself.
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

        config = SO101FollowerConfig(
            port=self.port,
            id=self.robot_id,
            calibration_dir=self.calibration_dir,
            use_degrees=True,
            cameras={},
            disable_torque_on_disconnect=True,
        )
        robot = SO101Follower(config)
        self._robot = robot
        try:
            robot.bus.connect()
            if not robot.is_calibrated:
                raise RuntimeError(
                    "calibration JSON does not match the servo EEPROM; "
                    "rerun lerobot-calibrate on the Linux host"
                )

            # configure() temporarily disables then re-enables torque. Latch
            # the current normalized position first so stale Goal_Position
            # cannot make the arm jump when torque is re-enabled.
            robot.bus.disable_torque()
            current = robot.bus.sync_read("Present_Position")
            robot.bus.sync_write("Goal_Position", current)
            robot.configure()
            if not self.torque:
                # configure() がトルクを入れ直すので、その後で切る。
                # 読み出し (Present_Position) はトルクの有無に関係なく動くので
                # /joint_states は出続ける。
                robot.bus.disable_torque()
        except Exception:
            self.disconnect()
            raise

    def _connected_robot(self):
        if self._robot is None or not self._robot.is_connected:
            raise RuntimeError("LeRobot backend is not connected")
        return self._robot

    def read_positions(self) -> dict[str, float]:
        observation = self._connected_robot().get_observation()
        return {name: float(observation[f"{name}.pos"]) for name in LEROBOT_JOINTS}

    def write_positions(self, positions: dict[str, float]) -> None:
        if set(positions) != set(LEROBOT_JOINTS):
            raise ValueError("LeRobot backend requires all SO-101 joints")
        robot = self._connected_robot()
        if not self.torque:
            # ★ 検証は上で済ませたうえで捨てる。壊れた指令は torque の有無に
            #   関わらず弾きたい (torque=True へ戻したときに挙動が変わらない)。
            return
        action = {f"{name}.pos": float(positions[name]) for name in LEROBOT_JOINTS}
        robot.send_action(action)

    def disconnect(self) -> None:
        robot, self._robot = self._robot, None
        if robot is None or not robot.bus.is_connected:
            return
        try:
            robot.disconnect()
        except Exception as exc:  # noqa: BLE001 - best-effort safety cleanup
            LOGGER.warning(
                "LeRobot disconnect failed; closing the serial port directly: %s", exc
            )
            # A communication failure may prevent the torque-off write, but
            # the serial port still must be closed where possible.
            try:
                robot.bus.port_handler.closePort()
            except Exception as close_exc:  # noqa: BLE001 - no recovery remains
                LOGGER.error("Failed to close the LeRobot serial port: %s", close_exc)


class LeRobotLeKiwiBackend:
    """Safe, non-interactive wrapper around LeRobot's nine-motor LeKiwi."""

    def __init__(
        self,
        port: str,
        robot_id: str,
        calibration_dir: str,
        *,
        torque: bool = True,
        wheel_acceleration: int = 20,
    ) -> None:
        if not robot_id:
            raise ValueError("robot_id is required for the shared LeKiwi backend")
        if not 0 <= int(wheel_acceleration) <= 254:
            raise ValueError("wheel_acceleration must be between 0 and 254")
        self.port = port
        self.robot_id = robot_id
        self.calibration_dir = Path(calibration_dir).expanduser()
        self.calibration_file = self.calibration_dir / f"{robot_id}.json"
        # In shared mode this controls only the arm. Wheels remain available
        # so arm_torque:=false can be used while testing or driving the base.
        self.torque = torque
        self.wheel_acceleration = int(wheel_acceleration)
        self._validate_calibration_file()
        self._robot = None

    def _validate_calibration_file(self) -> None:
        if not self.calibration_file.is_file():
            raise FileNotFoundError(
                f"LeRobot LeKiwi calibration file not found: {self.calibration_file}. "
                "Run lerobot-calibrate --robot.type=lekiwi on the Linux host first."
            )
        try:
            data = json.loads(self.calibration_file.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"invalid calibration JSON: {self.calibration_file}") from exc
        if not isinstance(data, dict) or set(data) != set(LEKIWI_NAMES):
            raise ValueError("LeKiwi calibration JSON must contain exactly the nine canonical motors")
        required = {"id", "drive_mode", "homing_offset", "range_min", "range_max"}
        for expected_id, name in enumerate(LEKIWI_NAMES, start=1):
            entry = data[name]
            if not isinstance(entry, dict) or not required.issubset(entry):
                raise ValueError(f"calibration entry for {name} is incomplete")
            actual_id = entry["id"]
            if isinstance(actual_id, bool) or not isinstance(actual_id, int):
                raise ValueError(f"calibration motor {name} has an invalid id")
            if actual_id != expected_id:
                raise ValueError(
                    f"calibration motor {name} must use ID {expected_id}, got {entry['id']}"
                )

    def connect(self) -> None:
        from lerobot.motors.feetech import OperatingMode
        from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig

        config = LeKiwiConfig(
            port=self.port,
            id=self.robot_id,
            calibration_dir=self.calibration_dir,
            use_degrees=True,
            cameras={},
            disable_torque_on_disconnect=True,
        )
        robot = LeKiwi(config)
        self._robot = robot
        bus = robot.bus
        try:
            # connect() handshakes all nine canonical IDs before returning.
            bus.connect()
            # Stop every actuator immediately after the all-ID handshake. Any
            # EEPROM/calibration mismatch below must fail with torque off.
            bus.disable_torque()
            if not robot.is_calibrated:
                raise RuntimeError(
                    "LeKiwi calibration JSON does not match the servo EEPROM; "
                    "rerun lerobot-calibrate --robot.type=lekiwi on the Linux host"
                )

            # Never enable torque against stale goals. The arm is latched at its
            # current normalized position and wheels are explicitly zeroed first.
            current = bus.sync_read("Present_Position", list(LEKIWI_ARM_NAMES))
            bus.sync_write("Goal_Position", current)
            self.write_wheel_ticks((0, 0, 0))

            bus.configure_motors()
            for name in LEKIWI_ARM_NAMES:
                bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
                bus.write("P_Coefficient", name, 16)
                bus.write("I_Coefficient", name, 0)
                bus.write("D_Coefficient", name, 32)
            bus.write("Max_Torque_Limit", "arm_gripper", 500)
            bus.write("Protection_Current", "arm_gripper", 250)
            bus.write("Overload_Torque", "arm_gripper", 25)

            for name in LEKIWI_WHEEL_NAMES:
                bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
                bus.write("Acceleration", name, self.wheel_acceleration)
            if self.torque:
                bus.enable_torque()
            else:
                bus.enable_torque(list(LEKIWI_WHEEL_NAMES))
        except Exception:
            self.disconnect()
            raise

    def _connected_robot(self):
        if self._robot is None or not self._robot.is_connected:
            raise RuntimeError("LeKiwi backend is not connected")
        return self._robot

    def read_positions(self) -> dict[str, float]:
        robot = self._connected_robot()
        values = robot.bus.sync_read("Present_Position", list(LEKIWI_ARM_NAMES))
        return {
            canonical: float(values[prefixed])
            for canonical, prefixed in zip(LEROBOT_JOINTS, LEKIWI_ARM_NAMES, strict=True)
        }

    def write_positions(self, positions: dict[str, float]) -> None:
        if set(positions) != set(LEROBOT_JOINTS):
            raise ValueError("shared LeKiwi backend requires all SO-101 joints")
        if not self.torque:
            return
        values = {
            prefixed: float(positions[canonical])
            for canonical, prefixed in zip(LEROBOT_JOINTS, LEKIWI_ARM_NAMES, strict=True)
        }
        self._connected_robot().bus.sync_write("Goal_Position", values)

    def write_wheel_ticks(self, ticks: tuple[int, int, int]) -> None:
        if len(ticks) != 3:
            raise ValueError("shared LeKiwi backend requires exactly three wheel ticks")
        values = {
            name: int(value)
            for name, value in zip(LEKIWI_WHEEL_NAMES, ticks, strict=True)
        }
        self._connected_robot().bus.sync_write(
            "Goal_Velocity", values, normalize=False
        )

    def recover_wheels(self) -> None:
        from lerobot.motors.feetech import OperatingMode

        bus = self._connected_robot().bus
        self.write_wheel_ticks((0, 0, 0))
        bus.disable_torque(list(LEKIWI_WHEEL_NAMES))
        for name in LEKIWI_WHEEL_NAMES:
            bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
            bus.write("Acceleration", name, self.wheel_acceleration)
        bus.enable_torque(list(LEKIWI_WHEEL_NAMES))

    def read_wheel_diagnostics(self) -> dict[str, dict[str, float]]:
        bus = self._connected_robot().bus
        result = {}
        for name in LEKIWI_WHEEL_NAMES:
            result[name] = {
                "voltage": float(bus.read("Present_Voltage", name, normalize=False)) / 10.0,
                "temperature": float(bus.read("Present_Temperature", name, normalize=False)),
                "load": float(bus.read("Present_Load", name, normalize=False)),
            }
        return result

    def disconnect(self) -> None:
        robot, self._robot = self._robot, None
        if robot is None or not robot.bus.is_connected:
            return
        try:
            robot.bus.sync_write(
                "Goal_Velocity",
                dict.fromkeys(LEKIWI_WHEEL_NAMES, 0),
                normalize=False,
                num_retry=5,
            )
            robot.bus.disconnect(disable_torque=True)
        except Exception as exc:  # noqa: BLE001 - best-effort safety cleanup
            LOGGER.warning("LeKiwi disconnect failed; closing serial port: %s", exc)
            try:
                robot.bus.port_handler.closePort()
            except Exception as close_exc:  # noqa: BLE001
                LOGGER.error("Failed to close the LeKiwi serial port: %s", close_exc)

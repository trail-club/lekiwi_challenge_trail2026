"""LeRobot and in-memory backends used by the ROS bridge."""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

from .bridge_core import LEROBOT_JOINTS

LOGGER = logging.getLogger(__name__)


class SO101Backend(Protocol):
    def connect(self) -> None: ...
    def read_positions(self) -> dict[str, float]: ...
    def write_positions(self, positions: dict[str, float]) -> None: ...
    def disconnect(self) -> None: ...


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

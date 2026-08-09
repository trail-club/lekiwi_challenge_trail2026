"""Convert LeRobot SO-101 calibration ranges to URDF joint limits."""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


# LeRobot's SO-101 implementation uses degree normalization for the five body
# motors.  wrist_roll is deliberately omitted: LeRobot treats it as a full-turn
# motor and always calibrates it as 0..4095 rather than recording a mechanical
# range of motion.
CALIBRATION_TO_JOINT = {
    "shoulder_pan": "shoulder_pan_joint",
    "shoulder_lift": "shoulder_lift_joint",
    "elbow_flex": "elbow_flex_joint",
    "wrist_flex": "wrist_flex_joint",
}

ENCODER_RESOLUTION = 4095.0

# Mechanical safety range selected for this installation.  Unlike the four
# calibrated body joints below, wrist_roll is treated as a full-turn motor by
# LeRobot and therefore does not provide a useful mechanical range in JSON.
WRIST_ROLL_LIMITS = (-3.0, 3.0)

# These are the limits in the upstream SO-101 description.  They are used when
# no calibration is selected (notably for the mock backend).
DEFAULT_XACRO_LIMITS = {
    "shoulder_pan_lower": -1.91986,
    "shoulder_pan_upper": 1.91986,
    "shoulder_lift_lower": -1.74533,
    "shoulder_lift_upper": 1.74533,
    "elbow_flex_lower": -1.69,
    "elbow_flex_upper": 1.54,
    "wrist_flex_lower": -1.6,
    "wrist_flex_upper": 1.6,
}


def _finite_int(entry: dict[str, Any], key: str, motor: str) -> int:
    try:
        value = int(entry[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"calibration {motor}.{key} must be an integer") from exc
    if not math.isfinite(float(value)):
        raise ValueError(f"calibration {motor}.{key} must be finite")
    return value


def limits_from_calibration_data(
    data: dict[str, Any], motor_bus_mode: str = "split"
) -> dict[str, float]:
    """Return xacro lower/upper arguments from a LeRobot calibration mapping.

    For body joints LeRobot's ``use_degrees=True`` path uses the midpoint of
    ``range_min``/``range_max`` as zero and maps encoder counts with 4095
    counts per revolution.  ``homing_offset`` is intentionally not applied a
    second time: it is already applied in the servo's Present_Position frame.
    """

    if motor_bus_mode not in ("split", "shared"):
        raise ValueError("motor_bus_mode must be either 'split' or 'shared'")
    calibration_prefix = "arm_" if motor_bus_mode == "shared" else ""
    result: dict[str, float] = {}
    for motor, joint in CALIBRATION_TO_JOINT.items():
        calibration_motor = f"{calibration_prefix}{motor}"
        entry = data.get(calibration_motor)
        if not isinstance(entry, dict):
            raise ValueError(f"calibration has no valid entry for {calibration_motor}")
        lower_raw = _finite_int(entry, "range_min", calibration_motor)
        upper_raw = _finite_int(entry, "range_max", calibration_motor)
        if lower_raw >= upper_raw:
            raise ValueError(
                f"calibration {calibration_motor}: range_min must be < range_max"
            )

        midpoint = (lower_raw + upper_raw) / 2.0
        lower = (lower_raw - midpoint) * 2.0 * math.pi / ENCODER_RESOLUTION
        upper = (upper_raw - midpoint) * 2.0 * math.pi / ENCODER_RESOLUTION
        result[f"{motor}_lower"] = lower
        result[f"{motor}_upper"] = upper

    return result


def load_limits(
    calibration_dir: str | Path,
    robot_id: str,
    *,
    required: bool,
    motor_bus_mode: str = "split",
) -> dict[str, float] | None:
    """Load limits for ``robot_id`` or return ``None`` when calibration is unused."""

    robot_id = str(robot_id).strip()
    if not robot_id:
        if required:
            raise ValueError("robot_id is required when backend:=lerobot")
        return None

    path = Path(calibration_dir).expanduser() / f"{robot_id}.json"
    if not path.is_file():
        if required:
            raise FileNotFoundError(f"LeRobot calibration file not found: {path}")
        return None
    try:
        data = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid calibration JSON: {path}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"calibration JSON must contain an object: {path}")
    return limits_from_calibration_data(data, motor_bus_mode)


def build_robot_description(
    xacro_file: str | Path,
    mappings: dict[str, str],
    calibration_dir: str | Path,
    robot_id: str,
    backend: str,
    motor_bus_mode: str = "split",
) -> str:
    """Expand xacro and replace four body-joint limits for a real calibration.

    The upstream description keeps these limits as XML literals.  Patching the
    expanded XML here avoids maintaining a fork of the upstream description and
    ensures the same runtime description is given to RSP, ros2_control, and the
    LeRobot bridge.
    """

    # Import xacro lazily so this module remains usable by unit tests without a
    # ROS installation.
    import xacro

    document = xacro.process_file(str(xacro_file), mappings=mappings)
    xml = document.toxml()
    limits = load_limits(
        calibration_dir,
        robot_id,
        required=backend == "lerobot",
        motor_bus_mode=motor_bus_mode,
    )
    root = ET.fromstring(xml)
    prefix = str(mappings.get("prefix", ""))
    if limits is not None:
        for motor, joint in CALIBRATION_TO_JOINT.items():
            element = next(
                (
                    item
                    for item in root.findall("joint")
                    if item.get("name") == f"{prefix}{joint}"
                ),
                None,
            )
            if element is None:
                raise ValueError(f"robot_description has no {prefix}{joint}")
            limit = element.find("limit")
            if limit is None:
                raise ValueError(
                    f"robot_description joint has no limit: {prefix}{joint}"
                )
            limit.set("lower", f"{limits[f'{motor}_lower']:.9f}")
            limit.set("upper", f"{limits[f'{motor}_upper']:.9f}")

    wrist_roll = next(
        (
            item
            for item in root.findall("joint")
            if item.get("name") == f"{prefix}wrist_roll_joint"
        ),
        None,
    )
    if wrist_roll is None:
        raise ValueError(f"robot_description has no {prefix}wrist_roll_joint")
    wrist_roll_limit = wrist_roll.find("limit")
    if wrist_roll_limit is None:
        raise ValueError(
            f"robot_description joint has no limit: {prefix}wrist_roll_joint"
        )
    wrist_roll_limit.set("lower", f"{WRIST_ROLL_LIMITS[0]:.9f}")
    wrist_roll_limit.set("upper", f"{WRIST_ROLL_LIMITS[1]:.9f}")

    return ET.tostring(root, encoding="unicode")

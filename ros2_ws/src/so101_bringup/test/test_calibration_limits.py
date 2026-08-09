import math

import pytest

from so101_bringup.calibration_limits import (
    DEFAULT_XACRO_LIMITS,
    WRIST_ROLL_LIMITS,
    limits_from_calibration_data,
)


def test_calibration_limits_match_lerobot_degree_conversion():
    data = {
        "shoulder_pan": {"range_min": 695, "range_max": 3409},
        "shoulder_lift": {"range_min": 802, "range_max": 3154},
        "elbow_flex": {"range_min": 670, "range_max": 3051},
        "wrist_flex": {"range_min": 1014, "range_max": 3350},
    }

    limits = limits_from_calibration_data(data)

    assert limits["shoulder_lift_lower"] == pytest.approx(-1.804401934)
    assert limits["shoulder_lift_upper"] == pytest.approx(1.804401934)
    assert limits["wrist_flex_lower"] == pytest.approx(-1.792127091)
    assert limits["wrist_flex_upper"] == pytest.approx(1.792127091)
    assert all(math.isfinite(value) for value in limits.values())


def test_wrist_roll_is_not_derived_from_full_turn_calibration():
    data = {
        "shoulder_pan": {"range_min": 0, "range_max": 4095},
        "shoulder_lift": {"range_min": 0, "range_max": 4095},
        "elbow_flex": {"range_min": 0, "range_max": 4095},
        "wrist_flex": {"range_min": 0, "range_max": 4095},
        "wrist_roll": {"range_min": 0, "range_max": 4095},
    }

    limits = limits_from_calibration_data(data)

    assert "wrist_roll_lower" not in limits
    assert "wrist_roll_upper" not in limits
    assert DEFAULT_XACRO_LIMITS["wrist_flex_upper"] == 1.6
    assert WRIST_ROLL_LIMITS == (-3.0, 3.0)


def test_shared_calibration_uses_arm_prefixed_entries():
    shared = {
        f"arm_{name}": {"range_min": 100, "range_max": 3000}
        for name in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
    }
    shared.update(
        {
            "arm_wrist_roll": {"range_min": 0, "range_max": 4095},
            "arm_gripper": {"range_min": 0, "range_max": 4095},
            "base_left_wheel": {"range_min": 0, "range_max": 4095},
            "base_back_wheel": {"range_min": 0, "range_max": 4095},
            "base_right_wheel": {"range_min": 0, "range_max": 4095},
        }
    )

    limits = limits_from_calibration_data(shared, "shared")
    split = limits_from_calibration_data(
        {name.removeprefix("arm_"): value for name, value in shared.items()},
        "split",
    )
    assert limits == split


def test_shared_calibration_rejects_six_axis_keys():
    data = {
        name: {"range_min": 100, "range_max": 3000}
        for name in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
    }
    with pytest.raises(ValueError, match="arm_shoulder_pan"):
        limits_from_calibration_data(data, "shared")

"""goal_drive の判断を ROS 抜きで固定する。"""

import math

from lekiwi_examples.goal_drive_logic import Limits, decide, sector_min


def test_vx_never_negative():
    cases = [
        decide(2.0, 2.0, 2.0, 1.0, 0.0, 0.0),
        decide(0.2, 0.2, 0.2, 1.0, 0.0, 0.0),
        decide(2.0, 2.0, 2.0, -1.0, 0.0, 0.0),
        decide(2.0, 2.0, 2.0, 0.0, 0.0, 1.0),
        decide(0.5, 0.3, 2.0, 0.8, 0.4, 0.0),
    ]
    assert all(cmd.vx >= 0.0 for cmd in cases)


def test_behind_goal_turns_in_place():
    cmd = decide(3.0, 3.0, 3.0, -1.0, 0.2, 0.0)
    assert cmd.mode == "look_toward_goal"
    assert cmd.vx == 0.0
    assert cmd.wz > 0.0


def test_arrived_stops():
    cmd = decide(3.0, 3.0, 3.0, 0.05, 0.0, 0.0)
    assert cmd.mode == "arrived"
    assert cmd.vx == 0.0
    assert cmd.wz == 0.0


def test_arrived_then_align_yaw():
    cmd = decide(3.0, 3.0, 3.0, 0.05, 0.0, 1.0)
    assert cmd.mode == "align_yaw"
    assert cmd.vx == 0.0
    assert cmd.wz > 0.0


def test_blocked_does_not_drive():
    cmd = decide(0.25, 0.25, 0.25, 1.0, 0.0, 0.0)
    assert cmd.mode == "blocked_turn"
    assert cmd.vx == 0.0


def test_clear_front_drives_forward():
    cmd = decide(2.0, 2.0, 2.0, 1.2, 0.0, 0.0)
    assert cmd.mode == "drive"
    assert cmd.vx == Limits().cruise


def test_sector_min_ignores_self_hits():
    # 1° 刻み、0° が index 180 相当ではなく angle_min=-pi の 360 本
    ranges = [0.10] * 360
    ranges[180] = 1.5  # 0°
    got = sector_min(ranges, -math.pi, math.radians(1.0), -5.0, 5.0, 0.22, 0.05, 12.0)
    assert got == 1.5

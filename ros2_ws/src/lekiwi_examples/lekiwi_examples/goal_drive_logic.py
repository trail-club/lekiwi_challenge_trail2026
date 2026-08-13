"""Nav2 なしのゴール追従の判断。ROS に依存しない。

後退しない。ゴールが後ろなら先に旋回して正面へ入れてから進む。
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Limits:
    self_r: float = 0.22
    stop_r: float = 0.40
    slow_r: float = 0.75
    cruise: float = 0.07
    creep: float = 0.035
    turn: float = 0.40
    arrive_xy: float = 0.20
    arrive_yaw: float = 0.35
    bearing_gain: float = 1.2
    side_block: float = 0.28


@dataclass(frozen=True)
class Cmd:
    vx: float
    wz: float
    mode: str


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def wrap_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def sector_min(
    ranges: list[float],
    angle_min: float,
    increment: float,
    deg0: float,
    deg1: float,
    self_r: float,
    range_min: float,
    range_max: float,
) -> float:
    """指定角度範囲の最短有効距離。点が無ければ +inf。"""
    best = math.inf
    angle = angle_min
    for rng in ranges:
        deg = wrap_deg(math.degrees(angle))
        angle += increment
        if deg < deg0 or deg > deg1:
            continue
        if range_min < rng < range_max and rng >= self_r and rng < best:
            best = rng
    return best


def decide(
    front: float,
    front_left: float,
    front_right: float,
    goal_x: float,
    goal_y: float,
    yaw_err: float,
    limits: Limits | None = None,
) -> Cmd:
    """base_footprint 上のゴールと前方スキャンから (vx, wz) を決める。

    vx は 0 以上。ゴールが後方半面なら旋回のみ。
    """
    lim = limits or Limits()
    dist = math.hypot(goal_x, goal_y)
    bearing = math.atan2(goal_y, goal_x)
    prefer = 1.0 if bearing > 0.0 else -1.0

    if dist <= lim.arrive_xy:
        if abs(yaw_err) > lim.arrive_yaw:
            return Cmd(0.0, lim.turn * (1.0 if yaw_err > 0.0 else -1.0), "align_yaw")
        return Cmd(0.0, 0.0, "arrived")

    if goal_x < 0.0:
        return Cmd(0.0, lim.turn * prefer, "look_toward_goal")

    if front < lim.stop_r or min(front_left, front_right) < lim.side_block:
        side = prefer
        if prefer > 0.0 and front_left + 0.2 < front_right:
            side = -1.0
        elif prefer < 0.0 and front_right + 0.2 < front_left:
            side = 1.0
        return Cmd(0.0, lim.turn * side, "blocked_turn")

    if front < lim.slow_r:
        return Cmd(lim.creep, 0.6 * lim.turn * prefer, "creep")

    wz = max(-lim.turn, min(lim.turn, lim.bearing_gain * bearing))
    return Cmd(lim.cruise, wz, "drive")

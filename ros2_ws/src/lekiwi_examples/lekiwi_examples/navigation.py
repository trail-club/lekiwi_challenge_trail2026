"""Thin competition-facing wrapper around Nav2's existing NavigateToPose API.

This module intentionally owns no planner, controller, localisation, or
``/cmd_vel`` publisher.  Those responsibilities already belong to Nav2 and
``lekiwi_base_bringup``.  It gives the task orchestrator one blocking call with
a bounded retry policy and a result it can turn into its own state transition.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .navigation_types import PlanarPose


class NavigationStatus(str, Enum):
    """Outcome exposed to the competition-level task orchestrator."""

    SUCCEEDED = "succeeded"
    REJECTED = "rejected"
    FAILED = "failed"
    TIMED_OUT = "timed_out"


@dataclass(frozen=True)
class NavigationResult:
    status: NavigationStatus
    attempts: int
    detail: str

    @property
    def succeeded(self) -> bool:
        return self.status is NavigationStatus.SUCCEEDED


def pose_stamped(target: PlanarPose, *, frame_id: str = "map") -> PoseStamped:
    """Convert a planar target to the Nav2 message type without tf helpers."""

    if not frame_id:
        raise ValueError("frame_id must not be empty")

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = target.x
    pose.pose.position.y = target.y
    pose.pose.orientation.z = math.sin(target.yaw / 2.0)
    pose.pose.orientation.w = math.cos(target.yaw / 2.0)
    return pose


class CompetitionNavigator(BasicNavigator):
    """Use Nav2's Simple Commander with a bounded retry/timeout policy."""

    def __init__(self, node_name: str = "competition_navigator") -> None:
        super().__init__(node_name=node_name)

    def navigate_to(
        self,
        target: PlanarPose,
        *,
        frame_id: str = "map",
        timeout_sec: float = 90.0,
        max_retries: int = 2,
    ) -> NavigationResult:
        """Navigate to ``target`` and return only after success or final failure.

        ``max_retries`` is the number of *additional* attempts, so the default
        permits three total Nav2 goals.  Cancelling on timeout returns control to
        Nav2 before the next attempt; no direct velocity command is ever sent.
        """

        if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be a positive finite value")
        if max_retries < 0:
            raise ValueError("max_retries must be greater than or equal to zero")

        goal = pose_stamped(target, frame_id=frame_id)
        last_status = NavigationStatus.FAILED
        last_detail = "NavigateToPose did not report a result"

        for attempt in range(1, max_retries + 2):
            self.get_logger().info(
                f"Nav2 attempt {attempt}/{max_retries + 1}: "
                f"({target.x:.3f}, {target.y:.3f}, {target.yaw:.3f}) in {frame_id}"
            )
            if not self.goToPose(goal):
                last_status = NavigationStatus.REJECTED
                last_detail = "NavigateToPose goal was rejected"
                continue

            deadline = time.monotonic() + timeout_sec
            timed_out = False
            while not self.isTaskComplete():
                if time.monotonic() >= deadline:
                    self.get_logger().warn(
                        f"Nav2 attempt {attempt} exceeded {timeout_sec:.1f}s; cancelling"
                    )
                    self.cancelTask()
                    timed_out = True
                    break

            if not timed_out and self.getResult() == TaskResult.SUCCEEDED:
                return NavigationResult(
                    NavigationStatus.SUCCEEDED,
                    attempt,
                    "Nav2 NavigateToPose succeeded",
                )

            if timed_out:
                last_status = NavigationStatus.TIMED_OUT
                last_detail = f"NavigateToPose exceeded {timeout_sec:.1f}s"
            else:
                last_status = NavigationStatus.FAILED
                last_detail = f"Nav2 result: {self.getResult().name}"

        return NavigationResult(last_status, max_retries + 1, last_detail)

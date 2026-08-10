"""Navigation application layer that is independent from ROS 2.

The competition tells us where an object or drop zone is, but the mobile base
must not drive onto that point.  This module deliberately keeps a landmark and
the pose at which the arm can work as separate concepts.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PlanarPose:
    """A pose in the map frame, expressed as x, y and yaw in radians."""

    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (self.x, self.y, self.yaw)):
            raise ValueError("x, y, and yaw must all be finite")


def approach_pose_from_landmark(
    landmark: PlanarPose,
    standoff_m: float,
    *,
    final_yaw: float | None = None,
) -> PlanarPose:
    """Return the base pose at ``standoff_m`` in front of a landmark.

    ``final_yaw`` is the heading required by the arm at arrival.  The robot is
    placed behind that heading so that its +X direction points at the landmark.
    Passing it explicitly keeps object/drop geometry independent from the
    preferred grasp/place orientation.  If omitted, the landmark yaw is used.
    """

    if not math.isfinite(standoff_m) or standoff_m < 0.0:
        raise ValueError("standoff_m must be a finite value greater than or equal to zero")

    yaw = landmark.yaw if final_yaw is None else final_yaw
    if not math.isfinite(yaw):
        raise ValueError("final_yaw must be finite")

    return PlanarPose(
        x=landmark.x - standoff_m * math.cos(yaw),
        y=landmark.y - standoff_m * math.sin(yaw),
        yaw=yaw,
    )

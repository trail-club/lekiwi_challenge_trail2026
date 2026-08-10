"""Navigate to an approach pose derived from an object or drop-zone landmark.

Example::

    ros2 run lekiwi_examples navigation_approach_demo --ros-args \
      -p landmark_x:=1.0 -p landmark_y:=0.0 -p approach_yaw:=0.0 \
      -p standoff_m:=0.45
"""

from __future__ import annotations

import rclpy

from .navigation import CompetitionNavigator
from .navigation_types import PlanarPose, approach_pose_from_landmark


def main() -> None:
    rclpy.init()
    navigator = CompetitionNavigator("navigation_approach_demo")
    navigator.declare_parameter("landmark_x", 0.0)
    navigator.declare_parameter("landmark_y", 0.0)
    navigator.declare_parameter("landmark_yaw", 0.0)
    navigator.declare_parameter("approach_yaw", 0.0)
    navigator.declare_parameter("standoff_m", 0.45)
    navigator.declare_parameter("timeout_sec", 90.0)
    navigator.declare_parameter("max_retries", 2)

    try:
        landmark = PlanarPose(
            float(navigator.get_parameter("landmark_x").value),
            float(navigator.get_parameter("landmark_y").value),
            float(navigator.get_parameter("landmark_yaw").value),
        )
        approach_yaw = float(navigator.get_parameter("approach_yaw").value)
        approach = approach_pose_from_landmark(
            landmark,
            float(navigator.get_parameter("standoff_m").value),
            final_yaw=approach_yaw,
        )

        navigator.get_logger().info(
            "Approach pose from landmark "
            f"({landmark.x:.3f}, {landmark.y:.3f}, {landmark.yaw:.3f}) "
            f"with standoff {float(navigator.get_parameter('standoff_m').value):.3f} m: "
            f"({approach.x:.3f}, {approach.y:.3f}, {approach.yaw:.3f})"
        )
        result = navigator.navigate_to(
            approach,
            timeout_sec=float(navigator.get_parameter("timeout_sec").value),
            max_retries=int(navigator.get_parameter("max_retries").value),
        )
        log = navigator.get_logger()
        if result.succeeded:
            log.info(f"ARRIVED_APPROACH after {result.attempts} attempt(s): {result.detail}")
        else:
            log.error(
                f"NAV_FAILED ({result.status.value}) after {result.attempts} attempt(s): "
                f"{result.detail}"
            )
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

"""指定したmap座標の姿勢をNav2へ送り、成功/失敗をログに出す最小デモ。

実行例（先に``make mock-shared``などでNav2を起動しておく）::

    ros2 run lekiwi_examples navigation_demo --ros-args \
      -p target_x:=1.0 -p target_y:=0.0 -p target_yaw:=0.0
"""

from __future__ import annotations

import rclpy

from .navigation import CompetitionNavigator
from .navigation_types import PlanarPose


def main() -> None:
    rclpy.init()
    navigator = CompetitionNavigator("navigation_demo")
    navigator.declare_parameter("target_x", 0.0)
    navigator.declare_parameter("target_y", 0.0)
    navigator.declare_parameter("target_yaw", 0.0)
    navigator.declare_parameter("timeout_sec", 90.0)
    navigator.declare_parameter("max_retries", 2)

    try:
        target = PlanarPose(
            float(navigator.get_parameter("target_x").value),
            float(navigator.get_parameter("target_y").value),
            float(navigator.get_parameter("target_yaw").value),
        )
        result = navigator.navigate_to(
            target,
            timeout_sec=float(navigator.get_parameter("timeout_sec").value),
            max_retries=int(navigator.get_parameter("max_retries").value),
        )
        log = navigator.get_logger()
        if result.succeeded:
            log.info(f"NAV_SUCCEEDED after {result.attempts} attempt(s): {result.detail}")
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

"""ROS 2に依存しないNavigation用の小さい型定義。

競技では物体やDrop Zoneの位置が与えられるが、ロボット中心をそこへ突っ込ませては
いけない。ここでは「ランドマークの位置」と「アームが作業しやすい停止位置」を
別の概念として扱う。
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PlanarPose:
    """map座標系での姿勢。x, y, yaw[rad]で表す。"""

    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (self.x, self.y, self.yaw)):
            raise ValueError("x, y, yawはすべて有限値にしてください")


def approach_pose_from_landmark(
    landmark: PlanarPose,
    standoff_m: float,
    *,
    final_yaw: float | None = None,
) -> PlanarPose:
    """ランドマークの手前``standoff_m``にあるロボット停止姿勢を返す。

    ``final_yaw``は到着時にアームが作業しやすいロボットの向きである。
    ロボットの+X方向がランドマークを向くように、その向きの後ろ側へ停止位置を置く。
    省略した場合はランドマーク自身のyawを使う。
    """

    if not math.isfinite(standoff_m) or standoff_m < 0.0:
        raise ValueError("standoff_mは0以上の有限値にしてください")

    yaw = landmark.yaw if final_yaw is None else final_yaw
    if not math.isfinite(yaw):
        raise ValueError("final_yawは有限値にしてください")

    return PlanarPose(
        x=landmark.x - standoff_m * math.cos(yaw),
        y=landmark.y - standoff_m * math.sin(yaw),
        yaw=yaw,
    )

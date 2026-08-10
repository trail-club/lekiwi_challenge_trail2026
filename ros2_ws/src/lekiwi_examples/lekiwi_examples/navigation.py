"""競技タスク側からNav2のNavigateToPoseを呼ぶための薄いラッパ。

このモジュールでは、経路計画・制御・自己位置推定・``/cmd_vel`` publishを持たない。
それらは既存のNav2と``lekiwi_base_bringup``に任せる。ここではTask Orchestratorが
「指定姿勢へ行く」「成功/失敗を受け取る」ための小さい入口だけを提供する。
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
    """競技レベルの状態遷移へ渡すNavigation結果。"""

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
    """x, y, yawの目標姿勢をNav2へ渡すPoseStampedへ変換する。"""

    if not frame_id:
        raise ValueError("frame_idは空にできません")

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = target.x
    pose.pose.position.y = target.y
    pose.pose.orientation.z = math.sin(target.yaw / 2.0)
    pose.pose.orientation.w = math.cos(target.yaw / 2.0)
    return pose


class CompetitionNavigator(BasicNavigator):
    """Nav2 Simple Commanderにリトライとタイムアウトを足した競技用入口。"""

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
        """``target``へ移動し、成功または最終失敗が決まってから結果を返す。

        ``max_retries``は追加試行回数で、既定値では合計3回までNav2 goalを送る。
        タイムアウト時は次の試行前にNav2へcancelを送り、ここから直接速度指令は出さない。
        """

        if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
            raise ValueError("timeout_secは正の有限値にしてください")
        if max_retries < 0:
            raise ValueError("max_retriesは0以上にしてください")

        goal = pose_stamped(target, frame_id=frame_id)
        last_status = NavigationStatus.FAILED
        last_detail = "NavigateToPoseの結果が返りませんでした"

        for attempt in range(1, max_retries + 2):
            self.get_logger().info(
                f"Nav2 attempt {attempt}/{max_retries + 1}: "
                f"({target.x:.3f}, {target.y:.3f}, {target.yaw:.3f}) in {frame_id}"
            )
            if not self.goToPose(goal):
                last_status = NavigationStatus.REJECTED
                last_detail = "NavigateToPose goalが拒否されました"
                continue

            deadline = time.monotonic() + timeout_sec
            timed_out = False
            while not self.isTaskComplete():
                if time.monotonic() >= deadline:
                    self.get_logger().warn(
                        f"Nav2試行{attempt}が{timeout_sec:.1f}秒を超えたためcancelします"
                    )
                    self.cancelTask()
                    timed_out = True
                    break

            if not timed_out and self.getResult() == TaskResult.SUCCEEDED:
                return NavigationResult(
                    NavigationStatus.SUCCEEDED,
                    attempt,
                    "Nav2 NavigateToPoseが成功しました",
                )

            if timed_out:
                last_status = NavigationStatus.TIMED_OUT
                last_detail = f"NavigateToPoseが{timeout_sec:.1f}秒を超えました"
            else:
                last_status = NavigationStatus.FAILED
                last_detail = f"Nav2 result: {self.getResult().name}"

        return NavigationResult(last_status, max_retries + 1, last_detail)

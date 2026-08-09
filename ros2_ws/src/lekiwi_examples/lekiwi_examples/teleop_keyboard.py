"""キーボードでベースとアームを同時に動かす。

    ros2 run lekiwi_examples teleop_keyboard

★ `robot.launch.py` が動いていることが前提。このノードはハードウェアに
  直接触らず、ROS のインターフェースだけを使う:

    ベース  -> /cmd_vel                                        (geometry_msgs/Twist)
    アーム  -> /joint_trajectory_controller/joint_trajectory    (trajectory_msgs)
    グリッパ -> /gripper_controller/gripper_cmd                  (action)

────────────────────────────────────────────────────────────────────────
★ 安全上の注意
────────────────────────────────────────────────────────────────────────
* **車輪を浮かせてから使うこと。** `/cmd_vel` は Nav2 の collision_monitor
  より下流なので、**衝突監視も加速度制限も効かない**。
* アームは可動域の内側 `joint_limit_margin` まで自動でクランプするが、
  **機体との干渉は見ていない**。LiDAR やプレートに当たりうる。
* キーを離せばベースは止まる (`base_driver` の watchdog が 0.5 秒で
  速度ゼロにする)。**アームは止まらず、その姿勢で保持する**。

────────────────────────────────────────────────────────────────────────
★ アームの指令の作り方 (実機で 2 回踏んだ落とし穴)
────────────────────────────────────────────────────────────────────────
アームは **行き先 `_goal`** と **実際に送る目標 `_command`** の 2 段構え。
キーは `_goal` を `arm_step` 進めるだけで、**送信は 20Hz のタイマーだけ**が
行い、`_command` を `arm_speed` [rad/s] で `_goal` へ寄せていく。

こう書かないと壊れる理由が 2 つある。どちらも実機でしか出ない。
**2 つとも実機で修正を確認済み (2026-08-08)。**

1. **実測値を読み直してはならない。**
   保持力が弱い (全関節 `P=16`。docs/examples.md の
   「既知の未解決事項」)。位置制御のトルクは概ね `P × 位置偏差`なので、
   目標に追い付いた関節は偏差ゼロ = トルクゼロになり重力で下がる。
   実測値に足し込むと**下がった値が次の目標に焼き込まれ**、どのキーを
   押しても shoulder_lift が下がり続ける。

2. **キー 1 打ごとに軌道を投げてはならない。**
   JTC は新しい軌道を受けるたび前の軌道を捨て、**実測値から引き直す**
   (`open_loop_control` は既定 false)。しかも始点速度に使われる
   `/joint_states` の velocity は、STS3215 の 1 量子が 0.077 rad/s に化ける
   信用できない値 (CLAUDE.md)。端末のオートリピート (約 30Hz) でこれを
   毎回踏むと、**押している間は震え、離すと溜まった目標へ動く**。

代償として目標と実測はずれていく。ステータス行に実測値も出すので、
ずれが大きければ **Space** で現在姿勢へ同期し直すこと。

────────────────────────────────────────────────────────────────────────
キー配置
────────────────────────────────────────────────────────────────────────
ベース (★ オムニなので真横にも動ける)

    u  i  o        i / ,  前後       j / l  左右 (strafe)
    j  k  l        u / o  左前/右前   m / .  左後/右後
    m  ,  .        k      停止        [ / ]  その場で旋回

★★ `teleop_twist_keyboard` とは j / l の意味が違う。取り違えに注意。

    | キー | teleop_twist_keyboard | このノード |
    | --- | --- | --- |
    | j / l | **旋回** (angular.z ±1)  | **strafe** (linear.y ±1) |
    | J / L | strafe                  | 未割当 |
    | [ / ] | 未割当                   | 旋回 |

  このベースはオムニで真横に動けるので、Shift 無しの押しやすいキーを
  strafe に割り当てている。**`teleop_twist_keyboard` を動かしていると
  「j / l で旋回する」ことになるが、それは仕様**で機体の故障ではない
  (2026-08-08 に実際に取り違えた)。

アーム (上段が +、下段が −)

    1 / q   shoulder_pan       2 / w   shoulder_lift
    3 / e   elbow_flex         4 / r   wrist_flex
    5 / t   wrist_roll         6 / y   gripper (開 / 閉)

    Space   アームを現在姿勢で保持する (目標を実測値へ同期し直す)
    ?       ヘルプ
    Ctrl+C  終了
"""

from __future__ import annotations

import sys
import termios
import threading
import tty
import xml.etree.ElementTree as ET

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import ParallelGripperCommand
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lekiwi_examples.cartesian_math import joint_limits_from_urdf

# ベース: キー -> (vx, vy, wz) の向き。大きさは speed 側で決める。
BASE_KEYS = {
    "i": (1.0, 0.0, 0.0),
    ",": (-1.0, 0.0, 0.0),
    "j": (0.0, 1.0, 0.0),
    "l": (0.0, -1.0, 0.0),
    "u": (1.0, 1.0, 0.0),
    "o": (1.0, -1.0, 0.0),
    "m": (-1.0, 1.0, 0.0),
    ".": (-1.0, -1.0, 0.0),
    "k": (0.0, 0.0, 0.0),
}
# ★ 回転は別キーにする。teleop_twist_keyboard は j/l を回転に使うが、
#   このベースはオムニで真横に動けるため、strafe を主に割り当てた。
BASE_TURN_KEYS = {"[": 1.0, "]": -1.0}

# アーム: キー -> (関節の並び順, 符号)。上段が +、下段が −。
ARM_JOINT_ORDER = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
)
ARM_KEYS = {
    "1": (0, +1.0), "q": (0, -1.0),
    "2": (1, +1.0), "w": (1, -1.0),
    "3": (2, +1.0), "e": (2, -1.0),
    "4": (3, +1.0), "r": (3, -1.0),
    "5": (4, +1.0), "t": (4, -1.0),
}
GRIPPER_KEYS = {"6": +1.0, "y": -1.0}

HELP = __doc__


class TeleopKeyboard(Node):
    def __init__(self) -> None:
        super().__init__("lekiwi_teleop_keyboard")

        defaults = {
            "joint_prefix": "arm_",
            "cmd_vel_topic": "/cmd_vel",
            "trajectory_topic": "/joint_trajectory_controller/joint_trajectory",
            "gripper_action": "/gripper_controller/gripper_cmd",
            # ★ base.yaml の上限 (0.26 / 0.23 / 1.8) より控えめにしておく。
            #   キー操作は微調整が効かないので、既定は遅いほうが安全。
            "base_linear_speed": 0.10,
            "base_angular_speed": 0.5,
            # アームは 1 キー押下あたりこれだけ「行き先」が進む [rad]。
            "arm_step": 0.05,
            # ★ 実際に送る目標は行き先へこの速度で近づく [rad/s]。
            #   キーのオートリピート速度 (端末・OS まかせ) に指令が
            #   引きずられないようにするための律速。
            "arm_speed": 0.5,
            # ★ 行き先が指令をこれ以上先行しない [rad]。押しっぱなしで
            #   行き先が暴走すると、キーを離した後もその差だけ動き続ける。
            "arm_max_lead": 0.15,
            # 送る軌道の到達時間 [s]。★ publish 周期の 2 倍程度にする。
            #   長くすると JTC が「ゆっくり減速して止まる」軌道を張り、
            #   次の軌道が来るまでにほとんど進まない（追従が遅れる）。
            "arm_step_duration": 0.10,
            # グリッパは 0.0-1.0 の正規化位置で送る。
            "gripper_step": 0.10,
            # 可動域の端から残す余白 [rad]。可動域そのものは /robot_description
            # から読む (URDF が唯一の情報源。ここに数値を持つと必ず古くなる)。
            "joint_limit_margin": 0.10,
            "publish_rate": 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def param(name):
            return self.get_parameter(name).value

        prefix = str(param("joint_prefix"))
        self._joints = [f"{prefix}{name}" for name in ARM_JOINT_ORDER]
        self._gripper_joint = f"{prefix}gripper_joint"
        self._arm_step = float(param("arm_step"))
        self._arm_speed = float(param("arm_speed"))
        self._arm_max_lead = float(param("arm_max_lead"))
        self._arm_duration = float(param("arm_step_duration"))
        self._rate = float(param("publish_rate"))
        self._gripper_step = float(param("gripper_step"))
        self._margin = float(param("joint_limit_margin"))
        self._linear = float(param("base_linear_speed"))
        self._angular = float(param("base_angular_speed"))

        self._cmd_pub = self.create_publisher(Twist, str(param("cmd_vel_topic")), 10)
        self._traj_pub = self.create_publisher(
            JointTrajectory, str(param("trajectory_topic")), 10
        )
        self._gripper = ActionClient(
            self, ParallelGripperCommand, str(param("gripper_action"))
        )

        # ★ /joint_states の publisher は 2 つ (車輪 / アーム) あるので、
        #   1 通では全関節が揃わない。辞書に蓄積する。
        self._positions: dict[str, float] = {}
        self._twist = (0.0, 0.0, 0.0)
        self._lock = threading.Lock()
        self._gripper_target: float | None = None
        # ★ アームは 2 段構え。どちらも実測値ではない
        #   (モジュール冒頭の注意を読むこと)。
        #   _goal    利用者の行き先。キー 1 打で arm_step 進む
        #   _command 実際に送っている目標。_goal へ arm_speed [rad/s] で近づく
        self._goal: list[float] | None = None
        self._command: list[float] | None = None
        self._limits: list[tuple[float, float]] | None = None
        # 到着後も数フレームだけ送って落ち着かせ、その後は黙る。
        # ★ 静止中に送り続けない。JTC を毎周期 preempt すると、
        #   リーチノードなど他の利用者と競合する。
        self._settle = 0
        self._settle_frames = max(1, int(self._arm_duration * self._rate))

        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        # /robot_description は TRANSIENT_LOCAL / depth 1。あとから繋いでも
        # 最後の 1 通が届く。
        self.create_subscription(
            String,
            "/robot_description",
            self._description_cb,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

        # ★ ベースもアームもこの 1 本のタイマーからしか送らない。
        #   キー入力そのものから送ると、端末のオートリピート速度が
        #   そのまま指令の周期になってしまう (実機で「押している間は
        #   震えて、離すと動く」症状を出した)。
        self.create_timer(1.0 / self._rate, self._tick)

    # ── 状態 ──────────────────────────────────────────────────────────

    def _joint_state_cb(self, message: JointState) -> None:
        with self._lock:
            self._positions.update(zip(message.name, message.position))

    def _description_cb(self, message: String) -> None:
        if self._limits is not None:
            return
        try:
            table = joint_limits_from_urdf(message.data)
            limits = [table[name] for name in self._joints]
        except (KeyError, ET.ParseError, ValueError) as exc:
            # クランプ無しでも動かせるので、落とさず警告だけにする。
            self.get_logger().warning(
                f"可動域を読めませんでした。クランプしません: {exc}"
            )
            return
        with self._lock:
            self._limits = limits
        self.get_logger().info("URDF から可動域を読みました")

    def _sync_target(self) -> bool:
        """実測姿勢を行き先・指令の両方に取り込む。関節が揃っていなければ False。"""
        with self._lock:
            if not all(name in self._positions for name in self._joints):
                return False
            current = [self._positions[name] for name in self._joints]
            self._goal = list(current)
            self._command = list(current)
        return True

    def _clamp(self, index: int, value: float) -> float:
        if self._limits is None:
            return value
        lower, upper = self._limits[index]
        lower, upper = lower + self._margin, upper - self._margin
        if lower >= upper:  # 余白が可動域より広い。クランプしない。
            return value
        return min(upper, max(lower, value))

    # ── ベース ────────────────────────────────────────────────────────

    def set_base(self, vx: float, vy: float, wz: float) -> None:
        with self._lock:
            self._twist = (vx, vy, wz)

    def _publish_twist(self) -> None:
        with self._lock:
            vx, vy, wz = self._twist
        message = Twist()
        message.linear.x = vx * self._linear
        message.linear.y = vy * self._linear
        message.angular.z = wz * self._angular
        self._cmd_pub.publish(message)

    # ── アーム ────────────────────────────────────────────────────────

    def nudge_joint(self, index: int, direction: float) -> str:
        """行き先を arm_step ぶん進める。送信はタイマーがやる。

        ★ ここでは publish しない。キー 1 打ごとに軌道を投げると、JTC が
          そのたび前の軌道を破棄して**実測値から引き直す**
          (open_loop_control は既定 false)。実測の velocity は量子化ノイズが
          0.077 rad/s に化ける値なので、引き直すたびに始点速度が暴れる。
          実機で「押している間は震えて、離すと動く」になったのはこれ。
        """
        if self._goal is None and not self._sync_target():
            return "関節状態を待っています（/joint_states）"
        with self._lock:
            assert self._goal is not None and self._command is not None
            goal = self._clamp(index, self._goal[index] + direction * self._arm_step)
            # ★ 行き先が指令を先行しすぎないようにする。オートリピートで
            #   行き先が暴走すると、キーを離した後もその差だけ動き続ける。
            lead = goal - self._command[index]
            if abs(lead) > self._arm_max_lead:
                goal = self._command[index] + self._arm_max_lead * (
                    1.0 if lead > 0 else -1.0
                )
            self._goal[index] = goal
        return self._describe(index)

    def hold_arm(self) -> str:
        """いまの実測姿勢で止める（行き先・指令をそこへ同期し直す）。"""
        if not self._sync_target():
            return "関節状態を待っています"
        with self._lock:
            self._settle = self._settle_frames
        return "アームを現在姿勢で保持（目標を実測値へ同期）"

    def _advance_arm(self) -> None:
        """指令を行き先へ arm_speed で近づけ、動いている間だけ送る。"""
        step = self._arm_speed / self._rate
        with self._lock:
            if self._goal is None or self._command is None:
                return
            moving = False
            for i, (goal, command) in enumerate(zip(self._goal, self._command)):
                delta = goal - command
                if abs(delta) < 1e-6:
                    continue
                moving = True
                if abs(delta) <= step:
                    self._command[i] = goal
                else:
                    self._command[i] = command + step * (1.0 if delta > 0 else -1.0)
            if moving:
                self._settle = self._settle_frames
            elif self._settle > 0:
                self._settle -= 1
            else:
                return  # ★ 静止中は黙る。JTC を毎周期 preempt しない
            command = list(self._command)
        self._publish_trajectory(command)

    def _publish_trajectory(self, positions) -> None:
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(self._joints)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        # ★ 最終点の速度は 0 でなければならない。JTC は 0 以外を
        #   「Velocity of last trajectory point ... is not zero」で**拒否**し、
        #   アームはまったく動かない（モックで実測して判明）。
        #   追従の速さは arm_step_duration（到達時間）で調整すること。
        point.velocities = [0.0] * len(positions)
        seconds = int(self._arm_duration)
        point.time_from_start = Duration(
            sec=seconds, nanosec=int((self._arm_duration - seconds) * 1e9)
        )
        message.points = [point]
        self._traj_pub.publish(message)

    def _describe(self, index: int) -> str:
        # ★ 実測値も併記する。P=16 で保持力が弱いため目標と実測はずれる。
        #   ずれが見えていないと「効いていない」と誤解する。
        name = self._joints[index]
        with self._lock:
            goal = self._goal[index] if self._goal else 0.0
            actual = self._positions.get(name)
        actual_text = "実測 --" if actual is None else f"実測 {actual:+.3f}"
        return f"{name} 目標 {goal:+.3f} ({actual_text}) rad"

    def _tick(self) -> None:
        self._publish_twist()
        self._advance_arm()

    def nudge_gripper(self, direction: float) -> str:
        if not self._gripper.server_is_ready():
            self._gripper.wait_for_server(timeout_sec=0.5)
            if not self._gripper.server_is_ready():
                return "グリッパのアクションサーバが居ません"
        if self._gripper_target is None:
            self._gripper_target = 0.5
        self._gripper_target = min(1.0, max(0.0, self._gripper_target + direction * self._gripper_step))
        goal = ParallelGripperCommand.Goal()
        goal.command.position = [float(self._gripper_target)]
        # ★ 結果は待たない。購読コールバックの中で spin すると詰まる。
        self._gripper.send_goal_async(goal)
        return f"gripper -> {self._gripper_target:.2f}"

    def stop_all(self) -> None:
        self.set_base(0.0, 0.0, 0.0)
        self._publish_twist()


def _read_key() -> str:
    """端末を raw にして 1 文字読む。pynput を使わないので SSH でも動く。"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def handle_key(node, key: str) -> str | None:
    """1 文字を解釈してノードを操作し、ステータス行を返す。

    None を返したら「表示するものは無い」(ヘルプを出したときなど)。

    ★ 端末から切り離してあるのは**テストするため**。ここを main() に埋めて
      いたせいでキー振り分けが一度も試験されていなかった。
    """
    if key in BASE_KEYS:
        node.set_base(*BASE_KEYS[key])
        return f"base {BASE_KEYS[key]}"
    if key in BASE_TURN_KEYS:
        node.set_base(0.0, 0.0, BASE_TURN_KEYS[key])
        return f"turn {BASE_TURN_KEYS[key]:+.0f}"
    if key in ARM_KEYS:
        index, direction = ARM_KEYS[key]
        return node.nudge_joint(index, direction)
    if key in GRIPPER_KEYS:
        return node.nudge_gripper(GRIPPER_KEYS[key])
    if key == " ":
        return node.hold_arm()
    if key == "?":
        print(HELP)
        return None
    # 知らないキーはベースを止める。暴走させないための既定動作。
    node.set_base(0.0, 0.0, 0.0)
    return "停止"


def main() -> None:
    rclpy.init()
    node = TeleopKeyboard()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    print(HELP)
    try:
        while rclpy.ok():
            key = _read_key()
            if key == "\x03":  # Ctrl+C
                break
            status = handle_key(node, key)
            if status is None:
                continue
            print(f"\r{status:<60}", end="", flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ★ 終了時に必ずベースを止める。ここを飛ばすと watchdog の 0.5 秒ぶん
        #   走り続ける。
        node.stop_all()
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

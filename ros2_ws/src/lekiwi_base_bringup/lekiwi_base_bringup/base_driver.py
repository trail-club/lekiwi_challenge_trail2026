"""LeKiwi 3輪オムニベースの ROS 2 ドライバ。

``/cmd_vel`` を購読して 3 個の STS3215 (ID 7/8/9) に速度指令を送り、
オープンループのオドメトリと TF、``/joint_states`` を出す。

オドメトリは **送った指令値の積分** であって実測ではない。飽和と整数丸めは
反映されるが、スリップも外乱も検出できない。自己位置は後段の LiDAR/SLAM が
担当する前提。

安全に関わる設計:
  * ``cmd_vel_timeout_s`` 無指令でウォッチドッグが速度をゼロにする
  * 終了時に「ゼロ送信 → トルク OFF」を必ず実行する
  * ``dry_run:=true`` でシリアルを開かずに全経路を検証できる
  * ``test_wheel_ticks`` で運動学をバイパスし 1 輪ずつ切り分けできる

split 機で 12V 給電中にアーム (ID 1-6, 7.4V 版) をホイールバスへ
繋がないこと。shared 機は全モータが 12V 対応であることを前提とする。
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from lekiwi_hardware_interfaces.msg import WheelCommand
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from lekiwi_base_bringup import kinematics as kin
from lekiwi_base_bringup.sts_bus import STS3215_MODEL_NUMBER, StsBus, StsBusError

#: 車輪の順序。行列の行順・パラメータ配列の順序すべてこれに従う
WHEEL_NAMES = ("left", "back", "right")


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class LekiwiBaseDriver(Node):
    def __init__(self) -> None:
        super().__init__("lekiwi_base_driver")

        # ── パラメータ ──────────────────────────────────────────────
        self.declare_parameter("port", "/dev/lekiwi")
        self.declare_parameter("hardware_backend", "serial")
        self.declare_parameter("baudrate", 1_000_000)
        self.declare_parameter("motor_ids", [7, 8, 9])  # left, back, right
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("base_radius", 0.125)
        # ★ Phase D2 で実機合わせする唯一のノブ。
        #   lekiwi_description の車輪方位角と同じ規約。片方だけ変えないこと。
        self.declare_parameter("wheel_angle_offset_deg", -90.0)
        # ★ Phase D1 で実機合わせする。順序は left, back, right
        self.declare_parameter("wheel_direction_signs", [1.0, 1.0, 1.0])
        self.declare_parameter("max_ticks", 3000)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("cmd_vel_timeout_s", 0.5)
        self.declare_parameter("max_linear_x", 0.26)
        self.declare_parameter("max_linear_y", 0.23)
        self.declare_parameter("max_angular_z", 1.8)
        self.declare_parameter("servo_acceleration", 20)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("diagnostics_period_s", 5.0)
        # 診断用: 非空なら /cmd_vel と運動学を無視して生 tick を書く
        self.declare_parameter("test_wheel_ticks", [0, 0, 0])

        p = self.get_parameter
        self.motor_ids = [int(i) for i in p("motor_ids").value]
        self.wheel_radius = float(p("wheel_radius").value)
        self.base_radius = float(p("base_radius").value)
        self.max_ticks = int(p("max_ticks").value)
        self.cmd_vel_timeout_s = float(p("cmd_vel_timeout_s").value)
        self.odom_frame = str(p("odom_frame").value)
        self.base_frame = str(p("base_frame").value)
        self.publish_tf = bool(p("publish_tf").value)
        self.dry_run = bool(p("dry_run").value)
        self.hardware_backend = str(p("hardware_backend").value)

        if self.hardware_backend not in ("serial", "bridge"):
            raise ValueError("hardware_backend は serial または bridge が必要")
        if self.hardware_backend == "bridge" and self.motor_ids != [7, 8, 9]:
            raise ValueError("bridge backend は canonical wheel IDs [7, 8, 9] が必要")

        if len(self.motor_ids) != 3:
            raise ValueError(f"motor_ids は 3 個必要 (left, back, right): {self.motor_ids}")

        self.direction_signs = np.asarray(p("wheel_direction_signs").value, dtype=float)
        if self.direction_signs.shape != (3,):
            raise ValueError("wheel_direction_signs は 3 要素必要 (left, back, right)")

        self.m = kin.wheel_matrix(self.base_radius, float(p("wheel_angle_offset_deg").value))
        self.m_inv = np.linalg.inv(self.m)

        # ── 実際に到達可能な速度上限を計算して知らせる ────────────────
        #   max_ticks で車輪周速度が頭打ちになるため、設定値より低い場合がある
        wheel_linear_max = math.radians(self.max_ticks / kin.STEPS_PER_DEG) * self.wheel_radius
        self.reachable = {
            "x": wheel_linear_max / abs(self.m[0, 0]),  # 前後は cos 成分で割る
            "y": wheel_linear_max / abs(self.m[1, 1]),  # 横は back 輪が 1.0
            "wz": wheel_linear_max / self.base_radius,
        }

        # ── 状態 ────────────────────────────────────────────────────
        self.cmd = (0.0, 0.0, 0.0)  # vx [m/s], vy [m/s], wz [rad/s]
        self.last_cmd_time = self.get_clock().now()
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.wheel_positions = np.zeros(3)  # 車輪の積算回転角 [rad]
        self.last_tick_time = self.get_clock().now()
        self.bus: StsBus | None = None
        self.wheel_command_pub = None
        self._warned_stale = False

        rate = float(p("control_rate_hz").value)
        if rate <= 0.0:
            raise ValueError(f"control_rate_hz は正の値が必要: {rate}")

        # ── ROS インタフェース ──────────────────────────────────────
        # ★ ハードウェア接続より先に済ませる。トルクを入れた後で例外が出ると、
        #   main() が node を受け取れず safe_stop() が呼ばれないまま
        #   トルクが入りっぱなしになるため。ここは失敗しても安全。
        #
        # シリアルへの同時アクセスを構造的に防ぐため、cmd_vel とタイマーを
        # 同一の排他コールバックグループに入れる
        group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10, callback_group=group)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        if self.hardware_backend == "serial":
            self.create_service(Trigger, "~/recover", self._on_recover, callback_group=group)
        else:
            self.wheel_command_pub = self.create_publisher(
                WheelCommand, "/lekiwi/hardware_wheel_commands", 1
            )

        # ── ハードウェア (ここでトルクが入る) ────────────────────────
        if self.hardware_backend == "bridge":
            self.get_logger().info(
                "bridge backend: シリアルを開かず共有 bridge へ wheel ticks を送ります"
            )
        elif self.dry_run:
            self.get_logger().warn("dry_run: シリアルを開きません。odom と TF のみ出力します")
        else:
            self._connect_bus()

        # ── タイマー (バス確立後に回し始める) ───────────────────────
        # ここから先で失敗したらトルクを入れたまま抜けてしまうので、
        # 必ず停止処理を通してから送出し直す。
        try:
            self.create_timer(1.0 / rate, self._on_timer, callback_group=group)

            diag_period = float(p("diagnostics_period_s").value)
            if (
                diag_period > 0.0
                and not self.dry_run
                and self.hardware_backend == "serial"
            ):
                self.create_timer(diag_period, self._on_diagnostics, callback_group=group)
        except Exception:
            self.safe_stop()
            raise

        self.get_logger().info(
            f"起動: backend={self.hardware_backend} port={p('port').value} "
            f"ids={self.motor_ids} "
            f"r={self.wheel_radius} R={self.base_radius} {rate:.0f}Hz"
        )
        self.get_logger().info(
            f"max_ticks={self.max_ticks} から求まる到達可能な上限: "
            f"x={self.reachable['x']:.3f} m/s, y={self.reachable['y']:.3f} m/s, "
            f"wz={math.degrees(self.reachable['wz']):.1f} deg/s "
            "(これを超える指令は3輪比例縮小されます)"
        )

    # ── ハードウェア ────────────────────────────────────────────────

    def _connect_bus(self) -> None:
        p = self.get_parameter
        self.bus = StsBus(
            str(p("port").value),
            self.motor_ids,
            baudrate=int(p("baudrate").value),
        )
        self.bus.connect()

        found = self.bus.ping_all()
        missing = [i for i, model in found.items() if model == 0]
        wrong = [i for i, model in found.items() if model not in (0, STS3215_MODEL_NUMBER)]
        for motor_id, model in found.items():
            self.get_logger().info(f"  ID {motor_id}: model {model}" if model else f"  ID {motor_id}: 応答なし")
        if missing:
            raise StsBusError(f"応答しないモータ: {missing}。配線と電源を確認してください")
        if wrong:
            self.get_logger().warn(f"想定外のモデル番号: {wrong} (STS3215 は {STS3215_MODEL_NUMBER})")

        accel = int(p("servo_acceleration").value)
        self.bus.configure_velocity_mode(accel if accel >= 0 else None)
        self.get_logger().info("速度モード設定完了、トルク ON")

    def safe_stop(self) -> None:
        """ゼロ送信 → トルク OFF。終了時に必ず呼ぶ。"""
        if self.hardware_backend == "bridge":
            self._publish_wheel_command((0, 0, 0))
            return
        if self.bus is None:
            return
        try:
            self.bus.stop()
            self.bus.disable_torque()
            self.get_logger().info("停止: 速度ゼロ + トルク OFF")
        except Exception as exc:  # 停止処理は何があっても最後まで進める
            self.get_logger().error(f"停止処理でエラー: {exc}")
        finally:
            self.bus.close()
            self.bus = None

    # ── コールバック ────────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist) -> None:
        p = self.get_parameter
        vx = float(np.clip(msg.linear.x, -p("max_linear_x").value, p("max_linear_x").value))
        vy = float(np.clip(msg.linear.y, -p("max_linear_y").value, p("max_linear_y").value))
        wz = float(np.clip(msg.angular.z, -p("max_angular_z").value, p("max_angular_z").value))
        self.cmd = (vx, vy, wz)
        self.last_cmd_time = self.get_clock().now()
        self._warned_stale = False

    def _on_recover(self, _request, response):
        if self.bus is None:
            response.success = False
            response.message = "dry_run または未接続のため復帰できません"
            return response
        try:
            accel = int(self.get_parameter("servo_acceleration").value)
            self.bus.recover(accel if accel >= 0 else None)
            response.success = True
            response.message = "過負荷ラッチを解除し速度モードを再設定しました"
        except StsBusError as exc:
            response.success = False
            response.message = str(exc)
        self.get_logger().info(f"recover: {response.message}")
        return response

    def _on_diagnostics(self) -> None:
        if self.bus is None:
            return
        try:
            for motor_id, values in self.bus.read_diagnostics().items():
                if "error" in values:
                    self.get_logger().warn(f"ID {motor_id}: {values['error']}")
                    continue
                msg = (
                    f"ID {motor_id}: {values['voltage']:.1f}V "
                    f"{values['temperature']:.0f}C load={values['load']}"
                )
                # 12V 版ホイールは 9V 未満で駆動段が働かない。7.4V 版は 8V 上限
                if values["voltage"] < 9.0 or values["temperature"] > 60.0:
                    self.get_logger().warn(msg)
                else:
                    self.get_logger().debug(msg)
        except StsBusError as exc:
            self.get_logger().warn(f"診断読み出し失敗: {exc}")

    # ── 制御ループ ──────────────────────────────────────────────────

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_tick_time).nanoseconds * 1e-9
        self.last_tick_time = now
        if dt <= 0.0 or dt > 1.0:  # 初回およびフリーズ復帰時は積分しない
            dt = 0.0

        test_ticks = list(self.get_parameter("test_wheel_ticks").value)
        if any(test_ticks):
            # 診断モード: 運動学を完全にバイパスして生 tick を書く
            ticks = [int(t) for t in test_ticks[:3]]
        else:
            vx, vy, wz = self._current_command(now)
            ticks = kin.body_to_wheel_ticks(
                vx, vy, math.degrees(wz),
                self.m, self.wheel_radius,
                max_ticks=self.max_ticks,
                direction_signs=self.direction_signs,
            )

        bridge_ready = True
        if self.hardware_backend == "bridge":
            bridge_ready = bool(
                self.wheel_command_pub
                and self.wheel_command_pub.get_subscription_count() > 0
            )
            if not bridge_ready:
                ticks = [0, 0, 0]
            self._publish_wheel_command(ticks)
        elif self.bus is not None:
            try:
                self.bus.sync_write_velocity(dict(zip(self.motor_ids, ticks)))
            except StsBusError as exc:
                self.get_logger().error(f"速度指令の送信失敗: {exc}", throttle_duration_sec=1.0)

        # 実際に送った tick から本当の指令速度を逆算する。
        # 飽和による縮小も整数丸めもここに含まれる。
        act_vx, act_vy, act_wz_degps = kin.wheel_ticks_to_body(
            ticks, self.m_inv, self.wheel_radius, direction_signs=self.direction_signs
        )
        act_wz = math.radians(act_wz_degps)

        integration_dt = dt if bridge_ready else 0.0
        self._integrate_odometry(act_vx, act_vy, act_wz, integration_dt)
        self._integrate_wheels(ticks, integration_dt)
        self._publish(now, act_vx, act_vy, act_wz, ticks)

    def _publish_wheel_command(self, ticks) -> None:
        if self.wheel_command_pub is None:
            return
        message = WheelCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.ticks = [int(value) for value in ticks]
        self.wheel_command_pub.publish(message)

    def _current_command(self, now) -> tuple[float, float, float]:
        age = (now - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_vel_timeout_s:
            if not self._warned_stale and self.cmd != (0.0, 0.0, 0.0):
                self.get_logger().warn(
                    f"{self.cmd_vel_timeout_s:.1f}s 以上 /cmd_vel が来ないので停止します"
                )
                self._warned_stale = True
            self.cmd = (0.0, 0.0, 0.0)
        return self.cmd

    def _integrate_odometry(self, vx: float, vy: float, wz: float, dt: float) -> None:
        if dt == 0.0:
            return
        # 中点法: 区間中央の姿勢で車体速度を世界座標へ回す
        mid_yaw = self.odom_yaw + wz * dt / 2.0
        self.odom_x += (vx * math.cos(mid_yaw) - vy * math.sin(mid_yaw)) * dt
        self.odom_y += (vx * math.sin(mid_yaw) + vy * math.cos(mid_yaw)) * dt
        self.odom_yaw = math.atan2(
            math.sin(self.odom_yaw + wz * dt), math.cos(self.odom_yaw + wz * dt)
        )

    def _integrate_wheels(self, ticks, dt: float) -> None:
        if dt == 0.0:
            return
        for i, tick in enumerate(ticks):
            self.wheel_positions[i] += kin.ticks_to_radps(tick, self.direction_signs[i]) * dt

    def _publish(self, now, vx: float, vy: float, wz: float, ticks) -> None:
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation = yaw_to_quaternion(self.odom_yaw)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        # オープンループなので信頼度は低い。SLAM 側に「信用するな」と伝える
        odom.pose.covariance[0] = 0.1
        odom.pose.covariance[7] = 0.1
        odom.pose.covariance[35] = 0.2
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[35] = 0.1
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.odom_x
            tf.transform.translation.y = self.odom_y
            tf.transform.rotation = yaw_to_quaternion(self.odom_yaw)
            self.tf_broadcaster.sendTransform(tf)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = [f"base_{n}_wheel_joint" for n in WHEEL_NAMES]
        joints.position = [float(x) for x in self.wheel_positions]
        joints.velocity = [
            float(kin.ticks_to_radps(t, self.direction_signs[i])) for i, t in enumerate(ticks)
        ]
        self.joint_pub.publish(joints)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LekiwiBaseDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (StsBusError, ValueError) as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f"起動失敗: {exc}")
    finally:
        if node is not None:
            # ★ destroy_node の前に停止処理を済ませる
            node.safe_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

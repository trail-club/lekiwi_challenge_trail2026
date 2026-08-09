#!/usr/bin/env python
"""leader / IK なしで SO-101 follower の各関節をキーボード操作する。

follower の現在関節角を読み、押しているキーに対応する関節の目標角を少しずつ
増減して送信する。関節空間で直接操作するため、placo・URDF・leader は不要。

操作（各ペアは右回転(+) / 左回転(-)）:
    Q / A : shoulder_pan
    W / S : shoulder_lift
    E / D : elbow_flex
    R / F : wrist_flex
    T / G : wrist_roll
    Y / H : gripper 開 / 閉
    Esc   : 終了

通常は起動した端末の raw stdin からキーを読み取るため、X11/Wayland の設定は不要。
端末が使えない場合だけ X11 の pynput にフォールバックする。回転方向はアームの
取り付け向きによって見え方が異なるため、実機で逆の場合は KEY_BINDINGS の + / -
を入れ替える。

設定は examples/config.toml の [follower] と [teleop] を使用する。
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import threading
import time
import tomllib
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


DEFAULT_CONFIG_PATH = Path(__file__).with_name("config.toml")

ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
MOTORS = ARM_JOINTS + ["gripper"]

# +1 が目標値を増やす方向（右回転）、-1 が目標値を減らす方向（左回転）。
# 実機の見え方が逆なら、この辞書の +1 / -1 を入れ替える。
KEY_BINDINGS = {
    "q": ("shoulder_pan", +1),
    "a": ("shoulder_pan", -1),
    "w": ("shoulder_lift", +1),
    "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", +1),
    "d": ("elbow_flex", -1),
    "r": ("wrist_flex", +1),
    "f": ("wrist_flex", -1),
    "t": ("wrist_roll", +1),
    "g": ("wrist_roll", -1),
    "y": ("gripper", +1),  # LeRobot の gripper は 100 が開
    "h": ("gripper", -1),
}

# SO-101 URDF の可動範囲 [deg]。直接関節操作でも範囲外へ送らない。
JOINT_LIMITS_DEG = {
    "shoulder_pan": (-110.0, 110.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-96.8, 96.8),
    "wrist_flex": (-95.0, 95.0),
    "wrist_roll": (-157.3, 162.8),
}

DEFAULT_CONTROL_FPS = 30
DEFAULT_JOINT_SPEED = 45.0  # arm joint speed [deg/s]
DEFAULT_GRIPPER_SPEED = 60.0  # gripper speed [0-100/s]
DEFAULT_GRIPPER_MIN = 0.0
DEFAULT_GRIPPER_MAX = 100.0

HELP = __doc__


def load_config(path: Path) -> dict:
    """config.toml を読み込み、follower の設定がなければ説明して終了する。"""
    try:
        with path.open("rb") as file:
            return tomllib.load(file)
    except FileNotFoundError:
        raise SystemExit(
            f"設定ファイルが見つかりません: {path}\n"
            "config.toml に [follower] の port と id を記述してください。"
        )


def observation_to_positions(observation: dict[str, float]) -> dict[str, float]:
    """follower の observation をモーター名順の目標角 dict に変換する。"""
    return {motor: float(observation[f"{motor}.pos"]) for motor in MOTORS}


def positions_to_action(positions: dict[str, float]) -> dict[str, float]:
    """モーター名順の目標角 dict を send_action() の形式へ変換する。"""
    return {f"{motor}.pos": float(positions[motor]) for motor in MOTORS}


def update_target(
    target: dict[str, float],
    pressed: set[str],
    dt: float,
    joint_speed: float,
    gripper_speed: float,
    gripper_min: float,
    gripper_max: float,
) -> None:
    """押下キーに応じて target を直接関節空間で更新する。"""
    for key in pressed:
        binding = KEY_BINDINGS.get(key)
        if binding is None:
            continue
        motor, direction = binding
        speed = gripper_speed if motor == "gripper" else joint_speed
        target[motor] += direction * speed * dt

        if motor == "gripper":
            lower, upper = gripper_min, gripper_max
        else:
            lower, upper = JOINT_LIMITS_DEG[motor]
        target[motor] = min(upper, max(lower, target[motor]))


def format_positions(positions: dict[str, float]) -> str:
    """関節角を表示用の短い文字列にする。"""
    return "  ".join(f"{motor}={positions[motor]:7.2f}" for motor in MOTORS)


def connect_follower_without_jump(follower: SO101Follower) -> None:
    """現在位置を Goal_Position にラッチしてから follower を接続する。

    SO101Follower.connect() は configure() 内でトルクを再投入するため、古い
    Goal_Position が残っていると接続直後にそこへ動くことがある。
    """
    follower.bus.connect()
    try:
        if not follower.is_calibrated:
            follower.calibrate()

        # トルクを切った直後の Present_Position を現在の目標として保存し、
        # configure() によるトルク再投入時のジャンプを防ぐ。
        follower.bus.disable_torque()
        current = follower.bus.sync_read("Present_Position")
        follower.bus.sync_write("Goal_Position", current)
        follower.configure()
    except Exception:
        follower.disconnect()
        raise


class KeyboardState:
    """端末 raw stdin または pynput からキー状態を受け取る。"""

    def __init__(self) -> None:
        self._pressed: set[str] = set()
        self._events: list[str] = []
        self._lock = threading.Lock()
        self._quit_requested = False
        self._listener = None
        self._mode = ""
        self._terminal_fd: int | None = None
        self._terminal_settings = None
        self._termios = None

    def start(self) -> None:
        # Wayland では pynput の X11 backend が使えないことがあるため、端末から
        # 起動した場合は raw stdin を優先する。
        try:
            if sys.stdin.isatty():
                import termios
                import tty

                fd = sys.stdin.fileno()
                self._terminal_settings = termios.tcgetattr(fd)
                tty.setcbreak(fd)
                self._terminal_fd = fd
                self._termios = termios
                self._mode = "terminal"
                return
        except (AttributeError, ImportError, OSError, ValueError):
            self._restore_terminal()

        try:
            from pynput import keyboard

            def on_press(key):
                if key == keyboard.Key.esc:
                    self._quit_requested = True
                    return False
                try:
                    character = key.char.lower()
                except (AttributeError, TypeError):
                    return None
                if character in KEY_BINDINGS:
                    with self._lock:
                        self._pressed.add(character)
                return None

            def on_release(key):
                try:
                    character = key.char.lower()
                except (AttributeError, TypeError):
                    return
                with self._lock:
                    self._pressed.discard(character)

            self._listener = keyboard.Listener(on_press=on_press, on_release=on_release)
            self._listener.start()
            self._mode = "pynput"
        except Exception as exc:
            raise RuntimeError(
                "端末入力も X11 キーボードも開始できません。"
                "端末から実行するか、DISPLAY と xhost 設定を確認してください"
            ) from exc

    def poll(self) -> None:
        """端末入力を読み、キーイベントをキューへ追加する。"""
        if self._mode != "terminal" or self._terminal_fd is None:
            return

        while True:
            readable, _, _ = select.select([self._terminal_fd], [], [], 0.0)
            if not readable:
                return
            data = os.read(self._terminal_fd, 64)
            if not data:
                self._quit_requested = True
                return
            with self._lock:
                for value in data:
                    if value == 0x1B:  # Esc
                        self._quit_requested = True
                    else:
                        character = chr(value).lower()
                        if character in KEY_BINDINGS:
                            self._events.append(character)

    def consume_events(self) -> list[str]:
        with self._lock:
            events = self._events
            self._events = []
            return events

    @property
    def terminal_mode(self) -> bool:
        return self._mode == "terminal"

    def snapshot(self) -> set[str]:
        with self._lock:
            return set(self._pressed)

    @property
    def quit_requested(self) -> bool:
        return self._quit_requested

    def stop(self) -> None:
        if self._listener is not None:
            self._listener.stop()
        self._restore_terminal()

    def _restore_terminal(self) -> None:
        if self._terminal_fd is not None and self._terminal_settings is not None:
            self._termios.tcsetattr(
                self._terminal_fd,
                self._termios.TCSADRAIN,
                self._terminal_settings,
            )
        self._terminal_fd = None
        self._terminal_settings = None
        self._termios = None
        self._mode = ""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help=f"設定ファイル (TOML) のパス（既定: {DEFAULT_CONFIG_PATH}）",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    try:
        follower_cfg = cfg["follower"]
    except KeyError:
        raise SystemExit("設定ファイルに [follower] セクションがありません。")

    teleop_cfg = cfg.get("teleop", {})
    control_fps = int(teleop_cfg.get("control_fps", DEFAULT_CONTROL_FPS))
    joint_speed = float(teleop_cfg.get("joint_speed", DEFAULT_JOINT_SPEED))
    gripper_speed = float(
        teleop_cfg.get("gripper_speed", DEFAULT_GRIPPER_SPEED)
    )
    gripper_min = float(teleop_cfg.get("gripper_min", DEFAULT_GRIPPER_MIN))
    gripper_max = float(teleop_cfg.get("gripper_max", DEFAULT_GRIPPER_MAX))
    if control_fps <= 0 or joint_speed < 0 or gripper_speed < 0:
        raise SystemExit("[teleop] control_fps は正、速度は0以上にしてください。")
    if gripper_min >= gripper_max:
        raise SystemExit("[teleop] gripper_min は gripper_max より小さくしてください。")

    follower_port = follower_cfg["port"]
    follower_id = follower_cfg["id"]
    follower = SO101Follower(
        SO101FollowerConfig(
            port=follower_port,
            id=follower_id,
            use_degrees=True,
            cameras={},
        )
    )
    keyboard = KeyboardState()

    print(f"Connecting follower '{follower_id}' on {follower_port} ...")
    connect_follower_without_jump(follower)
    try:
        target = observation_to_positions(follower.get_observation())
        target["gripper"] = min(gripper_max, max(gripper_min, target["gripper"]))

        keyboard.start()
        print("\nConnected.")
        print(HELP)
        print(f"入力方式: {'terminal raw stdin' if keyboard.terminal_mode else 'X11 pynput'}")
        print(
            f"設定: {control_fps} Hz, joint速度 {joint_speed:.1f} deg/s, "
            f"gripper速度 {gripper_speed:.1f}/s"
        )
        print(f"初期目標: {format_positions(target)}")

        period = 1.0 / control_fps
        last_time = time.perf_counter()
        while not keyboard.quit_requested:
            cycle_start = time.perf_counter()
            dt = min(cycle_start - last_time, 2.0 * period)
            last_time = cycle_start

            keyboard.poll()
            if keyboard.terminal_mode:
                # 端末はキーの release を通知しないため、キー入力1回を1周期分の
                # 移動として扱う。キーリピートが有効なら押し続けて連続移動できる。
                events = keyboard.consume_events()
                for key in events:
                    update_target(
                        target,
                        {key},
                        period,
                        joint_speed,
                        gripper_speed,
                        gripper_min,
                        gripper_max,
                    )
                command_pending = bool(events)
            else:
                pressed = keyboard.snapshot()
                update_target(
                    target,
                    pressed,
                    dt,
                    joint_speed,
                    gripper_speed,
                    gripper_min,
                    gripper_max,
                )
                command_pending = bool(pressed)
            if command_pending:
                follower.send_action(positions_to_action(target))
            time.sleep(max(period - (time.perf_counter() - cycle_start), 0.0))
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        keyboard.stop()
        print("\nDisconnecting ...")
        follower.disconnect()


if __name__ == "__main__":
    main()

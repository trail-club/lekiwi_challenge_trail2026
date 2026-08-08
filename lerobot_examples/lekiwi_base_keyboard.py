# アームなし・ホイール3個(ID 7,8,9)だけの LeKiwi ベースをキーボードで直接操縦する。
# host/client 構成は使わず、シリアルバスに直接速度指令を送る。
#
#   uv run python lekiwi_base_keyboard.py
#
# 操作: W/S 前後, A/D 左右, Z/X 旋回, R/F 速度段階, Q または Ctrl+C 終了
# 注意: 12V 給電中はアーム(7.4V版)を絶対にバスへ再接続しないこと。

import time

import numpy as np
from pynput import keyboard as pynput_keyboard

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

PORT = "/dev/tty.usbmodem5A7A0178741"
FPS = 30

# 速度プリセット (R で上げ、F で下げる)。従来の最速 0.4 m/s の上に 0.6 を追加
SPEED_LEVELS = [
    {"xy": 0.1, "theta": 30},
    {"xy": 0.25, "theta": 60},
    {"xy": 0.4, "theta": 90},
    {"xy": 0.6, "theta": 120},
]
START_SPEED_INDEX = 2  # 0.4 m/s から開始

WHEEL_RADIUS = 0.05  # m
BASE_RADIUS = 0.125  # m
MAX_RAW = 3000


def degps_to_raw(degps: float) -> int:
    speed_int = int(round(degps * 4096.0 / 360.0))
    return max(-0x8000, min(0x7FFF, speed_int))


def body_to_wheel_raw(x: float, y: float, theta_degps: float) -> dict[str, int]:
    """lerobot の LeKiwi._body_to_wheel_raw と同じ変換。"""
    theta_rad = theta_degps * (np.pi / 180.0)
    velocity_vector = np.array([x, y, theta_rad])
    angles = np.radians(np.array([240, 0, 120]) - 90)
    m = np.array([[np.cos(a), np.sin(a), BASE_RADIUS] for a in angles])
    wheel_degps = (m.dot(velocity_vector) / WHEEL_RADIUS) * (180.0 / np.pi)

    raw_floats = [abs(d) * 4096.0 / 360.0 for d in wheel_degps]
    if max(raw_floats) > MAX_RAW:
        wheel_degps = wheel_degps * (MAX_RAW / max(raw_floats))

    raw = [degps_to_raw(d) for d in wheel_degps]
    return {"base_left_wheel": raw[0], "base_back_wheel": raw[1], "base_right_wheel": raw[2]}


def main():
    bus = FeetechMotorsBus(
        port=PORT,
        motors={
            "base_left_wheel": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
            "base_back_wheel": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
            "base_right_wheel": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
        },
    )
    bus.connect()

    # 速度モードに設定してトルクON
    bus.disable_torque(num_retry=5)
    for name in bus.motors:
        bus.write("Operating_Mode", name, 1, num_retry=5)  # 1 = velocity mode
    bus.enable_torque(num_retry=5)

    pressed: set[str] = set()
    speed_index = START_SPEED_INDEX
    quit_flag = False

    def on_press(key):
        nonlocal speed_index, quit_flag
        try:
            ch = key.char.lower()
        except AttributeError:
            return
        if ch == "r" and ch not in pressed:
            speed_index = min(speed_index + 1, len(SPEED_LEVELS) - 1)
            print(f"speed: {SPEED_LEVELS[speed_index]}")
        if ch == "f" and ch not in pressed:
            speed_index = max(speed_index - 1, 0)
            print(f"speed: {SPEED_LEVELS[speed_index]}")
        if ch == "q":
            quit_flag = True
        pressed.add(ch)

    def on_release(key):
        try:
            pressed.discard(key.char.lower())
        except AttributeError:
            pass

    listener = pynput_keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    print(f"操作開始 (速度: {SPEED_LEVELS[speed_index]})")
    print("W/S: 前後, A/D: 左右, Z/X: 旋回, R/F: 速度, Q: 終了")
    try:
        while not quit_flag:
            t0 = time.perf_counter()
            spd = SPEED_LEVELS[speed_index]

            x = spd["xy"] * (("w" in pressed) - ("s" in pressed))
            y = spd["xy"] * (("a" in pressed) - ("d" in pressed))
            theta = spd["theta"] * (("z" in pressed) - ("x" in pressed))

            bus.sync_write("Goal_Velocity", body_to_wheel_raw(x, y, theta), normalize=False)

            time.sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
    except KeyboardInterrupt:
        pass
    finally:
        bus.sync_write("Goal_Velocity", body_to_wheel_raw(0.0, 0.0, 0.0), normalize=False)
        time.sleep(0.2)
        bus.disable_torque(num_retry=5)
        bus.disconnect()
        listener.stop()
        print("停止しました")


if __name__ == "__main__":
    main()

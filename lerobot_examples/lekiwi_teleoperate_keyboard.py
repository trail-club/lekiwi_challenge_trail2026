# リーダーアームなしで LeKiwi (Wired 版) をキーボードだけで操縦するスクリプト。
# ベースを WASD (前後左右) / Z X (旋回) / R F (速度変更) で動かす。
# アームは接続時の姿勢を保持する。
#
# 先に別ターミナルでホストを起動しておくこと:
#   uv run python -m lerobot.robots.lekiwi.lekiwi_host \
#       --robot.id=my_kiwi \
#       --robot.port=/dev/tty.usbmodem5A7A0178741 \
#       --robot.cameras='{
#           "front": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30, "rotation": 180},
#           "wrist": {"type": "opencv", "index_or_path": 1, "width": 480, "height": 640, "fps": 30, "rotation": 90}
#       }' \
#       --host.connection_time_s=3600
#
# macOS ではキーボード入力の取得に「入力監視」権限が必要:
#   システム設定 > プライバシーとセキュリティ > 入力監視 でターミナルを許可

import time

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

FPS = 30


def main():
    robot_config = LeKiwiClientConfig(remote_ip="127.0.0.1", id="my_kiwi")
    keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")

    robot = LeKiwiClient(robot_config)
    keyboard = KeyboardTeleop(keyboard_config)

    robot.connect()
    keyboard.connect()

    init_rerun(session_name="lekiwi_teleop_keyboard")

    if not robot.is_connected or not keyboard.is_connected:
        raise ValueError("Robot or keyboard is not connected!")

    # アームは接続時の姿勢を保持し続ける
    first_obs = robot.get_observation()
    arm_hold_action = {
        k: v for k, v in first_obs.items() if k.startswith("arm_") and k.endswith(".pos")
    }
    print("Holding arm pose:", {k: round(float(v), 1) for k, v in arm_hold_action.items()})

    print("Starting teleop loop... (W/A/S/D: 移動, Z/X: 旋回, R/F: 速度, Ctrl+C: 終了)")
    while True:
        t0 = time.perf_counter()

        observation = robot.get_observation()

        keyboard_keys = keyboard.get_action()
        base_action = robot._from_keyboard_to_base_action(keyboard_keys)
        if not base_action:
            base_action = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}

        action = {**arm_hold_action, **base_action}
        _ = robot.send_action(action)

        log_rerun_data(observation=observation, action=action)

        precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))


if __name__ == "__main__":
    main()

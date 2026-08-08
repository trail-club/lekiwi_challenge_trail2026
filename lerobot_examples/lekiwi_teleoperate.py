# 公式 examples/lekiwi/teleoperate.py (v0.5.1 相当) を Wired 版 LeKiwi 用にしたもの。
# pip インストール済みの lerobot だけで動く(クローン不要)。
#
# Wired 版では host も Mac 上で動かす。先に別ターミナルでホストを起動しておくこと:
#   uv run python -m lerobot.robots.lekiwi.lekiwi_host \
#       --robot.id=my_kiwi \
#       --robot.port=/dev/tty.usbmodem5A7A0178741 \
#       --robot.cameras='{
#           "front": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30, "rotation": 180},
#           "wrist": {"type": "opencv", "index_or_path": 1, "width": 480, "height": 640, "fps": 30, "rotation": 90}
#       }' \
#       --host.connection_time_s=3600
#   (connection_time_s のデフォルトは 30 秒で自動終了するので長めに指定する)
#   カメラの index は `uv run lerobot-find-cameras opencv` で確認できる
#   (撮影サンプルは outputs/captured_images/ に保存される)。
#
# 要編集: leader arm の port / 各 id を自分の環境に合わせる。

import time

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

FPS = 30


def main():
    # Create the robot and teleoperator configurations
    # Wired 版: host が同じ Mac 上で動くので localhost に接続する
    robot_config = LeKiwiClientConfig(remote_ip="127.0.0.1", id="my_kiwi")
    teleop_arm_config = SO101LeaderConfig(port="/dev/tty.usbmodemXXXXXXXX", id="my_leader_arm")
    keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")

    # Initialize the robot and teleoperator
    robot = LeKiwiClient(robot_config)
    leader_arm = SO101Leader(teleop_arm_config)
    keyboard = KeyboardTeleop(keyboard_config)

    # Connect to the robot and teleoperator
    robot.connect()
    leader_arm.connect()
    keyboard.connect()

    # Init rerun viewer
    init_rerun(session_name="lekiwi_teleop")

    if not robot.is_connected or not leader_arm.is_connected or not keyboard.is_connected:
        raise ValueError("Robot or teleop is not connected!")

    print("Starting teleop loop...")
    while True:
        t0 = time.perf_counter()

        # Get robot observation
        observation = robot.get_observation()

        # Get teleop action
        # Arm
        arm_action = leader_arm.get_action()
        arm_action = {f"arm_{k}": v for k, v in arm_action.items()}
        # Keyboard
        keyboard_keys = keyboard.get_action()
        base_action = robot._from_keyboard_to_base_action(keyboard_keys)

        action = {**arm_action, **base_action} if len(base_action) > 0 else arm_action

        # Send action to robot
        _ = robot.send_action(action)

        # Visualize
        log_rerun_data(observation=observation, action=action)

        precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))


if __name__ == "__main__":
    main()

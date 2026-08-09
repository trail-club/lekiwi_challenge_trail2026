#!/usr/bin/env python
"""ターミナルのコマンドで leader の関節角度を記録し、follower をその記録へ移動させる。

対話コマンド:
    r / record  : leader の現在の関節角度を記録（最新として保存）
    m / move    : follower を直前（最新）の記録へ移動
    s / show    : leader の現在の関節角度を表示（記録しない）
    l / list    : これまでの記録を一覧表示
    q / quit    : 終了

leader / follower のモーター構成・キー形式（"<motor>.pos"）は一致しているため、
leader.get_action() の戻り値をそのまま follower.send_action() に渡せる。

参考: https://huggingface.co/docs/lerobot
"""

import argparse
import time
import tomllib
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

# --- 設定 -------------------------------------------------------------------
# ポート/ID・補間設定は config.toml に記述する（同ディレクトリの config.toml が既定）。
DEFAULT_CONFIG_PATH = Path(__file__).with_name("config.toml")

# config.toml に [motion] が無い場合に使う既定値。
DEFAULT_MOVE_DURATION = 2.0  # 移動にかける時間 [s]
DEFAULT_MOVE_FPS = 30  # 補間の更新周波数 [Hz]

HELP = __doc__


def load_config(path: Path) -> dict:
    """config.toml を読み込んで dict を返す。見つからなければ分かりやすく失敗させる。"""
    try:
        with path.open("rb") as f:
            return tomllib.load(f)
    except FileNotFoundError:
        raise SystemExit(
            f"設定ファイルが見つかりません: {path}\n"
            f"config.toml を作成し、leader/follower の port と id を記述してください。"
        )


def format_action(action: dict[str, float]) -> str:
    """{"shoulder_pan.pos": 12.3, ...} を読みやすい1行文字列にする。"""
    return "  ".join(
        f"{key.removesuffix('.pos')}={val:7.2f}" for key, val in action.items()
    )


def move_follower_to(
    follower: SO101Follower,
    target: dict[str, float],
    duration: float,
    fps: int,
) -> None:
    """follower を target ポーズへ補間しながら滑らかに移動させる。"""
    # follower の現在角度を読む（カメラ無しなので .pos キーのみ）
    obs = follower.get_observation()
    current = {key: obs[key] for key in target if key in obs}

    steps = max(1, int(duration * fps))
    period = 1.0 / fps

    for i in range(1, steps + 1):
        alpha = i / steps
        interp = {
            key: current.get(key, target[key]) * (1.0 - alpha) + target[key] * alpha
            for key in target
        }
        follower.send_action(interp)
        time.sleep(period)


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
    leader_cfg = cfg["leader"]
    follower_cfg = cfg["follower"]
    motion = cfg.get("motion", {})
    move_duration = float(motion.get("move_duration", DEFAULT_MOVE_DURATION))
    move_fps = int(motion.get("move_fps", DEFAULT_MOVE_FPS))

    leader_port, leader_id = leader_cfg["port"], leader_cfg["id"]
    follower_port, follower_id = follower_cfg["port"], follower_cfg["id"]

    leader = SO101Leader(SO101LeaderConfig(port=leader_port, id=leader_id))
    follower = SO101Follower(SO101FollowerConfig(port=follower_port, id=follower_id))

    print(f"Connecting leader '{leader_id}' on {leader_port} ...")
    leader.connect()
    print(f"Connecting follower '{follower_id}' on {follower_port} ...")
    follower.connect()
    print("\nConnected.")
    print(HELP)

    records: list[dict[str, float]] = []

    try:
        while True:
            try:
                cmd = input("\n> ").strip().lower()
            except EOFError:
                break

            if cmd in ("r", "record"):
                action = leader.get_action()
                records.append(action)
                print(f"[記録 #{len(records)}] {format_action(action)}")

            elif cmd in ("m", "move"):
                if not records:
                    print("記録がありません。先に 'r' で記録してください。")
                    continue
                target = records[-1]
                print(f"follower を記録 #{len(records)} へ移動中 ... {format_action(target)}")
                move_follower_to(follower, target, move_duration, move_fps)
                print("移動完了。")

            elif cmd in ("s", "show"):
                print(f"[leader 現在角度] {format_action(leader.get_action())}")

            elif cmd in ("l", "list"):
                if not records:
                    print("記録はまだありません。")
                for idx, rec in enumerate(records, start=1):
                    marker = " <- 直前" if idx == len(records) else ""
                    print(f"  #{idx}: {format_action(rec)}{marker}")

            elif cmd in ("q", "quit", "exit"):
                break

            elif cmd in ("h", "help", "?"):
                print(HELP)

            elif cmd == "":
                continue

            else:
                print(f"不明なコマンド: {cmd!r}  ('h' でヘルプ)")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("\nDisconnecting ...")
        follower.disconnect()
        leader.disconnect()


if __name__ == "__main__":
    main()

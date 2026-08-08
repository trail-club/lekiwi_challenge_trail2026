#!/usr/bin/env python
"""leader の EE(エンドエフェクタ)位置を記録し、IK(逆運動学)で解いて follower をそこへ動かす。

仕組み:
    record : leader の現在関節角度を読み、順運動学(FK)で EE の 4x4 姿勢を計算して記録する。
    move   : follower の現在角度を初期値として、記録した EE 姿勢へ逆運動学(IK)を反復で解き、
             目標関節角度を求める。その後、現在角度から目標角度へ補間しながら滑らかに動かす。

leader と follower は同一機構(SO101)なので、本来は leader の関節角をそのまま渡せば一致するが、
ここでは「EE 位置を IK で解いて追従する」ことを目的に、あえて FK→IK を経由する。

対話コマンド:
    r / record  : leader の現在 EE 姿勢を記録（最新として保存）
    m / move    : follower を直前（最新）の記録 EE 姿勢へ IK で移動
    s / show    : leader の現在 EE 位置を表示（記録しない）
    l / list    : これまでの記録を一覧表示
    q / quit    : 終了

注意:
    - kinematics(placo) は関節角を「度」で扱うため、leader/follower とも use_degrees=True で接続する。
    - 設定（ポート/ID・URDF・IK の重み等）は config.toml に記述する。

参考: lerobot.model.kinematics.RobotKinematics, https://huggingface.co/docs/lerobot
"""

import argparse
import time
import tomllib
from pathlib import Path

import numpy as np

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

# --- 設定 -------------------------------------------------------------------
# config.toml はこのスクリプトと同じディレクトリの config.toml が既定。
DEFAULT_CONFIG_PATH = Path(__file__).with_name("config.toml")
# URDF の相対パスはリポジトリのルート（lerobot_examples/ の一つ上）基準で解決する。
REPO_ROOT = Path(__file__).resolve().parent.parent

# SO101 のモーター構成。IK で解くアーム関節（gripper を除く 5 自由度）と、配列の並び順。
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
MOTORS = ARM_JOINTS + ["gripper"]  # action/observation 配列の順序（末尾が gripper）
GRIPPER_IDX = MOTORS.index("gripper")

# config.toml に該当セクションが無い場合の既定値。
DEFAULT_MOVE_DURATION = 2.0  # 移動にかける時間 [s]
DEFAULT_MOVE_FPS = 30  # 補間の更新周波数 [Hz]
DEFAULT_TARGET_FRAME = "gripper_frame_link"
DEFAULT_POSITION_WEIGHT = 1.0
DEFAULT_ORIENTATION_WEIGHT = 0.01

# IK 反復の設定（この実装の inverse_kinematics は 1 回で 1 ステップしか進まないため反復する）
IK_MAX_ITERS = 100
IK_POS_TOL = 1e-4  # 位置の収束判定 [m]
IK_WARN_POS_ERR = 0.02  # この値[m]を超えて未収束なら警告し確認を求める

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


def build_kinematics(kin_cfg: dict) -> RobotKinematics:
    """config の [kinematics] から RobotKinematics を構築する。"""
    urdf_path = Path(kin_cfg.get("urdf_path", "lerobot_examples/SO101/so101_new_calib.urdf"))
    if not urdf_path.is_absolute():
        urdf_path = REPO_ROOT / urdf_path
    if not urdf_path.exists():
        raise SystemExit(f"URDF が見つかりません: {urdf_path}")

    target_frame = kin_cfg.get("target_frame", DEFAULT_TARGET_FRAME)
    try:
        return RobotKinematics(
            urdf_path=str(urdf_path),
            target_frame_name=target_frame,
            joint_names=ARM_JOINTS,
        )
    except ImportError as e:
        raise SystemExit(
            f"placo の読み込みに失敗しました: {e}\n"
            f"インストール:  uv pip install 'placo>=0.9.6,<0.9.17'\n"
            f"(macOS で liburdfdom*.dylib のロードに失敗する場合は README の対処を参照)"
        )


def action_to_array(action: dict[str, float]) -> np.ndarray:
    """{"shoulder_pan.pos": 12.3, ...} を MOTORS 順の numpy 配列（度）にする。"""
    return np.array([action[f"{m}.pos"] for m in MOTORS], dtype=float)


def array_to_action(arr: np.ndarray) -> dict[str, float]:
    """MOTORS 順の配列を {"<motor>.pos": 値} の dict に戻す。"""
    return {f"{m}.pos": float(arr[i]) for i, m in enumerate(MOTORS)}


def format_pose(pose: np.ndarray) -> str:
    """4x4 姿勢行列の位置成分を読みやすい1行にする。"""
    x, y, z = pose[:3, 3]
    return f"x={x:7.4f}  y={y:7.4f}  z={z:7.4f}  [m]"


def solve_ik(
    kin: RobotKinematics,
    current_arr: np.ndarray,
    target_pose: np.ndarray,
    position_weight: float,
    orientation_weight: float,
) -> tuple[np.ndarray, float]:
    """follower の現在角度を初期値に、target_pose へ IK を反復して解く。

    Returns:
        (目標関節角度配列[度, MOTORS 順], 収束時の位置誤差[m])
    """
    q = current_arr.copy()
    err = float("inf")
    for _ in range(IK_MAX_ITERS):
        q = kin.inverse_kinematics(q, target_pose, position_weight, orientation_weight)
        err = float(np.linalg.norm(kin.forward_kinematics(q)[:3, 3] - target_pose[:3, 3]))
        if err < IK_POS_TOL:
            break
    return q, err


def move_follower_to_joints(
    follower: SO101Follower, target_arr: np.ndarray, duration: float, fps: int
) -> None:
    """follower を現在角度から target_arr へ関節空間で補間しながら滑らかに動かす。"""
    obs = follower.get_observation()
    current = np.array([obs[f"{m}.pos"] for m in MOTORS], dtype=float)

    steps = max(1, int(duration * fps))
    period = 1.0 / fps
    for i in range(1, steps + 1):
        alpha = i / steps
        interp = current * (1.0 - alpha) + target_arr * alpha
        follower.send_action(array_to_action(interp))
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
    kin_cfg = cfg.get("kinematics", {})

    move_duration = float(motion.get("move_duration", DEFAULT_MOVE_DURATION))
    move_fps = int(motion.get("move_fps", DEFAULT_MOVE_FPS))
    position_weight = float(kin_cfg.get("position_weight", DEFAULT_POSITION_WEIGHT))
    orientation_weight = float(kin_cfg.get("orientation_weight", DEFAULT_ORIENTATION_WEIGHT))

    leader_port, leader_id = leader_cfg["port"], leader_cfg["id"]
    follower_port, follower_id = follower_cfg["port"], follower_cfg["id"]

    # IK ソルバ（placo + URDF）を先に構築。失敗するならロボット接続前に止める。
    print("Loading kinematics (placo) ...")
    kin = build_kinematics(kin_cfg)

    # use_degrees=True: 関節角を「度」で読み書きし、kinematics の入出力と単位を揃える。
    leader = SO101Leader(SO101LeaderConfig(port=leader_port, id=leader_id, use_degrees=True))
    follower = SO101Follower(
        SO101FollowerConfig(port=follower_port, id=follower_id, use_degrees=True)
    )

    print(f"Connecting leader '{leader_id}' on {leader_port} ...")
    leader.connect()
    print(f"Connecting follower '{follower_id}' on {follower_port} ...")
    follower.connect()
    print("\nConnected.")
    print(HELP)

    # 各記録は EE 姿勢(4x4)と、その時の gripper 開度(度/0-100)を保持する。
    records: list[dict] = []

    try:
        while True:
            try:
                cmd = input("\n> ").strip().lower()
            except EOFError:
                break

            if cmd in ("r", "record"):
                arr = action_to_array(leader.get_action())
                pose = kin.forward_kinematics(arr)
                records.append({"pose": pose, "gripper": float(arr[GRIPPER_IDX])})
                print(f"[記録 #{len(records)}] EE {format_pose(pose)}")

            elif cmd in ("m", "move"):
                if not records:
                    print("記録がありません。先に 'r' で記録してください。")
                    continue
                target = records[-1]
                target_pose = target["pose"]
                print(f"記録 #{len(records)} の EE へ IK を解いています ... {format_pose(target_pose)}")

                current = np.array(
                    [follower.get_observation()[f"{m}.pos"] for m in MOTORS], dtype=float
                )
                target_arr, err = solve_ik(
                    kin, current, target_pose, position_weight, orientation_weight
                )
                # gripper は IK 対象外。記録した leader の gripper 開度に合わせる。
                target_arr[GRIPPER_IDX] = target["gripper"]

                arm_deg = "  ".join(
                    f"{j}={target_arr[i]:7.2f}" for i, j in enumerate(ARM_JOINTS)
                )
                print(f"  IK 解(度): {arm_deg}  gripper={target_arr[GRIPPER_IDX]:.1f}")
                print(f"  位置誤差: {err * 1000:.2f} mm")

                if err > IK_WARN_POS_ERR:
                    ans = input(
                        f"  警告: IK が十分収束していません（誤差 {err * 1000:.1f} mm）。"
                        f"それでも移動しますか? (y/N) "
                    ).strip().lower()
                    if ans != "y":
                        print("  移動を中止しました。")
                        continue

                print("  follower を移動中 ...")
                move_follower_to_joints(follower, target_arr, move_duration, move_fps)
                print("移動完了。")

            elif cmd in ("s", "show"):
                pose = kin.forward_kinematics(action_to_array(leader.get_action()))
                print(f"[leader 現在 EE] {format_pose(pose)}")

            elif cmd in ("l", "list"):
                if not records:
                    print("記録はまだありません。")
                for idx, rec in enumerate(records, start=1):
                    marker = " <- 直前" if idx == len(records) else ""
                    print(f"  #{idx}: EE {format_pose(rec['pose'])}{marker}")

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

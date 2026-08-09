"""異常終了したあとにホイールとアームを解放するコマンド。

    ros2 run lekiwi_so101_bringup release_all --bus-mode split
    ros2 run lekiwi_so101_bringup release_all --bus-mode shared --yes --only wheels
    ros2 run lekiwi_so101_bringup release_all --bus-mode shared --dry-run

────────────────────────────────────────────────────────────────────────
なぜこれが要るのか
────────────────────────────────────────────────────────────────────────
停止処理はすべて Python の ``finally`` にある。分岐しているのは
「``finally`` に到達できるか」だけ:

    経路                                   アーム        ホイール
    Ctrl+C / SIGTERM / Python 例外         トルク OFF    ゼロ + トルク OFF
    SIGKILL (docker kill / OOM / 強制削除) **ON のまま**  **最後の指令速度のまま**

STS3215 には**コマンドウォッチドッグが無い**。SIGKILL でプロセスが消えても
サーボは最後に受け取った Goal_Velocity を保持し続けるので、ホイールは
物理的に回り続ける。アームは保持トルクが入ったまま凍る。

ROS のノードは死んでいる前提なので、このコマンドは **ROS を一切使わず**
シリアルポートを直接開く。

────────────────────────────────────────────────────────────────────────
★ アームはトルクを切ると落ちる
────────────────────────────────────────────────────────────────────────
凍ったアームを解放するのが目的なので、それが正しい挙動。**人が支えている
前提**で、``--yes`` が無ければ確認を求める。

────────────────────────────────────────────────────────────────────────
★ ホイールとアームで処理が違う
────────────────────────────────────────────────────────────────────────
ホイール: ``stop()`` (Goal_Velocity=0) → ``disable_torque()``
アーム  : ``disable_torque()`` **のみ**

アームに ``stop()`` を使わない。``stop()`` が書く Goal_Velocity は、速度モード
では速度指令だが、**位置モードでは速度上限**であって意味が違う。0 を書いたときの
挙動がファームウェア依存になるので、トルクを切るだけにする。
"""

from __future__ import annotations

import argparse
import glob
import os
import sys

from lekiwi_base_bringup.sts_bus import StsBus, StsBusError

WHEEL_PORT = "/dev/lekiwi"
SHARED_PORT = "/dev/lekiwi"
WHEEL_IDS = [7, 8, 9]

ARM_PORT = "/dev/so101_follower"
ARM_IDS = [1, 2, 3, 4, 5, 6]

BAUDRATE = 1_000_000


class Outcome:
    """1 本のバスに対する処理結果。"""

    def __init__(self, label: str, port: str, ids: list[int]) -> None:
        self.label = label
        self.port = port
        self.ids = ids
        self.error: str | None = None
        self.torque: dict[int, int | None] = {}
        # --dry-run。読んだだけで何も書いていない。表示の文言だけが変わる。
        self.read_only = False

    @property
    def released(self) -> bool:
        """全 ID の Torque_Enable が 0 だと**読み戻せた**ときだけ True。

        ★ 判定の根拠は読み戻しだけ。`error` の有無では決めない。
          - `disable_torque()` は ID ごとの失敗を握り潰すので、「呼べた」を
            根拠にすると嘘の成功報告になる。読めなかった ID (None) も成功にしない。
          - 逆に、途中で通信が切れて `error` が付いても、全 ID が 0 だと
            読めているならハードウェアは実際に解放されている。
            そこを失敗扱いにすると、解放済みの機体に対して人が
            もう一度アームを落としに行くことになる。
        """
        if set(self.torque) != set(self.ids):
            return False
        return all(value == 0 for value in self.torque.values())

    def report(self) -> None:
        mode = "読み出しのみ" if self.read_only else "解放"
        print(f"── {self.label} ({self.port}, ID {self.ids}, {mode}) " + "─" * 14)
        if self.error is not None:
            print(f"   ★ {self.error}")
        if not self.torque:
            return
        for motor_id in self.ids:
            value = self.torque.get(motor_id)
            if value is None:
                print(f"   ID {motor_id}: ★ 読み出せない (配線・電源・ID を確認)")
            elif value == 0:
                print(f"   ID {motor_id}: Torque_Enable=0  (脱力)")
            else:
                print(f"   ID {motor_id}: Torque_Enable={value}  (★ トルクが入っている)")


def port_holder(port: str) -> str | None:
    """このポートを**いま開いているプロセス**を返す。誰も開いていなければ None。

    ★ なぜ要るのか
      Feetech のバスは半二重で、マスタは 1 つしか居られない。ROS のノードが
      ポートを開いたまま release_all が読み書きすると**パケットが混線し、
      ブリッジが通信異常と判定してトルクを切る**（＝アームがその場で落ちる）。

    ★ なぜ「コンテナが動いているか」で判定しないのか
      いちばん多い故障は「**launch だけが落ちて、コンテナは生きている**」。
      このときポートは空いているので、**コンテナを落とさずに解放できる**のが
      正しい挙動。コンテナの有無で断ると、この一番ありふれた場合に
      復帰手段が使えなくなる（実際にそう作ってしまい、直したのがこれ）。

    /proc を舐めるだけで lsof や fuser に依存しない。コンテナからは
    自分のコンテナのプロセスしか見えないので、判定範囲もちょうどよい
    （compose.yaml は pid: host を使っていない）。
    """
    real = os.path.realpath(port)
    self_pid = str(os.getpid())
    for proc in glob.glob("/proc/[0-9]*"):
        pid = os.path.basename(proc)
        if pid == self_pid:
            continue
        try:
            for fd in glob.glob(f"{proc}/fd/*"):
                if os.path.realpath(fd) != real:
                    continue
                try:
                    raw = open(f"{proc}/cmdline", "rb").read()
                except OSError:
                    raw = b""
                cmd = raw.replace(b"\0", b" ").decode(errors="replace").strip()
                return f"PID {pid}" + (f" ({cmd})" if cmd else "")
        except OSError:
            # プロセスが読み取り中に消えるのは普通のこと。無視して次へ。
            continue
    return None


def diagnose_port(port: str) -> str | None:
    """ポートを触れない理由を人間向けに返す。触ってよさそうなら None。"""
    if not os.path.exists(port):
        return (
            f"{port} が存在しない。udev ルールが当たっているか、"
            "コンテナに devices: で渡っているかを確認すること"
        )
    if not os.access(port, os.R_OK | os.W_OK):
        return f"{port} に読み書き権限が無い (dialout グループの GID を確認)"
    holder = port_holder(port)
    if holder is not None:
        return (
            f"{port} は {holder} が開いています。\n"
            "      Feetech のバスはマスタが 1 つだけ。ここで触ると混線し、\n"
            "      ブリッジが通信異常と判定してトルクを切ります"
            "（★ アームがその場で落ちます）。\n"
            "      先に launch を止めてから実行すること"
            "（コンテナは落とさなくて構いません）。"
        )
    return None


def release_bus(
    label: str, port: str, ids: list[int], *, zero_velocity: bool, read_only: bool = False
) -> Outcome:
    """1 本のバスを解放する。失敗しても例外は投げず Outcome に載せる。

    ``read_only=True`` なら ``Torque_Enable`` を読むだけで**一切書き込まない**。
    解放する前後で状態を比べるために使う (``--dry-run``)。
    """
    outcome = Outcome(label, port, ids)
    outcome.read_only = read_only

    reason = diagnose_port(port)
    if reason is not None:
        outcome.error = reason
        return outcome

    bus = StsBus(port, ids, baudrate=BAUDRATE)
    try:
        bus.connect()
    except Exception as exc:  # noqa: BLE001
        # ★ StsBusError だけでは足りない。scservo_sdk は pyserial の
        #   SerialException をそのまま通すので、ポートがシリアルでない場合や
        #   排他ロックされている場合に**トレースバックで死ぬ**。
        #   これは異常終了からの復帰コマンドで、動かないときこそ
        #   「何をすればいいか」を出さなければならない。握って助言に変える。
        outcome.error = (
            f"{type(exc).__name__}: {exc}\n"
            "      ポートは開いていないのに接続できませんでした。\n"
            "      配線・電源・ボーレート・ポート名を確認すること\n"
            "      (誰かが掴んでいる場合は diagnose_port が先に検出します)。"
        )
        return outcome

    try:
        if not read_only:
            if zero_velocity:
                # ★ ホイールだけ。速度モードなので Goal_Velocity=0 が「止まれ」になる。
                #   トルクを切る前にゼロを入れておかないと、切った瞬間まで回り続ける。
                bus.stop()
            bus.disable_torque()
        outcome.torque = bus.read_torque_enable()
    except Exception as exc:  # noqa: BLE001
        # 途中で通信が死んでも、読めたぶんの Torque_Enable は報告する。
        # torque が空なら released は False になるので、嘘の成功にはならない。
        outcome.error = f"{type(exc).__name__}: {exc}"
    finally:
        bus.close()
    return outcome


def confirm(assume_yes: bool) -> bool:
    """アームが落ちることの確認。--yes が無く、対話端末も無ければ中止する。"""
    if assume_yes:
        return True
    print("★ アームのトルクを切ります。支えが無ければ**その場で落ちます**。")
    print("  低い姿勢にするか、人が支えてから続けること。")
    if not sys.stdin.isatty():
        print("  端末が対話的でないので中止しました。--yes を付けて実行してください。")
        return False
    try:
        answer = input("  続けますか? [y/N] ").strip().lower()
    except EOFError:
        return False
    return answer in ("y", "yes")


def release_shared_bus(
    port: str,
    *,
    do_wheels: bool,
    do_arm: bool,
    read_only: bool,
    assume_yes: bool,
) -> list[Outcome]:
    """Open the shared nine-motor bus once and release only selected ID groups."""
    outcomes = []
    wheel_outcome = Outcome("ホイール", port, WHEEL_IDS) if do_wheels else None
    arm_outcome = Outcome("アーム", port, ARM_IDS) if do_arm else None
    for outcome in (wheel_outcome, arm_outcome):
        if outcome is not None:
            outcome.read_only = read_only
            outcomes.append(outcome)

    reason = diagnose_port(port)
    if reason is not None:
        for outcome in outcomes:
            outcome.error = reason
        return outcomes

    bus = StsBus(port, ARM_IDS + WHEEL_IDS, baudrate=BAUDRATE)
    try:
        bus.connect()
    except Exception as exc:  # noqa: BLE001
        message = f"{type(exc).__name__}: {exc}"
        bus.close()
        for outcome in outcomes:
            outcome.error = message
        return outcomes

    try:
        if read_only:
            if wheel_outcome is not None:
                wheel_outcome.torque = bus.read_torque_enable(WHEEL_IDS)
            if arm_outcome is not None:
                arm_outcome.torque = bus.read_torque_enable(ARM_IDS)
            return outcomes

        # The arm IDs must never receive Goal_Velocity. Stop and verify the
        # wheel subset before waiting for any arm-drop confirmation.
        if wheel_outcome is not None:
            try:
                bus.stop(WHEEL_IDS)
                bus.disable_torque(WHEEL_IDS)
                wheel_outcome.torque = bus.read_torque_enable(WHEEL_IDS)
            except Exception as exc:  # noqa: BLE001
                wheel_outcome.error = f"{type(exc).__name__}: {exc}"

        if arm_outcome is not None:
            if not confirm(assume_yes):
                arm_outcome.error = "操作者がアームの解放を中止しました"
                arm_outcome.torque = bus.read_torque_enable(ARM_IDS)
                return outcomes
            try:
                bus.disable_torque(ARM_IDS)
                arm_outcome.torque = bus.read_torque_enable(ARM_IDS)
            except Exception as exc:  # noqa: BLE001
                arm_outcome.error = f"{type(exc).__name__}: {exc}"
        return outcomes
    finally:
        bus.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="異常終了後にホイールとアームを解放する (ROS を使わない)",
    )
    parser.add_argument(
        "--bus-mode",
        choices=("split", "shared"),
        required=True,
        help="REQUIRED: split は2ポート、shared は canonical ID 1..9 の1ポート",
    )
    parser.add_argument(
        "--only",
        choices=("both", "wheels", "arm"),
        default="both",
        help="既定は both。ホイールだけ止めたいなら wheels (アームは落ちない)",
    )
    parser.add_argument("--wheel-port", default=WHEEL_PORT)
    parser.add_argument("--arm-port", default=ARM_PORT)
    parser.add_argument("--shared-port", default=SHARED_PORT)
    parser.add_argument(
        "--yes", action="store_true", help="アームが落ちる確認をスキップする"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Torque_Enable を読むだけで**一切書き込まない**。"
        "解放の前後で状態を比べるのに使う (アームは落ちない)",
    )
    args = parser.parse_args()

    do_wheels = args.only in ("both", "wheels")
    do_arm = args.only in ("both", "arm")

    outcomes: list[Outcome] = []
    if args.bus_mode == "shared":
        outcomes = release_shared_bus(
            args.shared_port,
            do_wheels=do_wheels,
            do_arm=do_arm,
            read_only=args.dry_run,
            assume_yes=args.yes,
        )
    else:
        # ★ 順序が重要。both では確認入力より先にホイールを停止する。
        if do_wheels:
            outcomes.append(
                release_bus(
                    "ホイール", args.wheel_port, WHEEL_IDS,
                    zero_velocity=True, read_only=args.dry_run,
                )
            )
        if do_arm:
            if not args.dry_run and not confirm(args.yes):
                sys.exit(1)
            outcomes.append(
                release_bus(
                    "アーム", args.arm_port, ARM_IDS,
                    zero_velocity=False, read_only=args.dry_run,
                )
            )

    print()
    for outcome in outcomes:
        outcome.report()
    print()

    # ★ 片方のポートが無くてももう片方は処理する。両方が揃わないと失敗、では
    #   「アームだけ繋がった机上の切り分け」ができない。
    failed = [o for o in outcomes if not o.released]
    if args.dry_run:
        # ★ 終了コードの意味は通常時と同じ「全 ID の 0 を読めたか」。
        #   --dry-run では **exit 1 が異常とは限らない**（トルクが入っているのを
        #   確認しに来た、が普通の使い方）。
        #   ★「トルクが入っている」と「読めなかった」は別物なので混ぜない。
        #     前者は解放すれば直り、後者は配線・電源・ポートの問題。
        engaged = [o for o in failed if any(v not in (None, 0) for v in o.torque.values())]
        unknown = [o for o in failed if o not in engaged]
        if engaged:
            print("トルクが入っています: " + ", ".join(o.label for o in engaged))
            print("  解放するには --dry-run を外して実行する。")
        if unknown:
            print("★ 状態を読めませんでした: " + ", ".join(o.label for o in unknown))
            print("  ポート・電源・配線を確認すること。解放できたかは判断できない。")
        if failed:
            sys.exit(1)
        print("すべて Torque_Enable=0（脱力済み）。何も書き込んでいません。")
        return
    if failed:
        print("★ 解放を確認できなかったものがあります: "
              + ", ".join(o.label for o in failed))
        sys.exit(1)
    print("すべて解放を確認しました (Torque_Enable=0 を読み戻し済み)")


if __name__ == "__main__":
    main()

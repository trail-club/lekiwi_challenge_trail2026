"""release_all の判定ロジック。実機もシリアルポートも要らない部分だけを検査する。

いちばん守りたいのは `Outcome.released`。`StsBus.disable_torque()` は ID ごとの
失敗を握り潰すので、「呼べた」を成功の根拠にすると**嘘の成功報告**になる。
読み戻して 0 だった ID だけを成功として扱うこと。
"""

import subprocess
import sys
import time
import types

import pytest

# scservo_sdk は pip でしか入らず (rosdep キーが無い)、Mac のホスト環境には無い。
# release_all 自体はシリアルを触らないので、import を通すためだけに差し込む。
sys.modules.setdefault("scservo_sdk", types.ModuleType("scservo_sdk"))

from lekiwi_so101_bringup.release_all import (  # noqa: E402
    ARM_IDS,
    WHEEL_IDS,
    Outcome,
    diagnose_port,
    port_holder,
    release_shared_bus,
)


def _outcome(torque):
    outcome = Outcome("テスト", "/dev/null", list(torque))
    outcome.torque = dict(torque)
    return outcome


def test_released_only_when_every_id_reads_zero():
    assert _outcome({1: 0, 2: 0, 3: 0}).released


def test_not_released_when_any_id_still_has_torque():
    assert not _outcome({1: 0, 2: 1, 3: 0}).released


def test_unreadable_id_is_not_success():
    """★ 読めなかった ID (None) を成功扱いにしないこと。

    disable_torque() は失敗を握り潰すので、None を「たぶん切れた」と解釈すると
    バスが死んでいるのに「すべて解放を確認しました」と出てしまう。
    """
    assert not _outcome({1: 0, 2: None, 3: 0}).released


def test_not_released_when_nothing_was_read():
    outcome = Outcome("テスト", "/dev/null", [1])
    assert outcome.torque == {}
    assert not outcome.released


def test_not_released_when_an_id_is_missing_from_the_readback():
    """ID の一部しか読めていないなら成功にしない。"""
    outcome = Outcome("テスト", "/dev/null", [1, 2, 3])
    outcome.torque = {1: 0, 2: 0}
    assert not outcome.released


def test_readback_wins_over_a_communication_error():
    """★ 判定の根拠は読み戻しだけ。

    途中で通信が切れて error が付いても、全 ID が 0 だと読めているなら
    ハードウェアは実際に解放されている。失敗扱いにすると、解放済みの機体へ
    人がもう一度アームを落としに行くことになる。
    """
    outcome = _outcome({1: 0, 2: 0})
    outcome.error = "SerialException: 途中で切れた"
    assert outcome.released


def test_connect_failure_is_not_released():
    """接続できなければ読み戻しが空なので必ず失敗。"""
    outcome = Outcome("テスト", "/dev/null", [1])
    outcome.error = "ポートが無い"
    assert not outcome.released


def test_report_shows_both_the_error_and_the_readback(capsys):
    outcome = _outcome({1: 0, 2: 1})
    outcome.error = "途中で切れた"
    outcome.report()
    out = capsys.readouterr().out
    assert "途中で切れた" in out
    assert "ID 1" in out and "ID 2" in out


def test_missing_port_is_diagnosed():
    reason = diagnose_port("/dev/definitely-not-a-real-port")
    assert reason is not None
    assert "存在しない" in reason


def test_existing_writable_port_passes(tmp_path):
    """★ /dev/null は使えない。コンテナ内では多数のプロセスが開いており、
    port_holder が正しく「掴まれている」と判定するため。"""
    port = tmp_path / "writable"
    port.touch()
    assert diagnose_port(str(port)) is None


def test_id_sets_match_the_hardware():
    """ホイールは 7/8/9、アームは 1〜6。混ぜるとバスが違うので必ず失敗する。"""
    assert WHEEL_IDS == [7, 8, 9]
    assert ARM_IDS == [1, 2, 3, 4, 5, 6]
    assert not set(WHEEL_IDS) & set(ARM_IDS)


@pytest.mark.parametrize("value", [0, 1])
def test_report_does_not_raise(capsys, value):
    _outcome({1: value, 2: None}).report()
    assert "ID 1" in capsys.readouterr().out


def test_report_says_whether_it_wrote_anything(capsys):
    """--dry-run と実解放を取り違えないこと。

    実機では「読んだだけ」と「トルクを切った」を混同すると、切れていない
    アームを切れたと思って手を放すことになる。見出しで区別する。
    """
    outcome = _outcome({1: 0})
    outcome.read_only = True
    outcome.report()
    assert "読み出しのみ" in capsys.readouterr().out

    outcome.read_only = False
    outcome.report()
    assert "解放" in capsys.readouterr().out


def test_dry_run_distinguishes_engaged_from_unreadable():
    """「トルクが入っている」と「読めなかった」は別物。

    前者は解放すれば直る。後者は配線・電源・ポートの問題で、解放できたのか
    どうかすら判断できない。まとめて「失敗」にすると原因を取り違える。
    """
    engaged = _outcome({1: 1, 2: 0})
    unreadable = _outcome({1: None, 2: None})
    released = _outcome({1: 0, 2: 0})

    def classify(outcome):
        # main() の --dry-run 分岐と同じ判定。
        return any(v not in (None, 0) for v in outcome.torque.values())

    assert not engaged.released and classify(engaged)
    assert not unreadable.released and not classify(unreadable)
    assert released.released


# ── ポートを掴んでいるプロセスの検出 ────────────────────────────────
#
# ★ ここが「launch だけ落ちてコンテナは生きている」場合に効く。
#   以前は Makefile が「コンテナが動いていたら中止」としており、
#   **いちばんありふれた故障で復帰手段が使えなかった**。


def test_no_holder_when_nobody_has_it_open(tmp_path):
    port = tmp_path / "free_port"
    port.touch()
    assert port_holder(str(port)) is None


def test_open_file_is_detected(tmp_path):
    """★ 別プロセスが掴んでいる場合を検出すること。

    自分自身の fd は除外する仕様なので、テストも別プロセスで掴ませる
    （実際に防ぎたいのは「ROS のノードが掴んでいる」状況）。
    """
    port = tmp_path / "busy_port"
    port.touch()
    with open(port) as handle:
        proc = subprocess.Popen(["sleep", "30"], stdin=handle)
        try:
            time.sleep(0.5)
            holder = port_holder(str(port))
        finally:
            proc.kill()
            proc.wait()
    assert holder is not None, "別プロセスが開いているのに検出できていない"
    assert holder.startswith("PID ")
    assert str(proc.pid) in holder


def test_not_detected_after_close(tmp_path):
    port = tmp_path / "reopened_port"
    port.touch()
    open(port).close()
    assert port_holder(str(port)) is None


def test_held_port_is_refused_by_diagnose_port(tmp_path):
    """★ これが本体。混線でアームが落ちるのを防ぐ。"""
    port = tmp_path / "held_port"
    port.touch()
    with open(port) as handle:
        proc = subprocess.Popen(["sleep", "30"], stdin=handle)
        try:
            time.sleep(0.5)
            reason = diagnose_port(str(port))
        finally:
            proc.kill()
            proc.wait()
    assert reason is not None
    assert "が開いています" in reason
    # ★「コンテナを落とせ」と言わないこと。落とす必要は無い。
    assert "コンテナは落とさなくて構いません" in reason


def test_free_port_passes_diagnose_port(tmp_path):
    port = tmp_path / "open_port"
    port.touch()
    assert diagnose_port(str(port)) is None


def test_shared_release_opens_once_and_stops_wheels_before_arm_confirmation(
    monkeypatch,
):
    events = []

    class FakeBus:
        def __init__(self, port, ids, baudrate):
            events.append(("init", port, tuple(ids), baudrate))

        def connect(self):
            events.append(("connect",))

        def stop(self, ids):
            events.append(("stop", tuple(ids)))

        def disable_torque(self, ids):
            events.append(("disable", tuple(ids)))

        def read_torque_enable(self, ids):
            events.append(("read", tuple(ids)))
            return dict.fromkeys(ids, 0)

        def close(self):
            events.append(("close",))

    monkeypatch.setattr("lekiwi_so101_bringup.release_all.diagnose_port", lambda _p: None)
    monkeypatch.setattr("lekiwi_so101_bringup.release_all.StsBus", FakeBus)

    def fake_confirm(_yes):
        events.append(("confirm",))
        return True

    monkeypatch.setattr("lekiwi_so101_bringup.release_all.confirm", fake_confirm)
    outcomes = release_shared_bus(
        "/dev/lekiwi",
        do_wheels=True,
        do_arm=True,
        read_only=False,
        assume_yes=False,
    )
    assert len([event for event in events if event[0] == "init"]) == 1
    assert events.index(("stop", tuple(WHEEL_IDS))) < events.index(("confirm",))
    assert ("disable", tuple(ARM_IDS)) in events
    assert all(outcome.released for outcome in outcomes)


def test_shared_dry_run_never_writes(monkeypatch):
    events = []

    class FakeBus:
        def __init__(self, *_args, **_kwargs):
            pass

        def connect(self):
            events.append("connect")

        def read_torque_enable(self, ids):
            events.append(("read", tuple(ids)))
            return dict.fromkeys(ids, 0)

        def close(self):
            events.append("close")

        def stop(self, _ids):
            events.append("WRITE_STOP")

        def disable_torque(self, _ids):
            events.append("WRITE_TORQUE")

    monkeypatch.setattr("lekiwi_so101_bringup.release_all.diagnose_port", lambda _p: None)
    monkeypatch.setattr("lekiwi_so101_bringup.release_all.StsBus", FakeBus)
    release_shared_bus(
        "/dev/lekiwi", do_wheels=True, do_arm=True, read_only=True, assume_yes=True
    )
    assert "WRITE_STOP" not in events
    assert "WRITE_TORQUE" not in events

"""torque:=false のときバックエンドが何をするかを固定する。

★ 検証の主眼は「**切るのと書かないのが対になっている**」こと。
  トルクだけ切って Goal_Position を 50Hz で書き続けると、何かの拍子に
  トルクが戻った瞬間に最後の指令値へアームが飛ぶ。片方だけ実装しても
  テストが通ってしまわないよう、両方をここで押さえる。

LeRobot も serial も import せずに走る (連絡先は偽物の bus/robot)。
"""

from __future__ import annotations

import json

import pytest
from so101_bringup.lerobot_backend import (
    LEROBOT_JOINTS,
    LeRobotSO101Backend,
    MockSO101Backend,
)


def _write_calibration(tmp_path, robot_id="my_follower"):
    data = {
        name: {
            "id": index,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": 0,
            "range_max": 4095,
        }
        for index, name in enumerate(LEROBOT_JOINTS, start=1)
    }
    path = tmp_path / f"{robot_id}.json"
    path.write_text(json.dumps(data))
    return path


class _FakeBus:
    def __init__(self):
        self.calls: list[str] = []

    def connect(self):
        self.calls.append("connect")

    def disable_torque(self):
        self.calls.append("disable_torque")

    def sync_read(self, name):
        self.calls.append(f"sync_read:{name}")
        return dict.fromkeys(LEROBOT_JOINTS, 0)

    def sync_write(self, name, values):
        self.calls.append(f"sync_write:{name}")


class _FakeRobot:
    is_calibrated = True

    def __init__(self):
        self.bus = _FakeBus()
        self.actions: list[dict] = []
        self.is_connected = True

    def configure(self):
        self.bus.calls.append("configure")

    def send_action(self, action):
        self.actions.append(action)

    def get_observation(self):
        return {f"{name}.pos": 0.0 for name in LEROBOT_JOINTS}


def _backend(tmp_path, torque):
    _write_calibration(tmp_path)
    backend = LeRobotSO101Backend(
        port="/dev/null",
        robot_id="my_follower",
        calibration_dir=str(tmp_path),
        torque=torque,
    )
    robot = _FakeRobot()
    backend._robot = robot
    return backend, robot


# ── 実機バックエンド ──────────────────────────────────────────────────


def test_トルクありが既定():
    assert LeRobotSO101Backend.__init__.__defaults__[-1] is True


@pytest.mark.parametrize("torque", [True, False])
def test_読み出しはトルクに依存しない(tmp_path, torque):
    backend, _ = _backend(tmp_path, torque)
    # /joint_states を出し続けられることの担保。
    assert set(backend.read_positions()) == set(LEROBOT_JOINTS)


def test_トルクなしでは指令を書かない(tmp_path):
    backend, robot = _backend(tmp_path, torque=False)
    backend.write_positions(dict.fromkeys(LEROBOT_JOINTS, 1.0))
    assert robot.actions == []


def test_トルクありなら指令を書く(tmp_path):
    backend, robot = _backend(tmp_path, torque=True)
    backend.write_positions(dict.fromkeys(LEROBOT_JOINTS, 1.0))
    assert len(robot.actions) == 1


def test_壊れた指令はトルクの有無に関わらず弾く(tmp_path):
    # torque=False を「何をしても素通り」にしない。torque=True へ戻したときに
    # 挙動が変わらないようにするため。
    for torque in (True, False):
        backend, _ = _backend(tmp_path, torque)
        with pytest.raises(ValueError):
            backend.write_positions({"shoulder_pan": 0.0})


def _fake_lerobot_module(robot, monkeypatch):
    """`from lerobot.robots.so_follower import ...` を偽物に差し替える。"""
    import sys
    import types

    module = types.ModuleType("lerobot.robots.so_follower")
    module.SO101Follower = lambda config: robot
    module.SO101FollowerConfig = lambda **kwargs: None
    for name in ("lerobot", "lerobot.robots", "lerobot.robots.so_follower"):
        monkeypatch.setitem(sys.modules, name, module)


@pytest.mark.parametrize(
    "torque,expect_last",
    [(False, "disable_torque"), (True, "configure")],
)
def test_トルクなしでは_configure_のあとに切る(tmp_path, monkeypatch, torque, expect_last):
    backend, robot = _backend(tmp_path, torque)
    _fake_lerobot_module(robot, monkeypatch)
    backend._robot = None
    backend.connect()

    # ★ configure() がトルクを入れ直すので、切るならその**後**でなければ意味がない。
    assert robot.bus.calls[-1] == expect_last


# ── モックバックエンド ────────────────────────────────────────────────


def test_モックもトルクなしでは姿勢が変わらない():
    backend = MockSO101Backend(torque=False)
    backend.connect()
    before = backend.read_positions()
    backend.write_positions(dict.fromkeys(LEROBOT_JOINTS, 1.0))
    assert backend.read_positions() == before


def test_モックはトルクありなら追従する():
    backend = MockSO101Backend()
    backend.connect()
    backend.write_positions(dict.fromkeys(LEROBOT_JOINTS, 1.0))
    assert backend.read_positions()["shoulder_pan"] == pytest.approx(1.0)

import json
import sys
import types

import pytest

from so101_bringup.lerobot_backend import (
    LEROBOT_JOINTS,
    LEKIWI_ARM_NAMES,
    LEKIWI_NAMES,
    LEKIWI_WHEEL_NAMES,
    LeRobotLeKiwiBackend,
)


def _calibration():
    return {
        name: {
            "id": motor_id,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": 0,
            "range_max": 4095,
        }
        for motor_id, name in enumerate(LEKIWI_NAMES, start=1)
    }


def _write_calibration(tmp_path, data=None):
    path = tmp_path / "robot.json"
    path.write_text(json.dumps(_calibration() if data is None else data))
    return path


def test_nine_motor_calibration_accepts_canonical_ids(tmp_path):
    _write_calibration(tmp_path)
    backend = LeRobotLeKiwiBackend("/dev/null", "robot", str(tmp_path))
    assert backend.calibration_file.name == "robot.json"


@pytest.mark.parametrize(
    "mutation", ["missing", "wrong_id", "duplicate_id", "non_integer_id"]
)
def test_nine_motor_calibration_rejects_noncanonical_layout(tmp_path, mutation):
    data = _calibration()
    if mutation == "missing":
        data.pop("base_right_wheel")
    elif mutation == "wrong_id":
        data["base_left_wheel"]["id"] = 8
    elif mutation == "duplicate_id":
        data["base_back_wheel"]["id"] = 7
    else:
        data["base_left_wheel"]["id"] = 7.0
    _write_calibration(tmp_path, data)
    with pytest.raises(ValueError):
        LeRobotLeKiwiBackend("/dev/null", "robot", str(tmp_path))


class _FakeBus:
    def __init__(self, events):
        self.events = events
        self.is_connected = False
        self.port_handler = types.SimpleNamespace(closePort=lambda: None)

    def connect(self):
        self.events.append(("connect", tuple(LEKIWI_NAMES)))
        self.is_connected = True

    def disable_torque(self, names=None):
        self.events.append(("disable", tuple(names) if names else tuple(LEKIWI_NAMES)))

    def enable_torque(self, names=None):
        self.events.append(("enable", tuple(names) if names else tuple(LEKIWI_NAMES)))

    def sync_read(self, register, names):
        self.events.append(("read", register, tuple(names)))
        return dict.fromkeys(names, 12.0)

    def sync_write(self, register, values, **kwargs):
        self.events.append(("write", register, tuple(values), dict(kwargs)))

    def configure_motors(self):
        self.events.append(("configure",))

    def write(self, register, name, value):
        self.events.append(("write_one", register, name, value))

    def disconnect(self, disable_torque=True):
        self.events.append(("disconnect", disable_torque))
        self.is_connected = False


def _install_fake_lerobot(monkeypatch, events, *, calibrated=True):
    class FakeConfig:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    class FakeRobot:
        def __init__(self, _config):
            self.bus = _FakeBus(events)
            self.is_calibrated = calibrated

        @property
        def is_connected(self):
            return self.bus.is_connected

    root = types.ModuleType("lerobot")
    roots = types.ModuleType("lerobot.robots")
    module = types.ModuleType("lerobot.robots.lekiwi")
    module.LeKiwi = FakeRobot
    module.LeKiwiConfig = FakeConfig
    motors = types.ModuleType("lerobot.motors")
    feetech = types.ModuleType("lerobot.motors.feetech")
    feetech.OperatingMode = types.SimpleNamespace(
        POSITION=types.SimpleNamespace(value=0),
        VELOCITY=types.SimpleNamespace(value=1),
    )
    for name, value in {
        "lerobot": root,
        "lerobot.robots": roots,
        "lerobot.robots.lekiwi": module,
        "lerobot.motors": motors,
        "lerobot.motors.feetech": feetech,
    }.items():
        monkeypatch.setitem(sys.modules, name, value)


def test_connect_latches_arm_and_zeros_wheels_before_modes_and_torque(
    tmp_path, monkeypatch
):
    _write_calibration(tmp_path)
    events = []
    _install_fake_lerobot(monkeypatch, events)
    backend = LeRobotLeKiwiBackend("/dev/lekiwi", "robot", str(tmp_path))
    backend.connect()

    latch = next(i for i, event in enumerate(events) if event[:2] == ("write", "Goal_Position"))
    zero = next(i for i, event in enumerate(events) if event[:2] == ("write", "Goal_Velocity"))
    configure = events.index(("configure",))
    enable = next(i for i, event in enumerate(events) if event[0] == "enable")
    assert events[0][0] == "connect"
    assert events[1][0] == "disable"
    assert latch < zero < configure < enable
    assert events[zero][2] == LEKIWI_WHEEL_NAMES
    assert set(events[latch][2]) == set(LEKIWI_ARM_NAMES)


def test_arm_torque_false_enables_only_wheels_and_discards_arm_commands(
    tmp_path, monkeypatch
):
    _write_calibration(tmp_path)
    events = []
    _install_fake_lerobot(monkeypatch, events)
    backend = LeRobotLeKiwiBackend(
        "/dev/lekiwi", "robot", str(tmp_path), torque=False
    )
    backend.connect()

    assert ("enable", LEKIWI_WHEEL_NAMES) in events
    assert ("enable", LEKIWI_NAMES) not in events

    wheel_enable = events.index(("enable", LEKIWI_WHEEL_NAMES))
    arm_disable = events.index(("disable", LEKIWI_ARM_NAMES))
    assert wheel_enable < arm_disable

    events.clear()
    backend.write_positions(dict.fromkeys(LEROBOT_JOINTS, 1.0))
    assert not any(event[:2] == ("write", "Goal_Position") for event in events)


def test_eeprom_mismatch_fails_before_torque_enable(tmp_path, monkeypatch):
    _write_calibration(tmp_path)
    events = []
    _install_fake_lerobot(monkeypatch, events, calibrated=False)
    backend = LeRobotLeKiwiBackend("/dev/lekiwi", "robot", str(tmp_path))
    with pytest.raises(RuntimeError, match="EEPROM"):
        backend.connect()
    assert not any(event[0] == "enable" for event in events)


def test_wheel_recover_only_reconfigures_ids_seven_to_nine(tmp_path, monkeypatch):
    _write_calibration(tmp_path)
    events = []
    _install_fake_lerobot(monkeypatch, events)
    backend = LeRobotLeKiwiBackend("/dev/lekiwi", "robot", str(tmp_path))
    backend.connect()
    events.clear()
    backend.recover_wheels()
    assert events[0][:2] == ("write", "Goal_Velocity")
    assert events[1] == ("disable", LEKIWI_WHEEL_NAMES)
    assert events[-1] == ("enable", LEKIWI_WHEEL_NAMES)
    configured_names = {
        value
        for event in events
        for value in event
        if isinstance(value, str) and value.startswith("arm_")
    }
    assert not configured_names


def test_disconnect_zeros_wheels_before_all_torque_off(tmp_path, monkeypatch):
    _write_calibration(tmp_path)
    events = []
    _install_fake_lerobot(monkeypatch, events)
    backend = LeRobotLeKiwiBackend("/dev/lekiwi", "robot", str(tmp_path))
    backend.connect()
    events.clear()
    backend.disconnect()
    assert events[0][:3] == (
        "write",
        "Goal_Velocity",
        LEKIWI_WHEEL_NAMES,
    )
    assert events[1] == ("disconnect", True)

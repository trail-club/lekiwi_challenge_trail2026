from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]


def test_compose_motor_mounts_are_mode_specific():
    common = (ROOT / "docker/robot/compose.yaml").read_text()
    split = (ROOT / "docker/robot/compose.split.yaml").read_text()
    shared = (ROOT / "docker/robot/compose.shared.yaml").read_text()
    assert "SO101_DEVICE" not in common and "LEKIWI_DEVICE" not in common
    assert "SO101_DEVICE" in split and "LEKIWI_DEVICE" in split
    assert "SO101_DEVICE" not in shared and shared.count("LEKIWI_DEVICE") == 2


def test_makefile_has_only_explicit_run_commands():
    makefile = (ROOT / "docker/robot/Makefile").read_text()
    assert "\nrun:" not in makefile
    for target in ("run-split:", "run-shared:", "mock-split:", "mock-shared:"):
        assert target in makefile
    assert "require-bus-mode" in makefile


def test_combined_launch_requires_mode_and_selects_bridge_backend():
    launch = (
        ROOT
        / "ros2_ws/src/lekiwi_so101_bringup/launch/robot.launch.py"
    ).read_text()
    assert 'DeclareLaunchArgument(\n            "motor_bus_mode",' in launch
    assert "motor_bus_mode must be explicitly set" in launch
    assert "'bridge' if '" in launch
    assert '("hardware_backend", hardware_backend)' in launch


def test_internal_message_has_fixed_three_tick_order():
    message = (
        ROOT
        / "ros2_ws/src/lekiwi_hardware_interfaces/msg/WheelCommand.msg"
    ).read_text()
    assert message.splitlines() == ["std_msgs/Header header", "int32[3] ticks"]

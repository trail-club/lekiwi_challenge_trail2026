from pathlib import Path


def test_bridge_backend_does_not_connect_sts_bus():
    source = (
        Path(__file__).resolve().parents[1]
        / "lekiwi_base_bringup"
        / "base_driver.py"
    ).read_text()
    assert 'self.hardware_backend == "bridge"' in source
    assert '"/lekiwi/hardware_wheel_commands"' in source
    assert "bridge_ready" in source
    assert "integration_dt = dt if bridge_ready else 0.0" in source

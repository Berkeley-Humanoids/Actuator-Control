from __future__ import annotations

from pathlib import Path

import pytest

from actuator_control import Motor, RobstrideBus


@pytest.mark.skipif(
    not Path("/sys/class/net/vcan0").exists(),
    reason="requires a configured vcan0 interface",
)
def test_native_socketcan_smoke_on_vcan() -> None:
    bus = RobstrideBus(channel="vcan0", motors={"joint": Motor(id=1, model="rs-02")})
    bus.connect(handshake=False)
    try:
        assert bus.is_connected is True
    finally:
        bus.disconnect(disable_torque=False)

from actuator_control import (
    CalibrationEntry,
    ERobBus,
    Motor,
    RobstrideBus,
    RobstrideCommunicationType,
    RobstrideParameterType,
    SitoBus,
)


def test_motor_and_calibration_entry_are_python_dataclasses() -> None:
    motor = Motor(id=11, model="rs-02")
    calibration = CalibrationEntry(direction=-1, homing_offset=0.25)

    assert motor.id == 11
    assert motor.model == "rs-02"
    assert calibration.direction == -1
    assert calibration.homing_offset == 0.25


def test_wrapper_properties_and_calibration_normalization() -> None:
    bus = RobstrideBus(
        channel="can0",
        motors={"joint": Motor(id=11, model="rs-02")},
        calibration={"joint": {"direction": -1, "homing_offset": 0.25}},
    )

    assert len(bus) == 1
    assert bus.channel == "can0"
    assert bus.bitrate == 1_000_000
    assert bus.models == ["rs-02"]
    assert bus.ids == [11]
    assert isinstance(bus.calibration["joint"], CalibrationEntry)
    assert "RobstrideBus" in repr(bus)


def test_context_manager_uses_python_methods() -> None:
    bus = ERobBus(channel="can0", motors={"joint": Motor(id=15, model="eRob70")})
    calls: list[tuple[str, bool]] = []

    bus.connect = lambda handshake=True: calls.append(("connect", handshake))
    bus.disconnect = lambda disable_torque=True: calls.append(("disconnect", disable_torque))

    with bus as same_bus:
        assert same_bus is bus

    assert calls == [("connect", True), ("disconnect", True)]


def test_robstride_legacy_operation_frame_api_is_not_exposed() -> None:
    bus = RobstrideBus(channel="can0", motors={"joint": Motor(id=11, model="rs-02")})

    assert not hasattr(bus, "write_operation_frame")
    assert not hasattr(bus, "read_operation_frame")


def test_top_level_robstride_exports_remain_available() -> None:
    assert RobstrideCommunicationType.ENABLE == 3
    assert RobstrideCommunicationType.DISABLE == 4
    assert RobstrideParameterType.MECHANICAL_OFFSET.param_id == 0x2005
    assert RobstrideParameterType.ZERO_STATE.param_id == 0x7029


def test_sito_repr_includes_control_frequency() -> None:
    bus = SitoBus(
        channel="can0",
        motors={"joint": Motor(id=0x16, model="TA40-50")},
        control_frequency=100.0,
    )

    assert "control_frequency=100.0" in repr(bus)

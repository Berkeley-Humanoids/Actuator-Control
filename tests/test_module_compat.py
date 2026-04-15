from actuator_control.robstride import (
    CommunicationType,
    ParameterType,
    RobstrideBus,
    RobstrideParameter,
)
from actuator_control.robstride_protocol import (
    CommunicationType as ProtocolCommunicationType,
    ParameterType as ProtocolParameterType,
)
from actuator_control.robstride_table import (
    MODEL_MIT_KD_TABLE,
    MODEL_MIT_KP_TABLE,
    MODEL_MIT_POSITION_TABLE,
    MODEL_MIT_TORQUE_TABLE,
    MODEL_MIT_VELOCITY_TABLE,
)


def test_compatibility_modules_export_expected_protocol_objects() -> None:
    assert CommunicationType.ENABLE == ProtocolCommunicationType.ENABLE == 3
    assert ParameterType.ZERO_STATE.param_id == ProtocolParameterType.ZERO_STATE.param_id == 0x7029
    assert isinstance(ParameterType.MECHANICAL_OFFSET, RobstrideParameter)


def test_robstride_tables_remain_importable() -> None:
    assert MODEL_MIT_POSITION_TABLE["rs-02"] > 12.0
    assert MODEL_MIT_VELOCITY_TABLE["rs-04"] == 15
    assert MODEL_MIT_TORQUE_TABLE["rs-06"] == 60
    assert MODEL_MIT_KP_TABLE["rs-03"] == 5000.0
    assert MODEL_MIT_KD_TABLE["rs-05"] == 5.0


def test_robstride_wrapper_keeps_low_level_management_methods() -> None:
    bus = RobstrideBus(channel="can0", motors={})

    assert hasattr(bus, "scan_channel")
    assert hasattr(bus, "read_fault_status")
    assert hasattr(bus, "clear_fault")

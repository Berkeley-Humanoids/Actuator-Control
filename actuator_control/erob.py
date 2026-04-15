"""Python-friendly wrapper for the native eRob actuator backend."""

from __future__ import annotations

import math

from . import _actuator_control as _native
from .common import ActuatorBus, Value


class MotorConfig:
    """Shared scaling constants for the eRob position controller."""

    counts_per_rad: float = 524287 / (2 * math.pi)


class CommandCode:
    """Command identifiers used by the eRob CAN protocol."""

    START_MOTION = 0x83
    STOP_MOTION = 0x84
    SAVE_PARAMS = 0xE8


class Parameter:
    """Register identifiers used by the eRob CAN protocol."""

    ACTUAL_POSITION = 0x02
    ACTUAL_SPEED = 0x05
    MOTOR_CURRENT = 0x08
    ERROR_CODE = 0x1F
    RUN_STATUS = 0x20
    POWER_TEMP = 0x26
    CONTROL_MODE = 0x4E
    MAX_POSITION_ERROR = 0x54
    MAX_SPEED = 0x55
    TARGET_POSITION = 0x86
    RELATIVE_POSITION = 0x87
    ACCELERATION = 0x88
    DECELERATION = 0x89
    TARGET_SPEED = 0x8A
    MOTION_MODE = 0x8D
    PID_ADJUSTMENT = 0x0124
    POSITION_LOOP_GAIN = 0x64
    SPEED_LOOP_GAIN = 0x66
    SPEED_LOOP_INTEGRAL = 0x67


class Mode:
    """Control modes accepted by eRob actuators."""

    TORQUE = 1
    SPEED = 2
    POSITION = 3


class ERobBus(ActuatorBus):
    """CAN backend for eRob actuators using MIT-like position control."""

    _native_cls = _native.ERobBus

    def receive(self, device_id: int, timeout: float = 0.1) -> bytes | None:
        result = self._native.receive(device_id=device_id, timeout=timeout)
        return None if result is None else bytes(result)

    def transmit(self, device_id: int, data: bytes = b"\x00\x00\x00\x00\x00\x00\x00\x00") -> None:
        self._native.transmit(device_id=device_id, data=data)

    def send_command(self, motor: str, command: int) -> None:
        self._native.send_command(motor, command)

    def read(self, motor: str, parameter: int, extra_bytes: bytes = b"") -> Value | None:
        return self._native.read(motor, parameter, extra_bytes)

    def write(self, motor: str, parameter: int, value: int) -> None:
        self._native.write(motor, parameter, int(value))

    def write_subindex(self, motor: str, parameter: int, subindex: int, value: int) -> None:
        self._native.write_subindex(motor, parameter, subindex, int(value))


__all__ = [
    "CommandCode",
    "ERobBus",
    "Mode",
    "MotorConfig",
    "Parameter",
]

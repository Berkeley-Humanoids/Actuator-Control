"""Python-friendly wrapper for the native Robstride actuator backend."""

from __future__ import annotations

from . import _actuator_control as _native
from .common import ActuatorBus, Value


RobstrideParameter = _native.RobstrideParameter
RobstrideCommunicationType = _native.robstride_comm
RobstrideParameterType = _native.robstride_param

CommunicationType = RobstrideCommunicationType
ParameterType = RobstrideParameterType


class RobstrideBus(ActuatorBus):
    """CAN backend for Robstride actuators."""

    _native_cls = _native.RobstrideBus

    @classmethod
    def scan_channel(
        cls,
        channel: str,
        start_id: int = 1,
        end_id: int = 255,
        bitrate: int = 1_000_000,
    ) -> dict[int, tuple[int, bytes]]:
        raw = cls._native_cls.scan_channel(
            channel=channel,
            start_id=start_id,
            end_id=end_id,
            bitrate=bitrate,
        )
        return {
            int(device_id): (int(extra_data), bytes(payload))
            for device_id, (extra_data, payload) in raw.items()
        }

    @property
    def host_id(self) -> int:
        return int(self._native.host_id)

    @host_id.setter
    def host_id(self, value: int) -> None:
        self._native.host_id = int(value)

    @property
    def is_receiver_running(self) -> bool:
        return bool(self._native.is_receiver_running)

    def start_receiver(self) -> None:
        self._native.start_receiver()

    def stop_receiver(self) -> None:
        self._native.stop_receiver()

    def transmit(
        self,
        communication_type: int,
        extra_data: int,
        device_id: int,
        data: bytes = b"\x00\x00\x00\x00\x00\x00\x00\x00",
    ) -> None:
        self._native.transmit(
            communication_type=communication_type,
            extra_data=extra_data,
            device_id=device_id,
            data=data,
        )

    def receive(self, timeout: float | None = None) -> tuple[int, int, int, bytes] | None:
        result = self._native.receive(timeout)
        if result is None:
            return None
        communication_type, extra_data, device_id, data = result
        return int(communication_type), int(extra_data), int(device_id), bytes(data)

    def receive_status_frame(self, motor: str) -> tuple[float, float, float, float]:
        position, velocity, torque, temperature = self._native.receive_status_frame(motor)
        return float(position), float(velocity), float(torque), float(temperature)

    def receive_read_frame(self) -> bytes:
        return bytes(self._native.receive_read_frame())

    def ping_by_id(self, device_id: int, timeout: float | None = None) -> tuple[int, bytes] | None:
        result = self._native.ping_by_id(device_id, timeout)
        if result is None:
            return None
        extra_data, data = result
        return int(extra_data), bytes(data)

    def read_id(self, motor: str, timeout: float | None = None) -> tuple[int, bytes] | None:
        result = self._native.read_id(motor, timeout)
        if result is None:
            return None
        extra_data, data = result
        return int(extra_data), bytes(data)

    def write_id(self, motor: str, new_id: int) -> tuple[int, bytes] | None:
        result = self._native.write_id(motor, new_id)
        if result is None:
            return None
        self._motors[motor].id = int(new_id)
        extra_data, data = result
        return int(extra_data), bytes(data)

    def read(self, motor: str, parameter_type: RobstrideParameter) -> Value:
        value = self._native.read(motor, parameter_type)
        return float(value) if isinstance(value, float) else int(value)

    def write(self, motor: str, parameter_type: RobstrideParameter, value: Value) -> None:
        self._native.write(motor, parameter_type, value)

    def clear_fault(self, motor: str) -> None:
        self._native.clear_fault(motor)

    def disable_nowait(self, motor: str, clear_fault: bool = False) -> None:
        self._native.disable_nowait(motor, clear_fault=clear_fault)

    def disable_with_clear(self, motor: str) -> None:
        self._native.disable_with_clear(motor)

    def read_fault_status(
        self,
        motor: str | None = None,
    ) -> dict[str, list[str]] | list[str]:
        result = self._native.read_fault_status(motor)
        if motor is None:
            return {name: list(statuses) for name, statuses in result.items()}
        return list(result)

    def clear_fault_status(self, motor: str | None = None) -> None:
        self._native.clear_fault_status(motor)

    def transport_counters(self) -> dict[str, int]:
        return {name: int(value) for name, value in self._native.transport_counters().items()}


__all__ = [
    "CommunicationType",
    "ParameterType",
    "RobstrideBus",
    "RobstrideCommunicationType",
    "RobstrideParameter",
    "RobstrideParameterType",
]

"""Shared Python-facing wrappers and types for native actuator backends."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, TypeAlias

from . import _actuator_control as _native


Value: TypeAlias = int | float
Calibration: TypeAlias = dict[str, "CalibrationEntry"]


@dataclass(slots=True)
class Motor:
    """Metadata used to address a motor on a CAN bus."""

    id: int
    model: str


@dataclass(slots=True)
class CalibrationEntry:
    """Per-motor calibration used by MIT-style commands and state reads."""

    direction: int
    homing_offset: float


def _coerce_motor(value: Any) -> Motor:
    if isinstance(value, Motor):
        return value
    if isinstance(value, _native.Motor):
        return Motor(id=value.id, model=value.model)
    if hasattr(value, "id") and hasattr(value, "model"):
        return Motor(id=int(value.id), model=str(value.model))
    raise TypeError("Motor values must provide 'id' and 'model'.")


def _coerce_calibration_entry(value: Any) -> CalibrationEntry:
    if isinstance(value, CalibrationEntry):
        return value
    if isinstance(value, _native.CalibrationEntry):
        return CalibrationEntry(direction=value.direction, homing_offset=value.homing_offset)
    if isinstance(value, Mapping):
        if "direction" not in value or "homing_offset" not in value:
            raise TypeError(
                "Calibration dict entries must include 'direction' and 'homing_offset'."
            )
        return CalibrationEntry(
            direction=int(value["direction"]),
            homing_offset=float(value["homing_offset"]),
        )
    if hasattr(value, "direction") and hasattr(value, "homing_offset"):
        return CalibrationEntry(
            direction=int(value.direction),
            homing_offset=float(value.homing_offset),
        )
    raise TypeError(
        "Calibration values must be CalibrationEntry objects or mappings "
        "with 'direction' and 'homing_offset'."
    )


def _normalize_motors(motors: Mapping[str, Any]) -> dict[str, Motor]:
    normalized = {str(name): _coerce_motor(value) for name, value in motors.items()}
    motor_ids = [motor.id for motor in normalized.values()]
    if len(motor_ids) != len(set(motor_ids)):
        raise ValueError("Motor IDs must be unique within an actuator bus.")
    return normalized


def _normalize_calibration(calibration: Mapping[str, Any] | None) -> Calibration | None:
    if calibration is None:
        return None
    if not isinstance(calibration, Mapping):
        raise TypeError("calibration must be None or a dict[str, CalibrationEntry-like].")
    return {
        str(name): _coerce_calibration_entry(value)
        for name, value in calibration.items()
    }


def _to_native_motors(motors: Mapping[str, Motor]) -> dict[str, _native.Motor]:
    return {
        name: _native.Motor(motor.id, motor.model)
        for name, motor in motors.items()
    }


def _to_native_calibration(
    calibration: Mapping[str, CalibrationEntry] | None,
) -> dict[str, _native.CalibrationEntry] | None:
    if calibration is None:
        return None
    return {
        name: _native.CalibrationEntry(entry.direction, entry.homing_offset)
        for name, entry in calibration.items()
    }


class ActuatorBus:
    """Python-friendly base wrapper over the private native actuator bus."""

    _native_cls = _native.ActuatorBus

    def __init__(
        self,
        channel: str,
        motors: Mapping[str, Any],
        calibration: Mapping[str, Any] | None = None,
        bitrate: int = 1_000_000,
        **native_kwargs: Any,
    ) -> None:
        self.channel = channel
        self._motors = _normalize_motors(motors)
        self._calibration = _normalize_calibration(calibration)
        self.bitrate = bitrate
        self._native = self._build_native(native_kwargs)

    def _build_native(self, native_kwargs: dict[str, Any]) -> Any:
        if self._native_cls is _native.ActuatorBus:
            raise TypeError("ActuatorBus is an abstract base class and cannot be instantiated.")
        return self._native_cls(
            channel=self.channel,
            motors=_to_native_motors(self._motors),
            calibration=_to_native_calibration(self._calibration),
            bitrate=self.bitrate,
            **native_kwargs,
        )

    def __len__(self) -> int:
        return len(self._motors)

    def __repr__(self) -> str:
        return (
            f"{self.__class__.__name__}("
            f"channel={self.channel!r}, "
            f"motors={self._motors!r}, "
            f"bitrate={self.bitrate!r})"
        )

    def __enter__(self) -> "ActuatorBus":
        self.connect()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> bool:
        self.disconnect()
        return False

    @property
    def motors(self) -> dict[str, Motor]:
        return self._motors

    @property
    def calibration(self) -> Calibration | None:
        return self._calibration

    @property
    def models(self) -> list[str]:
        return [motor.model for motor in self._motors.values()]

    @property
    def ids(self) -> list[int]:
        return [motor.id for motor in self._motors.values()]

    @property
    def is_connected(self) -> bool:
        return bool(self._native.is_connected)

    def connect(self, handshake: bool = True) -> None:
        self._native.connect(handshake=handshake)

    def disconnect(self, disable_torque: bool = True) -> None:
        self._native.disconnect(disable_torque=disable_torque)

    def enable(self, motor: str) -> None:
        self._native.enable(motor)

    def disable(self, motor: str) -> None:
        self._native.disable(motor)

    def read(self, motor: str, parameter: Any) -> Value | None:
        raise NotImplementedError("This backend does not expose a generic read() method.")

    def write(self, motor: str, parameter: Any, value: Value) -> None:
        raise NotImplementedError("This backend does not expose a generic write() method.")

    def write_mit_kp_kd(self, motor: str, kp: float, kd: float) -> None:
        self._native.write_mit_kp_kd(motor, kp, kd)

    def write_mit_control(
        self,
        motor: str,
        position: float,
        velocity: float = 0.0,
        torque: float = 0.0,
    ) -> None:
        self._native.write_mit_control(
            motor=motor,
            position=position,
            velocity=velocity,
            torque=torque,
        )

    def read_mit_state(self, motor: str) -> tuple[float, float]:
        position, velocity = self._native.read_mit_state(motor)
        return float(position), float(velocity)


__all__ = [
    "ActuatorBus",
    "Calibration",
    "CalibrationEntry",
    "Motor",
    "Value",
]

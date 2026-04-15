"""Python-friendly wrapper for the native Sito actuator backend."""

from __future__ import annotations

from . import _actuator_control as _native
from .common import ActuatorBus


class CommunicationType:
    """Sito command and feedback identifiers."""

    RESET = 0x00
    SELECT_MODE = 0x01
    SET_MIT_CURRENT_VELOCITY_POSITION = 0x09
    SET_MIT_KP_KD = 0x45
    FEEDBACK_0 = 0xB0
    FEEDBACK_1 = 0xB1
    FEEDBACK_2 = 0xB2
    FEEDBACK_3 = 0xB3


class Mode:
    """Operating modes supported by the Sito protocol."""

    POSITION = 0x08
    MIT = 0x09


class SitoBus(ActuatorBus):
    """CAN backend for Sito TA40-series actuators."""

    _native_cls = _native.SitoBus

    def __init__(
        self,
        channel: str,
        motors: dict[str, object],
        calibration: dict[str, object] | None = None,
        bitrate: int = 1_000_000,
        control_frequency: float = 50.0,
    ) -> None:
        self.control_frequency = control_frequency
        super().__init__(
            channel=channel,
            motors=motors,
            calibration=calibration,
            bitrate=bitrate,
            control_frequency=control_frequency,
        )

    def __repr__(self) -> str:
        return (
            f"{self.__class__.__name__}("
            f"channel={self.channel!r}, "
            f"motors={self.motors!r}, "
            f"bitrate={self.bitrate!r}, "
            f"control_frequency={self.control_frequency!r})"
        )


__all__ = [
    "CommunicationType",
    "Mode",
    "SitoBus",
]

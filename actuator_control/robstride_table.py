"""MIT scaling tables for Robstride actuators."""

from __future__ import annotations

import math


MODEL_MIT_POSITION_TABLE = {
    "rs-00": 4 * math.pi,
    "rs-01": 4 * math.pi,
    "rs-02": 4 * math.pi,
    "rs-03": 4 * math.pi,
    "rs-04": 4 * math.pi,
    "rs-05": 4 * math.pi,
    "rs-06": 4 * math.pi,
}

MODEL_MIT_VELOCITY_TABLE = {
    "rs-00": 50,
    "rs-01": 44,
    "rs-02": 44,
    "rs-03": 50,
    "rs-04": 15,
    "rs-05": 33,
    "rs-06": 20,
}

MODEL_MIT_TORQUE_TABLE = {
    "rs-00": 17,
    "rs-01": 17,
    "rs-02": 17,
    "rs-03": 60,
    "rs-04": 120,
    "rs-05": 17,
    "rs-06": 60,
}

MODEL_MIT_KP_TABLE = {
    "rs-00": 500.0,
    "rs-01": 500.0,
    "rs-02": 500.0,
    "rs-03": 5000.0,
    "rs-04": 5000.0,
    "rs-05": 500.0,
    "rs-06": 5000.0,
}

MODEL_MIT_KD_TABLE = {
    "rs-00": 5.0,
    "rs-01": 5.0,
    "rs-02": 5.0,
    "rs-03": 100.0,
    "rs-04": 100.0,
    "rs-05": 5.0,
    "rs-06": 100.0,
}


__all__ = [
    "MODEL_MIT_KD_TABLE",
    "MODEL_MIT_KP_TABLE",
    "MODEL_MIT_POSITION_TABLE",
    "MODEL_MIT_TORQUE_TABLE",
    "MODEL_MIT_VELOCITY_TABLE",
]

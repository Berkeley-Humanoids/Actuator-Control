# Actuator Control

Native C++17 SocketCAN core with a thin Python wrapper for controlling eRob, Robstride, and Sito actuators.

## Installation

The package now builds a private `_actuator_control` extension with `scikit-build-core`, CMake, and `pybind11`.

```bash
uv sync
```

For the Python examples:

```bash
uv sync --extra examples
```

## Quick Start

```python
from actuator_control import ERobBus, Motor

motors = {
    "joint": Motor(id=15, model="eRob70"),
}

with ERobBus(channel="can0", motors=motors, bitrate=1_000_000) as bus:
    bus.enable("joint")
    bus.write_mit_kp_kd("joint", kp=10.0, kd=1.0)
    bus.write_mit_control("joint", position=0.25)
    position, velocity = bus.read_mit_state("joint")
    print(position, velocity)
```

## API Notes

- Top-level imports remain stable: `Motor`, `ActuatorBus`, `ERobBus`, `RobstrideBus`, `SitoBus`, `RobstrideCommunicationType`, and `RobstrideParameterType`.
- `Motor` and `CalibrationEntry` are regular Python dataclasses.
- The Python layer is intentionally thin: constructors, docstrings, context-manager support, and compatibility re-exports live in Python; transport and protocol logic live in C++.
- Robstride no longer exposes the legacy `write_operation_frame()` or `read_operation_frame()` methods. Use `write_mit_control()` and `read_mit_state()` instead.

## SocketCAN

The native core talks directly to Linux SocketCAN. Bring the interface up before connecting:

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

Or use [examples/canup.sh](/home/tk/Downloads/Actuator-Control/examples/canup.sh).

## Development

Run Python tests:

```bash
uv run pytest
```

Run native tests:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

#pragma once

#include <map>
#include <string>

namespace actuator_control {

struct Motor {
    int id;
    std::string model;
};

struct CalibrationEntry {
    int direction;
    double homing_offset;
};

struct MotorStatus {
    double position = 0.0;
    double velocity = 0.0;
    double torque = 0.0;
    double temperature = 0.0;
};

using MotorMap = std::map<std::string, Motor>;
using Calibration = std::map<std::string, CalibrationEntry>;

} // namespace actuator_control

#pragma once

#include <string>
#include <unordered_map>

namespace actuator_control {

const std::unordered_map<std::string, double>& mit_position_table();
const std::unordered_map<std::string, double>& mit_velocity_table();
const std::unordered_map<std::string, double>& mit_torque_table();
const std::unordered_map<std::string, double>& mit_kp_table();
const std::unordered_map<std::string, double>& mit_kd_table();

} // namespace actuator_control

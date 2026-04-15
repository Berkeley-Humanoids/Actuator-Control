#include "actuator_control/robstride_table.hpp"

#include <cmath>

namespace actuator_control {

const std::unordered_map<std::string, double>& mit_position_table() {
    static const std::unordered_map<std::string, double> table = {
        {"rs-00", 4 * M_PI},
        {"rs-01", 4 * M_PI},
        {"rs-02", 4 * M_PI},
        {"rs-03", 4 * M_PI},
        {"rs-04", 4 * M_PI},
        {"rs-05", 4 * M_PI},
        {"rs-06", 4 * M_PI},
    };
    return table;
}

const std::unordered_map<std::string, double>& mit_velocity_table() {
    static const std::unordered_map<std::string, double> table = {
        {"rs-00", 50},
        {"rs-01", 44},
        {"rs-02", 44},
        {"rs-03", 50},
        {"rs-04", 15},
        {"rs-05", 33},
        {"rs-06", 20},
    };
    return table;
}

const std::unordered_map<std::string, double>& mit_torque_table() {
    static const std::unordered_map<std::string, double> table = {
        {"rs-00", 17},
        {"rs-01", 17},
        {"rs-02", 17},
        {"rs-03", 60},
        {"rs-04", 120},
        {"rs-05", 17},
        {"rs-06", 60},
    };
    return table;
}

const std::unordered_map<std::string, double>& mit_kp_table() {
    static const std::unordered_map<std::string, double> table = {
        {"rs-00", 500.0},
        {"rs-01", 500.0},
        {"rs-02", 500.0},
        {"rs-03", 5000.0},
        {"rs-04", 5000.0},
        {"rs-05", 500.0},
        {"rs-06", 5000.0},
    };
    return table;
}

const std::unordered_map<std::string, double>& mit_kd_table() {
    static const std::unordered_map<std::string, double> table = {
        {"rs-00", 5.0},
        {"rs-01", 5.0},
        {"rs-02", 5.0},
        {"rs-03", 100.0},
        {"rs-04", 100.0},
        {"rs-05", 5.0},
        {"rs-06", 100.0},
    };
    return table;
}

} // namespace actuator_control

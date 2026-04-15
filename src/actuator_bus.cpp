#include "actuator_control/actuator_bus.hpp"

#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace actuator_control {

ActuatorBus::ActuatorBus(const std::string& channel, const MotorMap& motors,
                         const std::optional<Calibration>& calibration, int bitrate)
    : channel_(channel), motors_(motors), calibration_(calibration), bitrate_(bitrate) {
    std::set<int> id_set;
    for (const auto& [name, motor] : motors_) {
        if (!id_set.insert(motor.id).second) {
            throw std::invalid_argument("Motor IDs must be unique within an actuator bus.");
        }
    }

    if (calibration_) {
        std::cout << "Using calibration: {";
        bool first = true;
        for (const auto& [name, entry] : *calibration_) {
            if (!first) {
                std::cout << ", ";
            }
            std::cout << name << ": {direction=" << entry.direction
                      << ", homing_offset=" << entry.homing_offset << "}";
            first = false;
        }
        std::cout << "}" << std::endl;
    } else {
        std::cout << "WARNING: No calibration provided" << std::endl;
    }
}

ActuatorBus::~ActuatorBus() {
    try {
        if (channel_handler_) {
            channel_handler_->shutdown();
            channel_handler_.reset();
        }
    } catch (...) {
    }
}

std::string ActuatorBus::repr() const {
    std::ostringstream oss;
    oss << "ActuatorBus(channel='" << channel_ << "', motors={";
    bool first = true;
    for (const auto& [name, motor] : motors_) {
        if (!first) {
            oss << ", ";
        }
        oss << "'" << name << "': Motor{id=" << motor.id << ", model='" << motor.model << "'}";
        first = false;
    }
    oss << "}, bitrate=" << bitrate_ << ")";
    return oss.str();
}

void ActuatorBus::connect(bool /*handshake*/) {
    if (is_connected()) {
        throw std::runtime_error(repr() + " is already connected. Do not call connect() twice.");
    }

    channel_handler_ = std::make_unique<CanSocket>(channel_, bitrate_);
    std::cout << repr() << " connected." << std::endl;
}

void ActuatorBus::disconnect(bool disable_torque) {
    if (!is_connected()) {
        throw std::runtime_error(repr() + " is not connected. Try running connect() first.");
    }

    std::vector<std::string> disable_errors;
    if (disable_torque) {
        for (const auto& [name, motor] : motors_) {
            try {
                disable(name);
            } catch (const std::exception& exc) {
                std::ostringstream oss;
                oss << "Failed to disable motor '" << name << "' during disconnect: " << exc.what();
                disable_errors.push_back(oss.str());
            } catch (...) {
                std::ostringstream oss;
                oss << "Failed to disable motor '" << name << "' during disconnect: unknown error";
                disable_errors.push_back(oss.str());
            }
        }

        if (disable_errors.empty()) {
            std::cout << "Torque disabled for all motors." << std::endl;
        } else {
            for (const auto& error : disable_errors) {
                std::cout << "WARNING: " << error << std::endl;
            }
            std::cout << "WARNING: Disconnect completed with torque-disable errors." << std::endl;
        }
    }

    if (channel_handler_) {
        channel_handler_->shutdown();
        channel_handler_.reset();
    }

    std::cout << repr() << " disconnected." << std::endl;
}

void ActuatorBus::write_mit_kp_kd(const std::string& /*motor*/, double /*kp*/, double /*kd*/) {
    throw std::runtime_error("Not implemented");
}

void ActuatorBus::write_mit_control(
    const std::string& /*motor*/,
    double /*position*/,
    double /*velocity*/,
    double /*torque*/
) {
    throw std::runtime_error("Not implemented");
}

std::pair<double, double> ActuatorBus::read_mit_state(const std::string& /*motor*/) {
    throw std::runtime_error("Not implemented");
}

const std::vector<std::string>& ActuatorBus::models() {
    if (!models_cache_) {
        std::vector<std::string> values;
        values.reserve(motors_.size());
        for (const auto& [name, motor] : motors_) {
            values.push_back(motor.model);
        }
        models_cache_ = std::move(values);
    }
    return *models_cache_;
}

const std::vector<int>& ActuatorBus::ids() {
    if (!ids_cache_) {
        std::vector<int> values;
        values.reserve(motors_.size());
        for (const auto& [name, motor] : motors_) {
            values.push_back(motor.id);
        }
        ids_cache_ = std::move(values);
    }
    return *ids_cache_;
}

CanSocket& ActuatorBus::require_connected() {
    if (!channel_handler_) {
        throw std::runtime_error(repr() + " is not connected. Try running connect() first.");
    }
    return *channel_handler_;
}

const Motor& ActuatorBus::require_motor(const std::string& motor) const {
    auto it = motors_.find(motor);
    if (it == motors_.end()) {
        std::ostringstream oss;
        oss << "Unknown motor '" << motor << "'. Available motors: ";
        bool first = true;
        for (const auto& [name, value] : motors_) {
            if (!first) {
                oss << ", ";
            }
            oss << name;
            first = false;
        }
        if (motors_.empty()) {
            oss << "<none>";
        }
        throw std::out_of_range(oss.str());
    }
    return it->second;
}

std::tuple<double, double, double> ActuatorBus::process_mit_control(
    const std::string& motor,
    double position,
    double velocity,
    double torque
) const {
    if (calibration_) {
        auto it = calibration_->find(motor);
        if (it != calibration_->end()) {
            const auto& calibration = it->second;
            position = position * calibration.direction + calibration.homing_offset;
            velocity = velocity * calibration.direction;
            torque = torque * calibration.direction;
        }
    }
    return {position, velocity, torque};
}

std::pair<double, double> ActuatorBus::process_mit_state(
    const std::string& motor,
    double position,
    double velocity
) const {
    if (calibration_) {
        auto it = calibration_->find(motor);
        if (it != calibration_->end()) {
            const auto& calibration = it->second;
            position = (position - calibration.homing_offset) * calibration.direction;
            velocity = velocity * calibration.direction;
        }
    }
    return {position, velocity};
}

} // namespace actuator_control

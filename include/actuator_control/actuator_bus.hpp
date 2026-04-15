#pragma once

#include "actuator_control/can_socket.hpp"
#include "actuator_control/motor.hpp"

#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace actuator_control {

class ActuatorBus {
public:
    ActuatorBus(const std::string& channel, const MotorMap& motors,
                const std::optional<Calibration>& calibration = std::nullopt,
                int bitrate = 1000000);
    virtual ~ActuatorBus();

    ActuatorBus(const ActuatorBus&) = delete;
    ActuatorBus& operator=(const ActuatorBus&) = delete;

    size_t size() const { return motors_.size(); }
    std::string repr() const;
    bool is_connected() const { return channel_handler_ != nullptr; }
    const std::string& channel() const { return channel_; }
    const MotorMap& motors() const { return motors_; }
    const std::optional<Calibration>& calibration() const { return calibration_; }
    int bitrate() const { return bitrate_; }

    virtual void connect(bool handshake = true);
    virtual void disconnect(bool disable_torque = true);

    virtual void enable(const std::string& motor) = 0;
    virtual void disable(const std::string& motor) = 0;

    virtual void write_mit_kp_kd(const std::string& motor, double kp, double kd);
    virtual void write_mit_control(
        const std::string& motor,
        double position,
        double velocity = 0.0,
        double torque = 0.0
    );
    virtual std::pair<double, double> read_mit_state(const std::string& motor);

    const std::vector<std::string>& models();
    const std::vector<int>& ids();

protected:
    CanSocket& require_connected();
    const Motor& require_motor(const std::string& motor) const;

    std::tuple<double, double, double> process_mit_control(
        const std::string& motor,
        double position,
        double velocity,
        double torque
    ) const;
    std::pair<double, double> process_mit_state(
        const std::string& motor,
        double position,
        double velocity
    ) const;

    std::string channel_;
    MotorMap motors_;
    std::optional<Calibration> calibration_;
    int bitrate_;
    std::unique_ptr<CanSocket> channel_handler_;

private:
    std::optional<std::vector<std::string>> models_cache_;
    std::optional<std::vector<int>> ids_cache_;
};

} // namespace actuator_control

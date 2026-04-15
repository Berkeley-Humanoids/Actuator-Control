#pragma once

#include "actuator_control/actuator_bus.hpp"

#include <cmath>
#include <cstdint>
#include <optional>

namespace actuator_control {

namespace erob_config {
constexpr double COUNTS_PER_RAD = 524287.0 / (2.0 * M_PI);
} // namespace erob_config

namespace erob_command {
constexpr int START_MOTION = 0x83;
constexpr int STOP_MOTION = 0x84;
constexpr int SAVE_PARAMS = 0xE8;
} // namespace erob_command

namespace erob_param {
constexpr int ACTUAL_POSITION = 0x02;
constexpr int ACTUAL_SPEED = 0x05;
constexpr int MOTOR_CURRENT = 0x08;
constexpr int ERROR_CODE = 0x1F;
constexpr int RUN_STATUS = 0x20;
constexpr int POWER_TEMP = 0x26;
constexpr int CONTROL_MODE = 0x4E;
constexpr int MAX_POSITION_ERROR = 0x54;
constexpr int MAX_SPEED = 0x55;
constexpr int TARGET_POSITION = 0x86;
constexpr int RELATIVE_POSITION = 0x87;
constexpr int ACCELERATION = 0x88;
constexpr int DECELERATION = 0x89;
constexpr int TARGET_SPEED = 0x8A;
constexpr int MOTION_MODE = 0x8D;
constexpr int PID_ADJUSTMENT = 0x0124;
constexpr int POSITION_LOOP_GAIN = 0x64;
constexpr int SPEED_LOOP_GAIN = 0x66;
constexpr int SPEED_LOOP_INTEGRAL = 0x67;
} // namespace erob_param

namespace erob_mode {
constexpr int TORQUE = 1;
constexpr int SPEED = 2;
constexpr int POSITION = 3;
} // namespace erob_mode

class ERobBus : public ActuatorBus {
public:
    using ActuatorBus::ActuatorBus;

    static constexpr uint32_t CLIENT_ID_BASE = 0x640;
    static constexpr uint32_t SERVER_ID_BASE = 0x5C0;
    static constexpr uint8_t STATUS_SUCCESS = 0x3E;

    std::optional<std::vector<uint8_t>> receive(int device_id, double timeout = 0.1);
    void transmit(int device_id, const uint8_t* data = nullptr, size_t len = 0);
    void send_command(const std::string& motor, int command);

    void enable(const std::string& motor) override;
    void disable(const std::string& motor) override;

    std::optional<int32_t> read(
        const std::string& motor,
        int parameter,
        const uint8_t* extra = nullptr,
        size_t extra_len = 0
    );
    void write(const std::string& motor, int param, int32_t value);
    void write_subindex(const std::string& motor, int param, int subindex, int32_t value);

    void write_mit_kp_kd(const std::string& motor, double kp, double kd) override;
    void write_mit_control(
        const std::string& motor,
        double position,
        double velocity = 0.0,
        double torque = 0.0
    ) override;
    std::pair<double, double> read_mit_state(const std::string& motor) override;

private:
    std::pair<int, int> calculate_erob_pd(double desired_kp, double desired_kd);
};

} // namespace actuator_control

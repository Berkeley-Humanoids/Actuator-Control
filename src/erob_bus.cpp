#include "actuator_control/erob_bus.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>

#ifdef __linux__
#include <endian.h>
#else
#error "This library requires Linux."
#endif

namespace actuator_control {

namespace {

void pack_be16(uint8_t* buf, uint16_t value) {
    uint16_t be = htobe16(value);
    std::memcpy(buf, &be, 2);
}

void pack_be32(uint8_t* buf, uint32_t value) {
    uint32_t be = htobe32(value);
    std::memcpy(buf, &be, 4);
}

void pack_be32_signed(uint8_t* buf, int32_t value) {
    uint32_t unsigned_value;
    std::memcpy(&unsigned_value, &value, 4);
    pack_be32(buf, unsigned_value);
}

int32_t unpack_be32_signed(const uint8_t* buf) {
    uint32_t unsigned_value;
    std::memcpy(&unsigned_value, buf, 4);
    unsigned_value = be32toh(unsigned_value);
    int32_t signed_value;
    std::memcpy(&signed_value, &unsigned_value, 4);
    return signed_value;
}

int16_t unpack_be16_signed(const uint8_t* buf) {
    uint16_t unsigned_value;
    std::memcpy(&unsigned_value, buf, 2);
    unsigned_value = be16toh(unsigned_value);
    int16_t signed_value;
    std::memcpy(&signed_value, &unsigned_value, 2);
    return signed_value;
}

} // namespace

static const uint8_t k_default_data[8] = {};

void ERobBus::transmit(int device_id, const uint8_t* data, size_t len) {
    CanFrame frame;
    frame.arbitration_id = CLIENT_ID_BASE | static_cast<uint32_t>(device_id);
    frame.is_extended_id = false;

    if (data && len > 0) {
        frame.dlc = static_cast<uint8_t>(len > 8 ? 8 : len);
        std::memcpy(frame.data, data, frame.dlc);
    } else {
        frame.dlc = 8;
        std::memcpy(frame.data, k_default_data, 8);
    }

    require_connected().send(frame);
}

std::optional<std::vector<uint8_t>> ERobBus::receive(int device_id, double timeout) {
    uint32_t expected_id = SERVER_ID_BASE | static_cast<uint32_t>(device_id);

    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    while (true) {
        double remaining = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
        if (remaining <= 0) {
            break;
        }

        auto frame = require_connected().recv(remaining);
        if (!frame) {
            break;
        }
        if (frame->arbitration_id != expected_id) {
            continue;
        }

        if (frame->dlc == 1 && frame->data[0] == STATUS_SUCCESS) {
            return std::vector<uint8_t>{};
        }

        if (frame->dlc >= 5 && frame->data[4] == STATUS_SUCCESS) {
            return std::vector<uint8_t>(frame->data, frame->data + 4);
        }

        if (frame->dlc == 3 && frame->data[2] == STATUS_SUCCESS) {
            return std::nullopt;
        }

        return std::nullopt;
    }

    if (timeout > 0) {
        std::cout << "No frame received" << std::endl;
    }
    return std::nullopt;
}

void ERobBus::send_command(const std::string& motor, int command) {
    const auto& motor_config = require_motor(motor);
    uint8_t data[2];
    data[0] = 0x00;
    data[1] = static_cast<uint8_t>(command);
    transmit(motor_config.id, data, 2);
    receive(motor_config.id, 0.1);
}

void ERobBus::disable(const std::string& motor) {
    const auto& motor_config = require_motor(motor);
    send_command(motor, erob_command::STOP_MOTION);
    uint8_t data[6];
    data[0] = 0x01;
    data[1] = 0x00;
    pack_be32(data + 2, 0x00000000);
    transmit(motor_config.id, data, 6);
    receive(motor_config.id, 0.1);
}

void ERobBus::enable(const std::string& motor) {
    const auto& motor_config = require_motor(motor);

    write(motor, erob_param::CONTROL_MODE, erob_mode::POSITION);
    write(motor, erob_param::MOTION_MODE, 1);

    int accel = static_cast<int>(erob_config::COUNTS_PER_RAD * 100.0 * (2.0 * M_PI));
    int decel = accel;
    int speed = accel;
    write(motor, erob_param::ACCELERATION, accel);
    write(motor, erob_param::DECELERATION, decel);
    write(motor, erob_param::TARGET_SPEED, speed);

    read(motor, erob_param::ERROR_CODE);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    uint8_t data[6];
    data[0] = 0x01;
    data[1] = 0x00;
    pack_be32(data + 2, 0x00000001);
    transmit(motor_config.id, data, 6);
    receive(motor_config.id, 0.1);
}

std::optional<int32_t> ERobBus::read(
    const std::string& motor,
    int parameter,
    const uint8_t* extra,
    size_t extra_len
) {
    const auto& motor_config = require_motor(motor);

    uint8_t data[8] = {};
    pack_be16(data, static_cast<uint16_t>(parameter));
    size_t total = 2;
    if (extra && extra_len > 0) {
        size_t copy_len = extra_len > 6 ? 6 : extra_len;
        std::memcpy(data + 2, extra, copy_len);
        total += copy_len;
    }
    transmit(motor_config.id, data, total);
    auto result = receive(motor_config.id, 0.1);

    if (!result) {
        return std::nullopt;
    }
    if (result->size() == 4) {
        return unpack_be32_signed(result->data());
    }
    if (result->size() == 2) {
        return static_cast<int32_t>(unpack_be16_signed(result->data()));
    }
    return std::nullopt;
}

void ERobBus::write(const std::string& motor, int param, int32_t value) {
    const auto& motor_config = require_motor(motor);
    if (param == erob_param::CONTROL_MODE || param == erob_param::MOTION_MODE) {
        value = value & 0xFF;
    }

    uint8_t data[6];
    pack_be16(data, static_cast<uint16_t>(param));
    pack_be32_signed(data + 2, value);
    transmit(motor_config.id, data, 6);
    receive(motor_config.id, 0.1);
}

void ERobBus::write_subindex(const std::string& motor, int param, int subindex, int32_t value) {
    const auto& motor_config = require_motor(motor);
    uint8_t data[8];
    pack_be16(data, static_cast<uint16_t>(param));
    pack_be16(data + 2, static_cast<uint16_t>(subindex));
    pack_be32_signed(data + 4, value);
    transmit(motor_config.id, data, 8);
    receive(motor_config.id, 0.1);
}

std::pair<int, int> ERobBus::calculate_erob_pd(double desired_kp, double desired_kd) {
    if (desired_kd == 0) {
        return {0, 0};
    }

    constexpr double torque_constant = 0.132e-3;
    constexpr double gear_ratio = 50.0;
    constexpr double position_gain = 1000.0 / 37.5e-4;
    constexpr double velocity_gain = 1.0 / 19.3e-5;

    double ma_per_count = desired_kd / (torque_constant * erob_config::COUNTS_PER_RAD * gear_ratio);
    double kd_erob = ma_per_count * velocity_gain;
    double kp_erob = (desired_kp * position_gain) / (desired_kd * velocity_gain);
    return {static_cast<int>(std::llround(kp_erob)), static_cast<int>(std::llround(kd_erob))};
}

void ERobBus::write_mit_kp_kd(const std::string& motor, double kp, double kd) {
    auto can_pid_flag = read(motor, erob_param::PID_ADJUSTMENT);
    if (can_pid_flag && *can_pid_flag == 1) {
        write(motor, erob_param::PID_ADJUSTMENT, 0x00);
    }

    auto [kp_int, kd_int] = calculate_erob_pd(kp, kd);
    write_subindex(motor, erob_param::POSITION_LOOP_GAIN, 0x01, kp_int);
    write_subindex(motor, erob_param::SPEED_LOOP_GAIN, 0x01, kd_int);
    write_subindex(motor, erob_param::SPEED_LOOP_INTEGRAL, 0x01, 0);
}

void ERobBus::write_mit_control(const std::string& motor, double position, double velocity, double torque) {
    auto [processed_position, processed_velocity, processed_torque] =
        process_mit_control(motor, position, velocity, torque);
    (void)processed_velocity;
    (void)processed_torque;

    int target_counts = static_cast<int>((processed_position + M_PI) * erob_config::COUNTS_PER_RAD);
    write(motor, erob_param::TARGET_POSITION, target_counts);
    send_command(motor, erob_command::START_MOTION);
}

std::pair<double, double> ERobBus::read_mit_state(const std::string& motor) {
    auto position_counts = read(motor, erob_param::ACTUAL_POSITION);
    if (!position_counts) {
        throw std::runtime_error("Failed to read position");
    }
    double position = static_cast<double>(*position_counts) / erob_config::COUNTS_PER_RAD - M_PI;

    const uint8_t velocity_subindex[2] = {0x00, 0x01};
    auto velocity_counts = read(motor, erob_param::ACTUAL_SPEED, velocity_subindex, 2);
    if (!velocity_counts) {
        throw std::runtime_error("Failed to read velocity");
    }
    double velocity = static_cast<double>(*velocity_counts) / erob_config::COUNTS_PER_RAD;

    return process_mit_state(motor, position, velocity);
}

} // namespace actuator_control

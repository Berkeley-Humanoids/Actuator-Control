#include "actuator_control/sito_bus.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>

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

void pack_be16_signed(uint8_t* buf, int16_t value) {
    uint16_t unsigned_value;
    std::memcpy(&unsigned_value, &value, 2);
    pack_be16(buf, unsigned_value);
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

void pack_be_float(uint8_t* buf, float value) {
    uint32_t raw;
    std::memcpy(&raw, &value, 4);
    pack_be32(buf, raw);
}

int16_t unpack_be16_signed(const uint8_t* buf) {
    uint16_t unsigned_value;
    std::memcpy(&unsigned_value, buf, 2);
    unsigned_value = be16toh(unsigned_value);
    int16_t signed_value;
    std::memcpy(&signed_value, &unsigned_value, 2);
    return signed_value;
}

int32_t unpack_be32_signed(const uint8_t* buf) {
    uint32_t unsigned_value;
    std::memcpy(&unsigned_value, buf, 4);
    unsigned_value = be32toh(unsigned_value);
    int32_t signed_value;
    std::memcpy(&signed_value, &unsigned_value, 4);
    return signed_value;
}

} // namespace

const std::map<std::string, SitoBus::SitoMotorConfig>& SitoBus::motor_configs() {
    static const std::map<std::string, SitoMotorConfig> configs = {
        {"TA40-50", {65536.0 / (2.0 * M_PI), 51.0, 0.00009}},
        {"TA40-100", {65536.0 / (2.0 * M_PI), 101.0, 0.00009}},
    };
    return configs;
}

SitoBus::SitoBus(const std::string& channel, const MotorMap& motors,
                 const std::optional<Calibration>& calibration, int bitrate,
                 double control_frequency)
    : ActuatorBus(channel, motors, calibration, bitrate) {
    if (control_frequency <= 0) {
        throw std::invalid_argument("control_frequency must be greater than zero.");
    }
    int interval_ms = static_cast<int>(1000.0 / control_frequency);
    if (interval_ms <= 0) {
        throw std::invalid_argument("Message interval must be greater than 0 ms.");
    }
    if (interval_ms >= 0xFF) {
        throw std::invalid_argument("Message interval must be less than 255 ms.");
    }

    feedback_1_interval_ = interval_ms;
    feedback_2_interval_ = interval_ms;

    for (const auto& [name, motor] : motors_) {
        motor_status_[name] = {};
    }
}

SitoBus::~SitoBus() {
    try {
        stop_receiver_thread();
    } catch (...) {
    }
}

void SitoBus::connect(bool handshake) {
    ActuatorBus::connect(handshake);
    start_receiver_thread();
}

void SitoBus::disconnect(bool disable_torque) {
    stop_receiver_thread();
    ActuatorBus::disconnect(disable_torque);
}

void SitoBus::start_receiver_thread() {
    if (receiver_thread_ && receiver_thread_->joinable()) {
        return;
    }
    killed_ = false;
    receiver_thread_ = std::make_unique<std::thread>(&SitoBus::run_message_receiver, this);
}

void SitoBus::stop_receiver_thread() {
    killed_ = true;
    if (receiver_thread_ && receiver_thread_->joinable()) {
        receiver_thread_->join();
    }
    receiver_thread_.reset();
}

uint32_t SitoBus::make_arbitration_id(const std::string& motor, int comm_type) const {
    const auto& motor_config = require_motor(motor);
    return 0x05060000u | (static_cast<uint32_t>(motor_config.id) << 8) | static_cast<uint32_t>(comm_type);
}

std::string SitoBus::motor_name_from_id(int motor_id) const {
    for (const auto& [name, motor] : motors_) {
        if (motor.id == motor_id) {
            return name;
        }
    }
    return {};
}

const SitoBus::SitoMotorConfig& SitoBus::get_motor_config(const std::string& motor) const {
    const auto& motor_config = require_motor(motor);
    auto it = motor_configs().find(motor_config.model);
    if (it == motor_configs().end()) {
        std::ostringstream oss;
        oss << "Unsupported Sito motor model '" << motor_config.model << "' for motor '" << motor
            << "'. Supported models: ";
        bool first = true;
        for (const auto& [model, _] : motor_configs()) {
            if (!first) {
                oss << ", ";
            }
            oss << model;
            first = false;
        }
        throw std::out_of_range(oss.str());
    }
    return it->second;
}

void SitoBus::run_message_receiver() {
    while (!killed_.load()) {
        if (!is_connected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        auto* bus = channel_handler_.get();
        if (!bus) {
            continue;
        }

        std::optional<CanFrame> frame;
        try {
            frame = bus->recv(0.001);
        } catch (...) {
            if (!killed_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            continue;
        }

        if (!frame) {
            continue;
        }

        uint32_t arbitration_id = frame->arbitration_id;
        if ((arbitration_id & 0xFFFF0000u) != 0x05060000u) {
            continue;
        }

        int motor_id = (arbitration_id >> 8) & 0xFF;
        std::string motor_name = motor_name_from_id(motor_id);
        if (motor_name.empty()) {
            continue;
        }

        int message_type = arbitration_id & 0xFF;
        const auto& config = get_motor_config(motor_name);

        if (message_type == sito_comm::FEEDBACK_1 && frame->dlc >= 8) {
            int16_t velocity_counts = unpack_be16_signed(frame->data + 6);
            int16_t phase_current = unpack_be16_signed(frame->data + 4);

            double velocity = static_cast<double>(velocity_counts) / config.actuator_counts_per_rad;
            double torque = static_cast<double>(phase_current) * config.torque_constant * config.gear_ratio;

            std::lock_guard<std::mutex> lock(status_mutex_);
            motor_status_[motor_name].velocity = velocity;
            motor_status_[motor_name].torque = torque;
        } else if (message_type == sito_comm::FEEDBACK_2 && frame->dlc >= 8) {
            int32_t output_position_counts = unpack_be32_signed(frame->data + 4);
            double position = static_cast<double>(output_position_counts) / config.actuator_counts_per_rad;

            std::lock_guard<std::mutex> lock(status_mutex_);
            motor_status_[motor_name].position = position;
        }
    }
    std::cout << "Message receiver exited." << std::endl;
}

void SitoBus::enable(const std::string& motor) {
    auto& bus = require_connected();

    uint32_t arbitration_id = make_arbitration_id(motor, sito_comm::SELECT_MODE);
    uint8_t data[8] = {};
    data[0] = static_cast<uint8_t>(sito_mode::MIT);
    data[1] = static_cast<uint8_t>(feedback_1_interval_);
    data[2] = static_cast<uint8_t>(feedback_2_interval_);
    data[3] = static_cast<uint8_t>(feedback_3_interval_);

    CanFrame frame;
    frame.arbitration_id = arbitration_id;
    frame.is_extended_id = true;
    frame.dlc = 8;
    std::memcpy(frame.data, data, 8);
    bus.send(frame);
}

void SitoBus::disable(const std::string& motor) {
    auto& bus = require_connected();

    uint32_t arbitration_id = make_arbitration_id(motor, sito_comm::RESET);
    uint8_t data[8];
    pack_be32(data, 0x55555555u);
    pack_be32(data + 4, 0x55555555u);

    CanFrame frame;
    frame.arbitration_id = arbitration_id;
    frame.is_extended_id = true;
    frame.dlc = 8;
    std::memcpy(frame.data, data, 8);
    bus.send(frame);
}

void SitoBus::write_mit_kp_kd(const std::string& motor, double kp, double kd) {
    auto& bus = require_connected();

    double kp_sito = kp / 500.0;
    double kd_sito = kd / 0.5;
    uint32_t arbitration_id = make_arbitration_id(motor, sito_comm::SET_MIT_KP_KD);

    uint8_t data[8];
    pack_be_float(data, static_cast<float>(kp_sito));
    pack_be_float(data + 4, static_cast<float>(kd_sito));

    CanFrame frame;
    frame.arbitration_id = arbitration_id;
    frame.is_extended_id = true;
    frame.dlc = 8;
    std::memcpy(frame.data, data, 8);
    bus.send(frame);
}

void SitoBus::write_mit_control(const std::string& motor, double position, double velocity, double torque) {
    auto& bus = require_connected();

    auto [processed_position, processed_velocity, processed_torque] =
        process_mit_control(motor, position, velocity, torque);

    uint32_t arbitration_id = make_arbitration_id(motor, sito_comm::SET_MIT_CURRENT_VELOCITY_POSITION);
    const auto& config = get_motor_config(motor);

    double current = processed_torque / config.torque_constant / config.gear_ratio;
    double position_counts = config.actuator_counts_per_rad * processed_position;
    double velocity_counts = config.actuator_counts_per_rad * processed_velocity;

    uint8_t data[8];
    pack_be16_signed(data, static_cast<int16_t>(current));
    pack_be16_signed(data + 2, static_cast<int16_t>(velocity_counts));
    pack_be32_signed(data + 4, static_cast<int32_t>(position_counts));

    CanFrame frame;
    frame.arbitration_id = arbitration_id;
    frame.is_extended_id = true;
    frame.dlc = 8;
    std::memcpy(frame.data, data, 8);
    bus.send(frame);
}

std::pair<double, double> SitoBus::read_mit_state(const std::string& motor) {
    require_motor(motor);

    double position;
    double velocity;
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        position = motor_status_[motor].position;
        velocity = motor_status_[motor].velocity;
    }

    return process_mit_state(motor, position, velocity);
}

} // namespace actuator_control

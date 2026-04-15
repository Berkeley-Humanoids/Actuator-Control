#include "actuator_control/robstride_bus.hpp"

#include "actuator_control/robstride_table.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
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

void pack_le16(uint8_t* buf, uint16_t value) {
    uint16_t le = htole16(value);
    std::memcpy(buf, &le, 2);
}

void pack_le32(uint8_t* buf, uint32_t value) {
    uint32_t le = htole32(value);
    std::memcpy(buf, &le, 4);
}

void pack_le_float(uint8_t* buf, float value) {
    uint32_t raw;
    std::memcpy(&raw, &value, 4);
    pack_le32(buf, raw);
}

uint16_t unpack_le16(const uint8_t* buf) {
    uint16_t value;
    std::memcpy(&value, buf, 2);
    return le16toh(value);
}

uint32_t unpack_le32(const uint8_t* buf) {
    uint32_t value;
    std::memcpy(&value, buf, 4);
    return le32toh(value);
}

float unpack_le_float(const uint8_t* buf) {
    uint32_t raw = unpack_le32(buf);
    float value;
    std::memcpy(&value, &raw, 4);
    return value;
}

void pack_be16(uint8_t* buf, uint16_t value) {
    uint16_t be = htobe16(value);
    std::memcpy(buf, &be, 2);
}

constexpr int k_disable_attempts = 3;
constexpr auto k_disable_retry_delay = std::chrono::milliseconds(10);

template <typename SendFn>
void run_disable_with_retries(const SendFn& send_fn) {
    std::exception_ptr last_error;
    for (int attempt = 0; attempt < k_disable_attempts; ++attempt) {
        try {
            send_fn();
            return;
        } catch (...) {
            last_error = std::current_exception();
            if (attempt + 1 < k_disable_attempts) {
                std::this_thread::sleep_for(k_disable_retry_delay);
            }
        }
    }
    if (last_error) {
        std::rethrow_exception(last_error);
    }
}

std::vector<std::string> dedupe_statuses(const std::vector<std::string>& statuses) {
    std::vector<std::string> unique;
    for (const auto& status : statuses) {
        if (std::find(unique.begin(), unique.end(), status) == unique.end()) {
            unique.push_back(status);
        }
    }
    return unique;
}

} // namespace

static const uint8_t k_zero_data[8] = {};

RobstrideBus::RobstrideBus(const std::string& channel, const MotorMap& motors,
                           const std::optional<Calibration>& calibration, int bitrate)
    : ActuatorBus(channel, motors, calibration, bitrate) {
    for (const auto& [name, motor] : motors_) {
        motor_status_[name] = {};
    }
}

RobstrideBus::~RobstrideBus() {
    try {
        stop_receiver();
    } catch (...) {
    }
}

void RobstrideBus::disconnect(bool disable_torque) {
    stop_receiver();
    ActuatorBus::disconnect(disable_torque);
}

void RobstrideBus::start_receiver() {
    if (receiver_thread_ && receiver_thread_->joinable()) {
        return;
    }
    killed_ = false;
    receiver_thread_ = std::make_unique<std::thread>(&RobstrideBus::run_message_receiver, this);
}

void RobstrideBus::stop_receiver() {
    killed_ = true;
    if (receiver_thread_ && receiver_thread_->joinable()) {
        receiver_thread_->join();
    }
    receiver_thread_.reset();
}

bool RobstrideBus::is_receiver_running() const {
    return receiver_thread_ && receiver_thread_->joinable() && !killed_;
}

std::string RobstrideBus::motor_name_from_id(int motor_id) const {
    for (const auto& [name, motor] : motors_) {
        if (motor.id == motor_id) {
            return name;
        }
    }
    return {};
}

void RobstrideBus::decode_status_into_cache(
    const std::string& motor,
    const std::string& model,
    int extra_data,
    const uint8_t* data,
    size_t len
) {
    std::vector<std::string> fault_list;
    if ((extra_data >> 13) & 0x01) {
        fault_list.push_back("uncalibrated");
    }
    if ((extra_data >> 12) & 0x01) {
        fault_list.push_back("stall overload");
    }
    if ((extra_data >> 11) & 0x01) {
        fault_list.push_back("magnetic encoder fault");
    }
    if ((extra_data >> 10) & 0x01) {
        fault_list.push_back("overtemperature");
    }
    if ((extra_data >> 9) & 0x01) {
        fault_list.push_back("gate driver fault");
    }
    if ((extra_data >> 8) & 0x01) {
        fault_list.push_back("undervoltage");
    }

    if (len < 8) {
        return;
    }

    uint16_t position_u16 = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    uint16_t velocity_u16 = (static_cast<uint16_t>(data[2]) << 8) | data[3];
    uint16_t torque_u16 = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    uint16_t temperature_u16 = (static_cast<uint16_t>(data[6]) << 8) | data[7];

    double position = (static_cast<double>(position_u16) / 0x7FFF - 1.0) * mit_position_table().at(model);
    double velocity = (static_cast<double>(velocity_u16) / 0x7FFF - 1.0) * mit_velocity_table().at(model);
    double torque = (static_cast<double>(torque_u16) / 0x7FFF - 1.0) * mit_torque_table().at(model);
    double temperature = static_cast<double>(temperature_u16) * 0.1;

    std::lock_guard<std::mutex> lock(status_mutex_);
    motor_status_[motor] = {position, velocity, torque, temperature};
    update_fault_status(motor, dedupe_statuses(fault_list));
}

void RobstrideBus::run_message_receiver() {
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

        if (!frame || !frame->is_extended_id) {
            continue;
        }

        int communication_type = (frame->arbitration_id >> 24) & 0x1F;
        int extra_data = (frame->arbitration_id >> 8) & 0xFFFF;
        if (communication_type != robstride_comm::OPERATION_STATUS &&
            communication_type != robstride_comm::FAULT_REPORT) {
            continue;
        }

        int motor_id = extra_data & 0xFF;
        std::string motor_name = motor_name_from_id(motor_id);
        if (motor_name.empty()) {
            continue;
        }

        if (mit_position_table().find(motors_.at(motor_name).model) == mit_position_table().end()) {
            continue;
        }

        if (communication_type == robstride_comm::OPERATION_STATUS) {
            decode_status_into_cache(
                motor_name,
                motors_.at(motor_name).model,
                extra_data,
                frame->data,
                frame->dlc
            );
        } else {
            std::vector<std::string> fault_list;
            if ((extra_data >> 13) & 0x01) {
                fault_list.push_back("uncalibrated");
            }
            if ((extra_data >> 12) & 0x01) {
                fault_list.push_back("stall overload");
            }
            if ((extra_data >> 11) & 0x01) {
                fault_list.push_back("magnetic encoder fault");
            }
            if ((extra_data >> 10) & 0x01) {
                fault_list.push_back("overtemperature");
            }
            if ((extra_data >> 9) & 0x01) {
                fault_list.push_back("gate driver fault");
            }
            if ((extra_data >> 8) & 0x01) {
                fault_list.push_back("undervoltage");
            }

            if (frame->dlc >= 8) {
                uint32_t fault_value = unpack_le32(frame->data);
                uint32_t warning_value = unpack_le32(frame->data + 4);
                if (fault_value & 0x01) {
                    fault_list.push_back("overtemperature");
                }
                if ((fault_value >> 1) & 0x01) {
                    fault_list.push_back("gate driver fault");
                }
                if ((fault_value >> 2) & 0x01) {
                    fault_list.push_back("undervoltage");
                }
                if ((fault_value >> 3) & 0x01) {
                    fault_list.push_back("overvoltage");
                }
                if ((fault_value >> 7) & 0x01) {
                    fault_list.push_back("uncalibrated");
                }
                if ((fault_value >> 14) & 0x01) {
                    fault_list.push_back("stall overload");
                }
                if (warning_value & 0x01) {
                    fault_list.push_back("overtemperature warning");
                }
            }

            std::lock_guard<std::mutex> lock(status_mutex_);
            update_fault_status(motor_name, dedupe_statuses(fault_list));
        }
    }
    std::cout << "Robstride message receiver exited." << std::endl;
}

const std::string& RobstrideBus::require_model(const std::string& motor) {
    const auto& motor_config = require_motor(motor);
    auto it = mit_position_table().find(motor_config.model);
    if (it == mit_position_table().end()) {
        std::ostringstream oss;
        oss << "Unsupported Robstride model '" << motor_config.model << "' for motor '" << motor
            << "'. Supported models: ";
        bool first = true;
        for (const auto& [model, _] : mit_position_table()) {
            if (!first) {
                oss << ", ";
            }
            oss << model;
            first = false;
        }
        throw std::out_of_range(oss.str());
    }
    return motor_config.model;
}

void RobstrideBus::update_fault_status(
    const std::string& motor,
    const std::vector<std::string>& statuses
) {
    auto unique = dedupe_statuses(statuses);
    if (!unique.empty()) {
        fault_status_[motor] = std::move(unique);
    } else {
        fault_status_.erase(motor);
    }
}

void RobstrideBus::connect(bool handshake) {
    ActuatorBus::connect(handshake);
    drain_rx_buffer(0.05);
}

int RobstrideBus::drain_rx_buffer(double timeout, int max_frames) {
    if (!is_connected()) {
        return 0;
    }

    auto& bus = require_connected();
    int drained = 0;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(std::max(timeout, 0.0));
    while (drained < max_frames) {
        double remaining = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
        remaining = std::max(remaining, 0.0);
        auto frame = bus.recv(remaining);
        if (!frame) {
            break;
        }
        drained++;
    }

    if (drained > 0) {
        std::cout << "Drained " << drained << " pending CAN frame(s) from " << channel_ << "." << std::endl;
    }
    return drained;
}

void RobstrideBus::transmit(
    int communication_type,
    int extra_data,
    int device_id,
    const uint8_t* data,
    size_t len
) {
    if (communication_type < 0 || communication_type > 0x1F) {
        throw std::invalid_argument("communication_type out of range.");
    }
    if (extra_data < 0 || extra_data > 0xFFFF) {
        throw std::invalid_argument("extra_data out of range.");
    }
    if (device_id <= 0 || device_id > 0xFF) {
        throw std::invalid_argument("device_id out of range.");
    }

    CanFrame frame;
    frame.arbitration_id = (static_cast<uint32_t>(communication_type) << 24) |
                           (static_cast<uint32_t>(extra_data) << 8) |
                           static_cast<uint32_t>(device_id);
    frame.is_extended_id = true;

    if (data && len > 0) {
        frame.dlc = static_cast<uint8_t>(len > 8 ? 8 : len);
        std::memcpy(frame.data, data, frame.dlc);
    } else {
        frame.dlc = 8;
        std::memcpy(frame.data, k_zero_data, 8);
    }

    require_connected().send(frame);
}

std::optional<std::tuple<int, int, int, std::vector<uint8_t>>> RobstrideBus::receive(double timeout) {
    auto& bus = require_connected();
    auto start = std::chrono::steady_clock::now();

    while (true) {
        double remaining;
        if (timeout < 0) {
            remaining = -1.0;
        } else {
            remaining = timeout - std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            if (remaining <= 0) {
                return std::nullopt;
            }
        }

        auto frame = bus.recv(remaining);
        if (!frame) {
            // std::cout << "WARNING: Received no response from the motor" << std::endl;
            return std::nullopt;
        }
        if (!frame->is_extended_id) {
            continue;
        }

        int communication_type = (frame->arbitration_id >> 24) & 0x1F;
        int extra_data = (frame->arbitration_id >> 8) & 0xFFFF;
        int device_id = frame->arbitration_id & 0xFF;
        return std::make_tuple(
            communication_type,
            extra_data,
            device_id,
            std::vector<uint8_t>(frame->data, frame->data + frame->dlc)
        );
    }
}

std::optional<std::tuple<int, int, int, std::vector<uint8_t>>> RobstrideBus::receive_matching_frame(
    const FramePredicate& predicate,
    double timeout,
    const std::string& description
) {
    auto& bus = require_connected();

    auto deadline = timeout < 0
        ? std::chrono::steady_clock::time_point::max()
        : std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);

    while (true) {
        double remaining;
        if (timeout < 0) {
            remaining = -1.0;
        } else {
            remaining = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
            if (remaining <= 0) {
                return std::nullopt;
            }
        }

        auto frame = bus.recv(remaining);
        if (!frame) {
            std::cout << "WARNING: Received no response from the motor" << std::endl;
            return std::nullopt;
        }
        if (!frame->is_extended_id) {
            continue;
        }

        int communication_type = (frame->arbitration_id >> 24) & 0x1F;
        int extra_data = (frame->arbitration_id >> 8) & 0xFFFF;
        int device_id = frame->arbitration_id & 0xFF;

        if (predicate(communication_type, extra_data, device_id, frame->data, frame->dlc)) {
            return std::make_tuple(
                communication_type,
                extra_data,
                device_id,
                std::vector<uint8_t>(frame->data, frame->data + frame->dlc)
            );
        }

        std::cout << "Ignoring unrelated Robstride frame on " << channel_
                  << " while waiting for " << description << ": comm=" << communication_type
                  << " extra=0x" << std::hex << extra_data << std::dec << " device=" << device_id
                  << std::endl;
    }
}

RobstrideBus::StatusFrame RobstrideBus::receive_status_frame(const std::string& motor) {
    int motor_id = require_motor(motor).id;
    auto result = receive_matching_frame(
        [motor_id](int communication_type, int extra_data, int /*device_id*/, const uint8_t* /*data*/, size_t /*len*/) {
            return (communication_type == robstride_comm::OPERATION_STATUS ||
                    communication_type == robstride_comm::FAULT_REPORT) &&
                   (extra_data & 0xFF) == motor_id;
        },
        0.1,
        "status frame for '" + motor + "'"
    );

    if (!result) {
        throw std::runtime_error("No response from the motor '" + motor + "'.");
    }

    auto [communication_type, extra_data, host_id, data] = *result;
    (void)host_id;
    const auto& model = require_model(motor);

    std::vector<std::string> fault_list;
    if ((extra_data >> 13) & 0x01) {
        fault_list.push_back("uncalibrated");
    }
    if ((extra_data >> 12) & 0x01) {
        fault_list.push_back("stall overload");
    }
    if ((extra_data >> 11) & 0x01) {
        fault_list.push_back("magnetic encoder fault");
    }
    if ((extra_data >> 10) & 0x01) {
        fault_list.push_back("overtemperature");
    }
    if ((extra_data >> 9) & 0x01) {
        fault_list.push_back("gate driver fault");
    }
    if ((extra_data >> 8) & 0x01) {
        fault_list.push_back("undervoltage");
    }

    if (communication_type != robstride_comm::OPERATION_STATUS &&
        communication_type != robstride_comm::FAULT_REPORT) {
        throw std::runtime_error("Invalid communication type: " + std::to_string(communication_type));
    }

    if (communication_type == robstride_comm::FAULT_REPORT) {
        if (data.size() < 8) {
            throw std::runtime_error("Fault frame too short: " + std::to_string(data.size()));
        }

        uint32_t fault_value = unpack_le32(data.data());
        uint32_t warning_value = unpack_le32(data.data() + 4);
        if (fault_value & 0x01) {
            fault_list.push_back("overtemperature");
        }
        if ((fault_value >> 1) & 0x01) {
            fault_list.push_back("gate driver fault");
        }
        if ((fault_value >> 2) & 0x01) {
            fault_list.push_back("undervoltage");
        }
        if ((fault_value >> 3) & 0x01) {
            fault_list.push_back("overvoltage");
        }
        if ((fault_value >> 7) & 0x01) {
            fault_list.push_back("uncalibrated");
        }
        if ((fault_value >> 14) & 0x01) {
            fault_list.push_back("stall overload");
        }
        if (warning_value & 0x01) {
            fault_list.push_back("overtemperature warning");
        }

        std::lock_guard<std::mutex> lock(status_mutex_);
        update_fault_status(motor, dedupe_statuses(fault_list));
        throw std::runtime_error("Received fault frame from " + motor);
    }

    if (data.size() != 8) {
        throw std::runtime_error("Invalid Robstride status payload length: " + std::to_string(data.size()) + ".");
    }

    uint16_t position_u16 = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    uint16_t velocity_u16 = (static_cast<uint16_t>(data[2]) << 8) | data[3];
    uint16_t torque_u16 = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    uint16_t temperature_u16 = (static_cast<uint16_t>(data[6]) << 8) | data[7];

    double position = (static_cast<double>(position_u16) / 0x7FFF - 1.0) * mit_position_table().at(model);
    double velocity = (static_cast<double>(velocity_u16) / 0x7FFF - 1.0) * mit_velocity_table().at(model);
    double torque = (static_cast<double>(torque_u16) / 0x7FFF - 1.0) * mit_torque_table().at(model);
    double temperature = static_cast<double>(temperature_u16) * 0.1;

    std::lock_guard<std::mutex> lock(status_mutex_);
    motor_status_[motor] = {position, velocity, torque, temperature};
    update_fault_status(motor, dedupe_statuses(fault_list));
    return {position, velocity, torque, temperature};
}

std::vector<uint8_t> RobstrideBus::receive_read_frame() {
    auto result = receive(0.1);
    if (!result) {
        throw std::runtime_error("No response received for parameter read.");
    }

    auto [communication_type, extra_data, host_id, data] = *result;
    (void)extra_data;
    (void)host_id;
    if (communication_type != robstride_comm::READ_PARAMETER) {
        throw std::runtime_error("Invalid communication type: " + std::to_string(communication_type));
    }
    if (data.size() < 4) {
        throw std::runtime_error("Invalid Robstride read payload length: " + std::to_string(data.size()));
    }
    return std::vector<uint8_t>(data.begin() + 4, data.end());
}

std::optional<std::pair<int, std::vector<uint8_t>>> RobstrideBus::ping_by_id(int device_id, double timeout) {
    transmit(robstride_comm::GET_DEVICE_ID, host_id_, device_id);
    auto response = receive(timeout);
    if (!response) {
        return std::nullopt;
    }

    auto [communication_type, extra_data, host_id, data] = *response;
    (void)host_id;
    if (communication_type != robstride_comm::GET_DEVICE_ID) {
        throw std::runtime_error("Invalid communication type: " + std::to_string(communication_type));
    }
    return std::make_pair(extra_data, data);
}

std::optional<std::pair<int, std::vector<uint8_t>>> RobstrideBus::read_id(
    const std::string& motor,
    double timeout
) {
    return ping_by_id(require_motor(motor).id, timeout);
}

std::optional<std::pair<int, std::vector<uint8_t>>> RobstrideBus::write_id(
    const std::string& motor,
    int new_id
) {
    int current_id = require_motor(motor).id;
    transmit(robstride_comm::SET_DEVICE_ID, new_id, current_id);
    auto response = receive(0.1);
    if (!response) {
        return std::nullopt;
    }

    auto [communication_type, extra_data, host_id, data] = *response;
    (void)host_id;
    if (communication_type != robstride_comm::GET_DEVICE_ID) {
        throw std::runtime_error("Invalid communication type: " + std::to_string(communication_type));
    }
    motors_[motor].id = new_id;
    return std::make_pair(extra_data, data);
}

void RobstrideBus::enable(const std::string& motor) {
    int device_id = require_motor(motor).id;
    transmit(robstride_comm::ENABLE, host_id_, device_id);
    if (!is_receiver_running()) {
        receive_status_frame(motor);
    }
}

void RobstrideBus::disable(const std::string& motor) {
    int device_id = require_motor(motor).id;
    run_disable_with_retries([&]() {
        uint8_t data[8] = {};
        transmit(robstride_comm::DISABLE, host_id_, device_id, data, 8);
        if (!is_receiver_running()) {
            receive_status_frame(motor);
        }
    });
}

void RobstrideBus::clear_fault(const std::string& motor) {
    int device_id = require_motor(motor).id;
    run_disable_with_retries([&]() {
        clear_fault_status(motor);
        uint8_t data[8] = {};
        data[0] = 0x01;
        transmit(robstride_comm::DISABLE, host_id_, device_id, data, 8);
        if (!is_receiver_running()) {
            receive_status_frame(motor);
        }
    });
}

void RobstrideBus::disable_nowait(const std::string& motor, bool clear_fault) {
    int device_id = require_motor(motor).id;
    uint8_t data[8] = {};
    if (clear_fault) {
        clear_fault_status(motor);
        data[0] = 0x01;
    }
    transmit(robstride_comm::DISABLE, host_id_, device_id, data, 8);
}

void RobstrideBus::disable_with_clear(const std::string& motor) {
    clear_fault(motor);
}

RobstrideValue RobstrideBus::read(const std::string& motor, const RobstrideParameter& param) {
    int device_id = require_motor(motor).id;

    uint8_t data[8] = {};
    pack_le16(data, param.param_id);
    transmit(robstride_comm::READ_PARAMETER, host_id_, device_id, data, 8);
    auto response = receive_read_frame();

    switch (param.dtype) {
        case ParamDtype::UINT8:
            return static_cast<int>(response[0]);
        case ParamDtype::INT8:
            return static_cast<int>(static_cast<int8_t>(response[0]));
        case ParamDtype::UINT16:
            return static_cast<int>(unpack_le16(response.data()));
        case ParamDtype::INT16:
            return static_cast<int>(static_cast<int16_t>(unpack_le16(response.data())));
        case ParamDtype::UINT32:
            return static_cast<int>(unpack_le32(response.data()));
        case ParamDtype::INT32: {
            uint32_t raw = unpack_le32(response.data());
            int32_t value;
            std::memcpy(&value, &raw, 4);
            return static_cast<int>(value);
        }
        case ParamDtype::FLOAT32:
            return unpack_le_float(response.data());
    }
    throw std::runtime_error(std::string("Unsupported parameter type for ") + param.name);
}

void RobstrideBus::write(const std::string& motor, const RobstrideParameter& param, RobstrideValue value) {
    int device_id = require_motor(motor).id;

    uint8_t data[8] = {};
    pack_le16(data, param.param_id);

    int int_value = 0;
    float float_value = 0.0f;
    if (auto* int_ptr = std::get_if<int>(&value)) {
        int_value = *int_ptr;
        float_value = static_cast<float>(*int_ptr);
    } else {
        float_value = std::get<float>(value);
        int_value = static_cast<int>(float_value);
    }

    switch (param.dtype) {
        case ParamDtype::UINT8:
            data[4] = static_cast<uint8_t>(int_value);
            break;
        case ParamDtype::INT8:
            data[4] = static_cast<uint8_t>(static_cast<int8_t>(int_value));
            break;
        case ParamDtype::UINT16:
            pack_le16(data + 4, static_cast<uint16_t>(int_value));
            break;
        case ParamDtype::INT16:
            pack_le16(data + 4, static_cast<uint16_t>(static_cast<int16_t>(int_value)));
            break;
        case ParamDtype::UINT32:
            pack_le32(data + 4, static_cast<uint32_t>(int_value));
            break;
        case ParamDtype::INT32:
            pack_le32(data + 4, static_cast<uint32_t>(int_value));
            break;
        case ParamDtype::FLOAT32:
            pack_le_float(data + 4, float_value);
            break;
    }

    transmit(robstride_comm::WRITE_PARAMETER, host_id_, device_id, data, 8);
    receive_status_frame(motor);
}

void RobstrideBus::write_mit_frame(
    const std::string& motor,
    double position,
    double kp,
    double kd,
    double velocity,
    double torque
) {
    int device_id = require_motor(motor).id;
    const auto& model = require_model(motor);

    auto [processed_position, processed_velocity, processed_torque] =
        process_mit_control(motor, position, velocity, torque);

    double position_range = mit_position_table().at(model);
    double velocity_range = mit_velocity_table().at(model);
    double kp_range = mit_kp_table().at(model);
    double kd_range = mit_kd_table().at(model);
    double torque_range = mit_torque_table().at(model);

    processed_position = std::clamp(processed_position, -position_range, position_range);
    int position_u16 = static_cast<int>((processed_position / position_range + 1.0) * 0x7FFF);
    position_u16 = std::clamp(position_u16, 0, 0xFFFF);

    processed_velocity = std::clamp(processed_velocity, -velocity_range, velocity_range);
    int velocity_u16 = static_cast<int>((processed_velocity / velocity_range + 1.0) * 0x7FFF);
    velocity_u16 = std::clamp(velocity_u16, 0, 0xFFFF);

    kp = std::clamp(kp, 0.0, kp_range);
    int kp_u16 = static_cast<int>((kp / kp_range) * 0xFFFF);

    kd = std::clamp(kd, 0.0, kd_range);
    int kd_u16 = static_cast<int>((kd / kd_range) * 0xFFFF);

    int torque_u16 = static_cast<int>((processed_torque / torque_range + 1.0) * 0x7FFF);
    torque_u16 = std::clamp(torque_u16, 0, 0xFFFF);

    uint8_t data[8];
    pack_be16(data, static_cast<uint16_t>(position_u16));
    pack_be16(data + 2, static_cast<uint16_t>(velocity_u16));
    pack_be16(data + 4, static_cast<uint16_t>(kp_u16));
    pack_be16(data + 6, static_cast<uint16_t>(kd_u16));

    transmit(robstride_comm::OPERATION_CONTROL, torque_u16, device_id, data, 8);
}

void RobstrideBus::write_mit_kp_kd(const std::string& motor, double kp, double kd) {
    require_model(motor);
    mit_kp_[motor] = kp;
    mit_kd_[motor] = kd;
}

void RobstrideBus::write_mit_control(const std::string& motor, double position, double velocity, double torque) {
    double kp = 0.0;
    double kd = 0.0;
    auto kp_it = mit_kp_.find(motor);
    if (kp_it != mit_kp_.end()) {
        kp = kp_it->second;
    }
    auto kd_it = mit_kd_.find(motor);
    if (kd_it != mit_kd_.end()) {
        kd = kd_it->second;
    }

    write_mit_frame(motor, position, kp, kd, velocity, torque);
}

std::pair<double, double> RobstrideBus::read_mit_state(const std::string& motor) {
    if (is_receiver_running()) {
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

    auto frame = receive_status_frame(motor);
    return process_mit_state(motor, frame.position, frame.velocity);
}

std::map<std::string, std::vector<std::string>> RobstrideBus::read_fault_status() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return fault_status_;
}

std::vector<std::string> RobstrideBus::read_fault_status(const std::string& motor) const {
    require_motor(motor);
    std::lock_guard<std::mutex> lock(status_mutex_);
    auto it = fault_status_.find(motor);
    if (it != fault_status_.end()) {
        return it->second;
    }
    return {};
}

void RobstrideBus::clear_fault_status(const std::string& motor) {
    require_motor(motor);
    std::lock_guard<std::mutex> lock(status_mutex_);
    fault_status_.erase(motor);
}

void RobstrideBus::clear_fault_status() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    fault_status_.clear();
}

CanCounters RobstrideBus::transport_counters() const {
    if (!channel_handler_) {
        return {};
    }
    return channel_handler_->counters();
}

std::map<int, std::pair<int, std::vector<uint8_t>>> RobstrideBus::scan_channel(
    const std::string& channel,
    int start_id,
    int end_id,
    int bitrate
) {
    if (start_id > end_id) {
        throw std::invalid_argument("start_id must be less than or equal to end_id.");
    }

    RobstrideBus bus(channel, {}, std::nullopt, bitrate);
    bus.connect(false);

    std::map<int, std::pair<int, std::vector<uint8_t>>> device_ids;
    try {
        for (int device_id = start_id; device_id <= end_id; ++device_id) {
            if ((device_id - start_id) % 10 == 0) {
                std::cout << "Scanning " << device_id << "/" << end_id << "..." << std::endl;
            }
            auto response = bus.ping_by_id(device_id, 0.1);
            if (response) {
                std::cout << "Motor found for device_id=" << device_id << std::endl;
                device_ids[device_id] = *response;
            }
        }
    } catch (...) {
        bus.disconnect(false);
        throw;
    }

    bus.disconnect(false);
    return device_ids;
}

} // namespace actuator_control

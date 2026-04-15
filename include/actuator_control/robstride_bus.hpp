#pragma once

#include "actuator_control/actuator_bus.hpp"
#include "actuator_control/robstride_protocol.hpp"

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <tuple>
#include <variant>
#include <vector>

namespace actuator_control {

using RobstrideValue = std::variant<int, float>;

class RobstrideBus : public ActuatorBus {
public:
    RobstrideBus(const std::string& channel, const MotorMap& motors,
                 const std::optional<Calibration>& calibration = std::nullopt,
                 int bitrate = 1000000);
    ~RobstrideBus() override;

    int host_id() const { return host_id_; }
    void set_host_id(int host_id) { host_id_ = host_id; }

    static std::map<int, std::pair<int, std::vector<uint8_t>>> scan_channel(
        const std::string& channel,
        int start_id = 1,
        int end_id = 255,
        int bitrate = 1000000
    );

    void connect(bool handshake = true) override;
    void disconnect(bool disable_torque = true) override;
    void enable(const std::string& motor) override;
    void disable(const std::string& motor) override;
    void clear_fault(const std::string& motor);
    void disable_nowait(const std::string& motor, bool clear_fault = false);
    void disable_with_clear(const std::string& motor);

    void start_receiver();
    void stop_receiver();
    bool is_receiver_running() const;

    void transmit(int communication_type, int extra_data, int device_id,
                  const uint8_t* data = nullptr, size_t len = 0);
    std::optional<std::tuple<int, int, int, std::vector<uint8_t>>> receive(
        double timeout = -1.0
    );

    struct StatusFrame {
        double position;
        double velocity;
        double torque;
        double temperature;
    };

    StatusFrame receive_status_frame(const std::string& motor);
    std::vector<uint8_t> receive_read_frame();

    std::optional<std::pair<int, std::vector<uint8_t>>> ping_by_id(
        int device_id,
        double timeout = -1.0
    );
    std::optional<std::pair<int, std::vector<uint8_t>>> read_id(
        const std::string& motor,
        double timeout = -1.0
    );
    std::optional<std::pair<int, std::vector<uint8_t>>> write_id(
        const std::string& motor,
        int new_id
    );

    RobstrideValue read(const std::string& motor, const RobstrideParameter& param);
    void write(
        const std::string& motor,
        const RobstrideParameter& param,
        RobstrideValue value
    );

    void write_mit_kp_kd(const std::string& motor, double kp, double kd) override;
    void write_mit_control(
        const std::string& motor,
        double position,
        double velocity = 0.0,
        double torque = 0.0
    ) override;
    std::pair<double, double> read_mit_state(const std::string& motor) override;

    std::map<std::string, std::vector<std::string>> read_fault_status() const;
    std::vector<std::string> read_fault_status(const std::string& motor) const;
    void clear_fault_status(const std::string& motor);
    void clear_fault_status();
    CanCounters transport_counters() const;

private:
    const std::string& require_model(const std::string& motor);
    void update_fault_status(const std::string& motor, const std::vector<std::string>& statuses);
    int drain_rx_buffer(double timeout = 0.0, int max_frames = 256);
    void write_mit_frame(
        const std::string& motor,
        double position,
        double kp,
        double kd,
        double velocity = 0.0,
        double torque = 0.0
    );

    using FramePredicate = std::function<bool(int, int, int, const uint8_t*, size_t)>;
    std::optional<std::tuple<int, int, int, std::vector<uint8_t>>> receive_matching_frame(
        const FramePredicate& predicate,
        double timeout,
        const std::string& description
    );

    std::string motor_name_from_id(int motor_id) const;
    void run_message_receiver();
    void decode_status_into_cache(
        const std::string& motor,
        const std::string& model,
        int extra_data,
        const uint8_t* data,
        size_t len
    );

    int host_id_ = 0xFF;
    std::map<std::string, double> mit_kp_;
    std::map<std::string, double> mit_kd_;
    std::map<std::string, std::vector<std::string>> fault_status_;
    std::map<std::string, MotorStatus> motor_status_;
    mutable std::mutex status_mutex_;
    std::atomic<bool> killed_{false};
    std::unique_ptr<std::thread> receiver_thread_;
};

} // namespace actuator_control

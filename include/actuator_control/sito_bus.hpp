#pragma once

#include "actuator_control/actuator_bus.hpp"

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace actuator_control {

namespace sito_comm {
constexpr int RESET = 0x00;
constexpr int SELECT_MODE = 0x01;
constexpr int SET_MIT_CURRENT_VELOCITY_POSITION = 0x09;
constexpr int SET_MIT_KP_KD = 0x45;
constexpr int FEEDBACK_0 = 0xB0;
constexpr int FEEDBACK_1 = 0xB1;
constexpr int FEEDBACK_2 = 0xB2;
constexpr int FEEDBACK_3 = 0xB3;
} // namespace sito_comm

namespace sito_mode {
constexpr int POSITION = 0x08;
constexpr int MIT = 0x09;
} // namespace sito_mode

class SitoBus : public ActuatorBus {
public:
    SitoBus(const std::string& channel, const MotorMap& motors,
            const std::optional<Calibration>& calibration = std::nullopt,
            int bitrate = 1000000, double control_frequency = 50.0);
    ~SitoBus() override;

    void connect(bool handshake = true) override;
    void disconnect(bool disable_torque = true) override;
    void enable(const std::string& motor) override;
    void disable(const std::string& motor) override;

    void write_mit_kp_kd(const std::string& motor, double kp, double kd) override;
    void write_mit_control(
        const std::string& motor,
        double position,
        double velocity = 0.0,
        double torque = 0.0
    ) override;
    std::pair<double, double> read_mit_state(const std::string& motor) override;

private:
    struct SitoMotorConfig {
        double actuator_counts_per_rad;
        double gear_ratio;
        double torque_constant;
    };

    static const std::map<std::string, SitoMotorConfig>& motor_configs();
    const SitoMotorConfig& get_motor_config(const std::string& motor) const;
    uint32_t make_arbitration_id(const std::string& motor, int comm_type) const;
    std::string motor_name_from_id(int motor_id) const;

    void start_receiver_thread();
    void stop_receiver_thread();
    void run_message_receiver();

    int feedback_1_interval_;
    int feedback_2_interval_;
    int feedback_3_interval_ = 0;
    std::map<std::string, MotorStatus> motor_status_;
    mutable std::mutex status_mutex_;
    std::atomic<bool> killed_{false};
    std::unique_ptr<std::thread> receiver_thread_;
};

} // namespace actuator_control

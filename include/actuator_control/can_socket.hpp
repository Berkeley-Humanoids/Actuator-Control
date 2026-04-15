#pragma once

#include <atomic>
#include <cstdint>
#include <optional>
#include <string>

namespace actuator_control {

struct CanFrame {
    uint32_t arbitration_id = 0;
    bool is_extended_id = false;
    uint8_t dlc = 0;
    uint8_t data[8] = {};
};

struct CanCounters {
    uint64_t tx_frames = 0;
    uint64_t rx_frames = 0;
};

class CanSocket {
public:
    explicit CanSocket(const std::string& channel, int bitrate = 1000000);
    ~CanSocket();

    CanSocket(const CanSocket&) = delete;
    CanSocket& operator=(const CanSocket&) = delete;
    CanSocket(CanSocket&& other) noexcept;
    CanSocket& operator=(CanSocket&& other) noexcept;

    void send(const CanFrame& frame);
    std::optional<CanFrame> recv(double timeout_sec);
    void shutdown();
    bool is_open() const;
    int fd() const { return fd_; }
    CanCounters counters() const;

private:
    int fd_ = -1;
    std::string channel_;
    std::atomic<uint64_t> tx_frames_{0};
    std::atomic<uint64_t> rx_frames_{0};
};

} // namespace actuator_control

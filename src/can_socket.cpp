#include "actuator_control/can_socket.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <utility>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace actuator_control {

namespace {

constexpr int k_max_send_retries = 20;
constexpr auto k_send_retry_delay = std::chrono::milliseconds(1);

} // namespace

CanSocket::CanSocket(const std::string& channel, int /*bitrate*/) : channel_(channel) {
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        throw std::runtime_error(
            "Failed to create CAN socket: " + std::string(std::strerror(errno))
        );
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, channel.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error(
            "CAN interface '" + channel + "' not found: " + std::string(std::strerror(errno))
        );
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error(
            "Failed to bind CAN socket on '" + channel + "': " + std::string(std::strerror(errno))
        );
    }
}

CanSocket::~CanSocket() {
    shutdown();
}

CanSocket::CanSocket(CanSocket&& other) noexcept : fd_(other.fd_), channel_(std::move(other.channel_)) {
    other.fd_ = -1;
}

CanSocket& CanSocket::operator=(CanSocket&& other) noexcept {
    if (this != &other) {
        shutdown();
        fd_ = other.fd_;
        channel_ = std::move(other.channel_);
        other.fd_ = -1;
    }
    return *this;
}

void CanSocket::send(const CanFrame& frame) {
    if (fd_ < 0) {
        throw std::runtime_error("CAN socket is not open.");
    }

    struct can_frame raw_frame {};
    raw_frame.can_id = frame.arbitration_id;
    if (frame.is_extended_id) {
        raw_frame.can_id |= CAN_EFF_FLAG;
    }
    raw_frame.can_dlc = frame.dlc;
    std::memcpy(raw_frame.data, frame.data, frame.dlc);

    for (int attempt = 0; attempt <= k_max_send_retries; ++attempt) {
        auto written = ::write(fd_, &raw_frame, sizeof(raw_frame));
        if (written == sizeof(raw_frame)) {
            tx_frames_.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        int error = errno;
        bool can_retry = (written < 0) && (error == ENOBUFS || error == EINTR);
        if (can_retry && attempt < k_max_send_retries) {
            std::this_thread::sleep_for(k_send_retry_delay);
            continue;
        }

        if (written < 0) {
            throw std::runtime_error(
                "Failed to send CAN frame on '" + channel_ + "': " + std::string(std::strerror(error))
            );
        }

        throw std::runtime_error(
            "Short CAN write on '" + channel_ + "': wrote " + std::to_string(written) +
            " bytes, expected " + std::to_string(sizeof(raw_frame))
        );
    }
}

std::optional<CanFrame> CanSocket::recv(double timeout_sec) {
    if (fd_ < 0) {
        return std::nullopt;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);

    struct timeval timeout {};
    struct timeval* timeout_ptr = nullptr;
    if (timeout_sec >= 0.0) {
        auto seconds = static_cast<long>(timeout_sec);
        auto microseconds = static_cast<long>((timeout_sec - static_cast<double>(seconds)) * 1e6);
        timeout.tv_sec = seconds;
        timeout.tv_usec = microseconds;
        timeout_ptr = &timeout;
    }

    int result = ::select(fd_ + 1, &read_fds, nullptr, nullptr, timeout_ptr);
    if (result < 0) {
        if (errno == EBADF || errno == EINTR) {
            return std::nullopt;
        }
        throw std::runtime_error("select() failed: " + std::string(std::strerror(errno)));
    }
    if (result == 0) {
        return std::nullopt;
    }

    struct can_frame raw_frame {};
    auto bytes_read = ::read(fd_, &raw_frame, sizeof(raw_frame));
    if (bytes_read < 0) {
        if (errno == EBADF) {
            return std::nullopt;
        }
        throw std::runtime_error("Failed to read CAN frame: " + std::string(std::strerror(errno)));
    }
    if (bytes_read < static_cast<ssize_t>(sizeof(raw_frame))) {
        return std::nullopt;
    }

    CanFrame frame;
    frame.is_extended_id = (raw_frame.can_id & CAN_EFF_FLAG) != 0;
    frame.arbitration_id = raw_frame.can_id & (frame.is_extended_id ? CAN_EFF_MASK : CAN_SFF_MASK);
    frame.dlc = raw_frame.can_dlc;
    std::memcpy(frame.data, raw_frame.data, raw_frame.can_dlc);
    rx_frames_.fetch_add(1, std::memory_order_relaxed);
    return frame;
}

void CanSocket::shutdown() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool CanSocket::is_open() const {
    return fd_ >= 0;
}

CanCounters CanSocket::counters() const {
    return {
        tx_frames_.load(std::memory_order_relaxed),
        rx_frames_.load(std::memory_order_relaxed),
    };
}

} // namespace actuator_control

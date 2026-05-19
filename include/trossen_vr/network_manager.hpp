#pragma once

#include "trossen_vr/vr_types.hpp"
#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>

namespace trossen_vr {

struct ReceiverConfig {
    uint16_t port = 9000;
    size_t buffer_size = 2048;
};

// Thread-safe UDP receiver that always holds the newest VR frame.
// Runs a background thread that drains the socket and keeps only the latest packet.
class UDPReceiver {
public:
    explicit UDPReceiver(const ReceiverConfig& config = {});
    ~UDPReceiver();

    UDPReceiver(const UDPReceiver&) = delete;
    UDPReceiver& operator=(const UDPReceiver&) = delete;

    // Start the background receive thread.
    void start();

    // Stop the background thread and close the socket.
    void stop();

    // Get the latest frame (thread-safe). Returns nullopt if no frame available yet.
    std::optional<VRFrame> latest_frame() const;

    // Check if the receiver thread is running.
    bool is_running() const noexcept;

private:
    void run();

    int sockfd_ = -1;
    size_t buffer_size_;
    char* buffer_ = nullptr;

    std::thread thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex frame_mutex_;
    std::optional<VRFrame> latest_frame_;
};

} // namespace trossen_vr

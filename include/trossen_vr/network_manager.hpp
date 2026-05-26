#ifndef TROSSEN_VR_NETWORK_MANAGER_HPP
#define TROSSEN_VR_NETWORK_MANAGER_HPP

#include <cstddef>
#include <cstdint>

#include <atomic>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "trossen_vr/vr_types.hpp"

namespace trossen_vr {

/// @brief UDP receiver configuration
struct ReceiverConfig {
    /// @brief UDP port to listen on (default: 9000)
    uint16_t port = 9000;

    /// @brief UDP receive buffer size in bytes (default: 2048)
    size_t buffer_size = 2048;
};

/**
 * @brief Network manager for VR controller data
 *
 * Runs a background thread that continuously receives UDP packets and maintains
 * the latest VR frame. Thread-safe access via latest_frame().
 */
class NetworkManager {
public:
    /**
     * @brief Construct network manager
     *
     * @param config Receiver configuration (port and buffer size)
     */
    explicit NetworkManager(const ReceiverConfig& config = {});

    /// @brief Destroy manager and stop background thread
    ~NetworkManager();

    NetworkManager(const NetworkManager&) = delete;
    NetworkManager& operator=(const NetworkManager&) = delete;

    /**
     * @brief Start receiving VR data on background thread
     *
     * Binds to the configured UDP port and starts packet reception.
     *
     * @throws std::runtime_error if socket creation or binding fails
     */
    void start();

    /**
     * @brief Stop receiving and shutdown background thread
     *
     * Safe to call multiple times.
     */
    void stop();

    /**
     * @brief Get the most recent VR frame
     *
     * Thread-safe. Returns empty optional if no frame received yet.
     *
     * @return Latest VR frame, or empty if none available
     */
    std::optional<VRFrame> latest_frame() const;

    /**
     * @brief Check if receiver thread is running
     *
     * @return true if background thread is active
     */
    bool is_running() const noexcept;

private:
    void run();

    int sockfd_ = -1;
    std::vector<char> buffer_;

    std::thread thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex frame_mutex_;
    std::optional<VRFrame> latest_frame_;
};

} // namespace trossen_vr

#endif // TROSSEN_VR_NETWORK_MANAGER_HPP

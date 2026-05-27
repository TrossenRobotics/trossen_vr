#ifndef TROSSEN_VR_NETWORK_MANAGER_HPP
#define TROSSEN_VR_NETWORK_MANAGER_HPP

#include <netinet/in.h>
#include <sys/socket.h>

#include <cstddef>
#include <cstdint>

#include <atomic>
#include <chrono>
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

    /// @brief Connection timeout in seconds (default: 2.0)
    /// If no message received for this duration, status becomes Disconnected
    double timeout_seconds = 2.0;

    /// @brief Minimum expected message frequency in Hz (default: 30.0)
    /// If frequency drops below this, status becomes Degraded
    double min_frequency_hz = 30.0;

    /// @brief Window size for packet loss tracking (default: 100)
    size_t loss_window = 100;

    /// @brief UDP port to send ACK packets back to the VR app (default: 9001)
    /// The VR app listens on this port for acknowledgement packets.
    uint16_t ack_port = 9001;
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

    /**
     * @brief Get current connection status
     *
     * Thread-safe. Status is automatically updated based on message
     * reception timing and frequency.
     *
     * @return Current connection status
     */
    ConnectionStatus get_connection_status() const noexcept;

    /**
     * @brief Get current message reception frequency in Hz
     *
     * Thread-safe. Calculated over a rolling window.
     *
     * @return Frequency in Hz, or 0.0 if no messages received yet
     */
    double get_message_frequency() const noexcept;

    /**
     * @brief Get packet loss rate over recent window
     *
     * Thread-safe. Based on expected vs received message count.
     *
     * @return Loss rate between 0.0 (no loss) and 1.0 (100% loss)
     */
    double get_packet_loss_rate() const noexcept;

private:
    void run();
    void send_ack();
    void update_connection_status();

    ReceiverConfig config_;
    int sockfd_ = -1;
    std::vector<char> buffer_;
    sockaddr_in client_addr_{};
    socklen_t client_addr_len_ = 0;  // 0 means no client address received yet

    std::thread thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex frame_mutex_;
    std::optional<VRFrame> latest_frame_;

    // Connection tracking
    mutable std::mutex status_mutex_;
    std::atomic<ConnectionStatus> connection_status_{ConnectionStatus::Disconnected};
    std::chrono::steady_clock::time_point last_received_time_;
    std::chrono::steady_clock::time_point first_received_time_;
    size_t total_messages_received_ = 0;
    size_t expected_messages_ = 0;
    size_t lost_messages_ = 0;
};

} // namespace trossen_vr

#endif // TROSSEN_VR_NETWORK_MANAGER_HPP

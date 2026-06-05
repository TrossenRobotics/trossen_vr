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

#include <nlohmann/json.hpp>

#include "trossen_vr/vr_types.hpp"
#include "trossen_vr/vr_conversions.hpp"

namespace trossen_vr {

/**
 * @brief Network receiver configuration
 *
 * Controls network behavior, connection monitoring, and ACK transmission.
 */
struct ReceiverConfig {
    /// @brief Port to listen on (default: 9000)
    uint16_t port = 9000;

    /// @brief Receive buffer size in bytes (default: 2048)
    size_t buffer_size = 2048;

    /// @brief Connection timeout in seconds (default: 2.0)
    double timeout_seconds = 2.0;

    /// @brief Minimum expected message frequency in Hz (default: 30.0)
    double min_frequency_hz = 30.0;

    /// @brief Port to send ACK packets to VR app (default: 9001)
    uint16_t ack_port = 9001;

    /// @brief Grip trigger threshold for deadman switch (default: 0.9)
    /// Controllers are only considered tracked when hand_trigger >= this value
    double grip_threshold = 0.9;
};

/**
 * @brief Network manager for VR controller data
 *
 * Runs a background thread that continuously receives packets and maintains
 * the latest VR frame. Thread-safe access via latest_frame().
 */
class NetworkManager {
public:
    /**
     * @brief Construct network manager
     *
     * @param config Receiver configuration
     */
    explicit NetworkManager(const ReceiverConfig& config = {});

    /**
     * @brief Destroy manager and stop background thread
     */
    ~NetworkManager();

    // Prevent copying to ensure unique ownership of the underlying network socket
    NetworkManager(const NetworkManager&) = delete;
    NetworkManager& operator=(const NetworkManager&) = delete;

    /**
     * @brief Start receiving VR data on background thread
     *
     * Binds to the configured port and starts packet reception.
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
     * Status is updated based on message
     * reception timing and frequency.
     *
     * @return Current connection status
     */
    ConnectionStatus get_connection_status() const noexcept;

    /**
     * @brief Get current message reception frequency in Hz
     *
     * Calculated over a rolling window.
     *
     * @return Frequency in Hz, or 0.0 if no messages received yet
     */
    double get_message_frequency() const noexcept;

private:
    /**
     * @brief Main receiver thread loop
     *
     * Continuously polls for packets, parses VR frames,
     * updates connection status, and sends ACK packets.
     */
    void run();

    /**
     * @brief Send ACK packet to VR client
     *
     * Sends JSON ACK with current frequency and packet loss statistics
     * to the client address on the configured ACK port.
     */
    void send_ack();

    /**
     * @brief Update connection status based on message timing
     *
     * Determines status (Disconnected/Connecting/Connected/Degraded)
     * based on message count, timeout, and frequency thresholds.
     */
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
    std::chrono::steady_clock::time_point freq_window_start_;
    size_t freq_window_count_ = 0;
    double message_frequency_hz_ = 0.0;
    size_t total_messages_received_ = 0;
};

} // namespace trossen_vr

#endif // TROSSEN_VR_NETWORK_MANAGER_HPP

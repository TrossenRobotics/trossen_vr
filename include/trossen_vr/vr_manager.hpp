#ifndef TROSSEN_VR__INCLUDE__VR_MANAGER_HPP_
#define TROSSEN_VR__INCLUDE__VR_MANAGER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <websocketpp/config/asio_no_tls.hpp> 
#include <websocketpp/server.hpp>

#include "trossen_vr/vr_types.hpp"

namespace trossen_vr {

class Teleop;
class VRManager;

/**
 * @class WebsocketServerClient
 * @brief WebSocket server implementation for receiving VR input from remote clients.
 *
 * This class implements a WebSocket server that listens for incoming connections
 * from VR rigs and receives JSON-encoded VR state frames containing controller
 * poses and button data.
 *
 * Thread-safety:
 * - Connection state is protected by conn_mutex_
 * - Message queue is protected by msg_mutex_
 * - Can safely be called from multiple threads
 */
class WebsocketServerClient {
public:
    /**
     * @brief Construct a WebSocket server client with the given port.
     * @param port Server port to listen on.
     */
    explicit WebsocketServerClient(uint16_t port);

    /**
     * @brief Start the WebSocket server and begin listening for connections.
     * 
     * Initializes the ASIO I/O context, sets up connection/message handlers,
     * and starts listening on the configured port. Launches a background thread
     * to run the I/O loop.
     *
     * @throws std::runtime_error if the server fails to listen on the port.
     */
    void connect();

    /**
     * @brief Stop the WebSocket server and close all connections.
     * 
     * Stops the ASIO I/O context and joins the background I/O thread.
     */
    void disconnect();

    /**
     * @brief Check if a VR client is currently connected.
     * @return true if a client connection is established, false otherwise.
     */
    bool is_connected() const;

    /**
     * @brief Read a VR input frame with timeout.
     * 
     * Polls for incoming messages and parses the most recent JSON frame
     * containing VR controller poses and button states. Blocks up to the
     * specified timeout waiting for a frame.
     *
     * @param timeout Maximum duration to wait for a frame.
     * @return A VRState frame if received and parsed successfully, std::nullopt otherwise.
     */
    std::optional<VRState> read_frame(std::chrono::milliseconds timeout);

    /**
     * @brief Send a message payload to the connected VR client.
     * 
     * Transmits a string payload over the WebSocket connection if a client
     * is currently connected.
     *
     * @param payload String data to send to the VR client.
     */
    void send(const std::string& payload);

private:
    uint16_t extract_port(const std::string& uri);

    uint16_t port_ {4582};
    websocketpp::server<websocketpp::config::asio> s_;
    websocketpp::connection_hdl connection_;
    mutable std::mutex conn_mutex_;
    mutable std::mutex msg_mutex_;
    std::string last_message_;
    std::atomic<bool> connected_;
    std::thread thread_;
    uint64_t sequence_;
};

/**
 * @class VRManager
 * @brief Manages VR device communication and state synchronization over WebSocket connection.
 *
 * VRManager handles the lifecycle of a WebSocket-based VR input system, managing
 * connection state, receiving VR input frames (pose and button data), and polling
 * the Teleop for outbound updates to send back to the VR rig. It runs a dedicated
 * I/O thread for asynchronous communication and provides thread-safe access to
 * the latest VR state.
 *
 * Thread-safety:
 * - All public methods are thread-safe
 * - Internal state is protected by lifecycle_mutex_ and data_mutex_
 * - Atomic variables used for flags to minimize lock contention
 */
class VRManager {
public:
    /**
     * @struct Config
     * @brief Configuration parameters for VRManager.
     *
     * server_port: Port to listen for incoming VR rig connections.
     * reconnect_delay: How long to wait between connection retries.
     * read_timeout: Max duration to block waiting for a VR frame.
     */
    struct Config {
        uint16_t server_port {4582}; 
        std::chrono::milliseconds reconnect_delay{1000};
        std::chrono::milliseconds read_timeout{50};
    };

    /**
     * @brief Construct VRManager using the default WebSocket client implementation (server-mode).
     *
     * @param config Connection and operating parameters.
     */
    explicit VRManager(Config config);

    /**
     * @brief Destructor shuts down I/O thread and cleans up resources.
     */
    ~VRManager();

    /**
     * @brief Start the background I/O communication thread.
     */
    void start();

    /**
     * @brief Request the I/O thread to stop and wait for shutdown.
     */
    void stop();

    /**
     * @brief Restart the VRManager by stopping and starting the I/O thread.
     */
    void restart();

    /**
     * @brief Check whether the I/O thread is actively running.
     * @return true if active, false otherwise.
     */
    bool is_active() const noexcept;

    /**
     * @brief Check whether the WebSocket connection is established.
     * @return true if connected, false otherwise.
     */
    bool is_connected() const noexcept;

    /**
     * @brief Retrieve the latest controller pose.
     *
     * @return The most recently received VRPose, or std::nullopt if none available.
     */
    std::optional<VRPose> get_pose() const;

    /**
     * @brief Retrieve the latest button value for a specific button.
     *
     * @param button Name of the button to query.
     * @return Value if available, std::nullopt otherwise.
     */
    std::optional<VRButtonValue> get_button_state(const std::string& button) const;

    /**
     * @brief Access the most recent full VR input frame.
     *
     * @return Latest VRState if available, otherwise std::nullopt.
     */
    std::optional<VRState> get_latest_frame() const;

    /**
     * @brief Poll the Teleop instance for messages to send to robot.
     *
     * If Teleop has posted an update, VRManager will transmit it to the robot. 
     * Called regularly inside the I/O loop.
     *
     * @param teleop Teleop instance being polled.
     */  
    void poll_teleop(Teleop& teleop);


    /**
     * @brief Poll the the vr_manager for a vrstate frame manually (without starting the io thread).
     *
     * @param manual Indicates manual polling mode.
     */  
    std::optional<VRState> get_current_state();

private:
    /**
     * @brief Main loop executed by the I/O thread.
     *
     * Handles reconnection logic, frame reading, and outbound Teleop updates.
     */
    void run();
    /**
     * @brief Process a newly received VR input frame.
     *
     * Stores the frame and updates tracking metadata.
     *
     * @param frame The frame to process.
     */
    void handle_frame(VRState&& frame);

    /**
     * @brief Mark manager state as disconnected.
     *
     * Updates internal flags to indicate loss of connection to the VR rig.
     */
    void mark_disconnected();

    /**
     * @brief Ensure a WebSocket client exists, creating one if needed.
     */
    void ensure_client();

    Config config_;
    std::unique_ptr<WebsocketServerClient> client_connection_;
    mutable std::mutex client_mutex_;

    std::thread io_thread_;
    mutable std::mutex lifecycle_mutex_;
    std::atomic<bool> running_{false};
    std::atomic<bool> manual_mode_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> connected_{false};

    mutable std::mutex data_mutex_;
    std::optional<VRState> latest_frame_;
    std::uint64_t next_sequence_id_{0};
    std::uint64_t last_dispatched_sequence_{0};
};

} // namespace trossen_vr

#endif // TROSSEN_VR__INCLUDE__VR_MANAGER_HPP_

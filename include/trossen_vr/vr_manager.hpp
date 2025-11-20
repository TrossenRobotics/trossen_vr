#ifndef TROSSEN_VR__INCLUDE__VR_MANAGER_HPP_
#define TROSSEN_VR__INCLUDE__VR_MANAGER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <queue>


#include "trossen_vr/vr_types.hpp"

namespace trossen_vr {

class Teleop;

/**
 * @class IWebsocketClient
 * @brief Interface abstraction for WebSocket transport used by VRManager.
 *
 * This interface allows VRManager to operate with any WebSocket backend.
 * Implementations must provide connection management, message sending,
 * and timed frame reception. It now supports server-mode operation, 
 * where VRManager listens for an incoming connection from a VR rig.
 */
class IWebsocketClient {
public:
    virtual ~IWebsocketClient() = default;

    /**
     * @brief Establish the WebSocket connection or start listening (server mode).
     */
    virtual void connect() = 0;

    /**
     * @brief Close the WebSocket connection.
     */
    virtual void disconnect() = 0;

    /**
     * @brief Check whether the client is currently connected.
     * @return true if connected, false otherwise.
     */
    virtual bool is_connected() const = 0;

    /**
     * @brief Read a VR input frame with timeout.
     *
     * @param timeout Maximum duration to block for a frame.
     * @return A frame if received, or std::nullopt if timeout or no data.
     */
    virtual std::optional<VRState> read_frame(std::chrono::milliseconds timeout) = 0;

    /**
     * @brief Send a payload string over the WebSocket connection.
     *
     * @param payload Serialized data to transmit.
     */
    virtual void send(const std::string& payload) = 0;
};

/**
 * @class VRManager
 * @brief Manages VR device communication and state synchronization over WebSocket.
 *
 * VRManager handles the lifecycle of a WebSocket-based VR input system.
 * It now operates as a **server**, listening for incoming connections
 * from a VR rig, receiving controller pose and button data, and sending
 * outbound Teleop updates.
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
        uint16_t server_port = 5432; 
        std::chrono::milliseconds reconnect_delay{1000};
        std::chrono::milliseconds read_timeout{50};
    };

    // Factory type for supplying custom WebSocket client implementations.
    using Client = std::function<std::unique_ptr<IWebsocketClient>(const Config&)>;

    /**
     * @brief Construct VRManager using the default WebSocket client implementation (server-mode).
     *
     * @param config Connection and operating parameters.
     */
    explicit VRManager(Config config);

    /**
     * @brief Construct VRManager with a user-provided WebSocket client factory.
     *
     * @param config VR manager configuration.
     * @param client Custom client factory.
     */
    VRManager(Config config, Client client);

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
     * 
     *
     * @param manual Indicates manual polling mode.
     */  
    std::optional<VRState> poll_manual();

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

    /**
     * @brief Create the default WebSocket server implementation.
     *
     * @param config VR manager configuration.
     * @return A fully constructed WebSocket server client.
     */
    static std::unique_ptr<IWebsocketClient> create_default_client(const Config& config);

    Config config_;
    std::unique_ptr<IWebsocketClient> client_connection_;
    Client client_;
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

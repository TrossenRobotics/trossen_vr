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

#include "trossen_vr/vr_types.hpp"

namespace trossen_vr {

class Teleop;

/**
 * @class IWebsocketClient
 * @brief Interface abstraction for WebSocket transport used by VRManager.
 *
 * This interface allows VRManager to operate with any WebSocket backend.
 * Implementations must provide connection management, message sending,
 * and timed frame reception.
 */
class IWebsocketClient {
public:
    virtual ~IWebsocketClient() = default;

    /**
     * @brief Establish the WebSocket connection.
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
  struct Config {
    std::string endpoint_uri;
    std::chrono::milliseconds reconnect_delay{1000};
    std::chrono::milliseconds read_timeout{50};
  };

  // Factory type for supplying custom WebSocket client implementations.
  using Client = std::function<std::unique_ptr<IWebsocketClient>(const Config&)>;

  /**
    * @brief Construct VRManager using the default WebSocket client implementation.
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
    * @brief Start the background I/O communication thread.
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
    * @return Latest VRInputFrame if available, otherwise std::nullopt.
    */
  std::optional<VRState> get_latest_frame() const;

  /**
    * @brief Poll the Teleop instance for outbound messages to send to VR.
    *
    * If Teleop has posted an update, VRManager will transmit it using the
    * WebSocket client. Called regularly inside the I/O loop.
    *
    * @param teleop Teleop instance being polled.
    */  
  void poll_teleop(const Teleop& teleop);

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
    * @brief Create the default WebSocket client implementation.
    *
    * @param config VR manager configuration.
    * @return A fully constructed WebSocket client.
    */
  static std::unique_ptr<IWebsocketClient> create_default_client(const Config& config);

  Config config_;
  std::unique_ptr<IWebsocketClient> client_connection_;
  Client client_;
  mutable std::mutex client_mutex_;

  std::thread io_thread_;
  mutable std::mutex lifecycle_mutex_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> connected_{false};

  mutable std::mutex data_mutex_;
  std::optional<VRState> latest_frame_;
  std::uint64_t next_sequence_id_{0};
  std::uint64_t last_dispatched_sequence_{0};
};

}

#endif // TROSSEN_VR__INCLUDE__VR_MANAGER_HPP_

#ifndef Trossen_vr__include__vr_manager_HPP_
#define Trossen_vr__include__vr_manager_HPP_

#include "vr_types.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace trossen_vr {

class Teleop;

// Interface for websocket implementation.
class IWebsocketClient {
public:
    virtual ~IWebsocketClient() = default;
    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual bool is_connected() const = 0;
    virtual std::optional<VRInputFrame> read_frame(std::chrono::milliseconds timeout) = 0;
    virtual void send(const std::string& payload) = 0;
};

class VRManager {
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

public:
    struct Config {
        std::string endpoint_uri;
        std::chrono::milliseconds reconnect_delay{1000};
        std::chrono::milliseconds read_timeout{50};
    };

    using Client = std::function<std::unique_ptr<IWebsocketClient>(const Config&)>;

    explicit VRManager(Config config);
    VRManager(Config config, Client client);
    ~VRManager();

    // start up and shut down the IO thread.
    void start();
    void stop();
    void restart();
    bool is_active() const noexcept;
    bool is_connected() const noexcept;

    // Get the latest frame contents.
    std::optional<VRPose> get_pose() const;
    std::optional<VRButtonValue> get_button_state(const std::string& button) const;
    std::optional<VRInputFrame> get_latest_frame() const;

    // Poll teleop for any outbound updates (VRManager is responsible for sending them).
    void poll_teleop(const Teleop& teleop);

private:
    void run();
    void handle_frame(VRInputFrame&& frame);
    void mark_disconnected();
    void ensure_client();
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
    std::optional<VRInputFrame> latest_frame_;
    std::uint64_t next_sequence_id_ = 0;
    std::uint64_t last_dispatched_sequence_ = 0;
};

}

#endif // Trossen_vr__include__vr_manager_HPP_
#pragma once

#include "vr_types.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace trossen::meta_vr {

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

// Manages connection and sends out VR frames to Teleop.
class VRManager {
public:
    struct Config {
        std::string endpoint_uri;
        std::chrono::milliseconds reconnect_delay{1000};
        std::chrono::milliseconds read_timeout{50};
    };

    VRManager(Config config, std::unique_ptr<IWebsocketClient> client);
    ~VRManager();

    // start up and shut down the IO thread.
    void start();
    void stop();
    void restart();
    bool is_active() const noexcept;
    bool is_connected() const noexcept;

    // Get the latest frame contents.
    std::optional<VRPose> get_pose() const;
    std::optional<VRButtonState> get_button_state(const std::string& button) const;
    std::optional<VRInputFrame> get_latest_frame() const;

    // Deliver new frame data.
    void send_updates(Teleop& teleop);

private:
    void run();
    void handle_frame(VRInputFrame&& frame);
    void mark_disconnected();

    Config config_;
    std::unique_ptr<IWebsocketClient> client_;

    std::thread io_thread_;
    mutable std::mutex lifecycle_mutex_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> connected_{false};

    mutable std::mutex data_mutex_;
    std::optional<VRInputFrame> latest_frame_;
    std::uint64_t next_sequence_id_ = 0;
    std::uint64_t last_dispatched_sequence_ = 0;
    bool last_teleop_enabled_state_ = false;
};

}

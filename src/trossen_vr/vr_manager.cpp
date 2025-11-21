#include "trossen_vr/vr_manager.hpp"
#include "trossen_vr/teleop.hpp"

#include <thread>
#include <chrono>
#include <iostream>
#include <mutex>
#include <atomic>

#include <nlohmann/json.hpp>
#include <websocketpp/config/asio_no_tls.hpp> 
#include <websocketpp/server.hpp>


using json = nlohmann::json;

namespace trossen_vr {

class WebsocketServerClient : public IWebsocketClient {
public:
    explicit WebsocketServerClient(const VRManager::Config& config)
        : port_(config.server_port), connected_(false), sequence_(0) {}


    void connect() override {
        try {
            s_.init_asio();

            s_.set_open_handler([this](websocketpp::connection_hdl hdl) {
                std::lock_guard<std::mutex> lock(conn_mutex_);
                connection_ = hdl;
                connected_ = true;
                std::cout << "[VRManager] VR rig connected\n";
            });

            s_.set_close_handler([this](websocketpp::connection_hdl) {
                std::lock_guard<std::mutex> lock(conn_mutex_);
                connected_ = false;
                connection_.reset();
                std::cout << "[VRManager] VR rig disconnected\n";
            });

            s_.set_message_handler([this](websocketpp::connection_hdl, websocketpp::server<websocketpp::config::asio>::message_ptr msg) {
                std::lock_guard<std::mutex> lock(msg_mutex_);
                last_message_ = msg->get_payload();


            });

            websocketpp::lib::error_code ec;
            s_.listen(port_, ec);
            if (ec) {
                std::cerr << "[VRManager] Failed to listen on port " << port_ << ": " << ec.message() << "\n";
                return;
            }

            s_.start_accept();
            thread_ = std::thread([this]() { s_.run(); });

            std::cout << "[VRManager] Server listening on port " << port_ << "\n";
        } catch (const std::exception& e) {
            std::cerr << "[VRManager] WebSocket server exception: " << e.what() << "\n";
        }
    }

    void disconnect() override {
        s_.stop();
        if (thread_.joinable()) thread_.join();
        connected_ = false;
    }

    bool is_connected() const override {
        return connected_;
    }

    std::optional<VRState> read_frame(std::chrono::milliseconds timeout) override {
        auto start = std::chrono::steady_clock::now();
        while ((std::chrono::steady_clock::now() - start) < timeout) {
            std::string msg;
            {
                std::lock_guard<std::mutex> lock(msg_mutex_);
                if (!last_message_.empty()) {
                    msg = last_message_; // copy, do NOT move
                }
            }

            if (!msg.empty()) {
                try {
                    json j = json::parse(msg);
                    VRState frame;

                    if (j.contains("left_pose")) {
                        auto& lp = j["left_pose"];
                        frame.left_pose = VRPose{
                            {lp["position"][0], lp["position"][1], lp["position"][2]},
                            {lp["rotation"][0], lp["rotation"][1], lp["rotation"][2]}
                        };
                    }

                    if (j.contains("right_pose")) {
                        auto& rp = j["right_pose"];
                        frame.right_pose = VRPose{
                            {rp["position"][0], rp["position"][1], rp["position"][2]},
                            {rp["rotation"][0], rp["rotation"][1], rp["rotation"][2]}
                        };
                    }

                    if (j.contains("buttons")) {
                        for (auto it = j["buttons"].begin(); it != j["buttons"].end(); ++it) {
                            if (it.value().is_boolean()) frame.buttons[it.key()] = it.value().get<bool>();
                            else if (it.value().is_number_float()) frame.buttons[it.key()] = it.value().get<double>();
                        }
                    }

                    frame.timestamp = std::chrono::steady_clock::now();
                    frame.sequence = j.value("sequence", ++sequence_);

                    return frame;
                } catch (const std::exception& e) {
                    std::cerr << "[VRManager] JSON parse error: " << e.what() << "\n";
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return std::nullopt;
    }


    void send(const std::string& payload) override {
        std::lock_guard<std::mutex> lock(conn_mutex_);
        if (connected_ && connection_.lock()) {
            websocketpp::lib::error_code ec;
            s_.send(connection_, payload, websocketpp::frame::opcode::text, ec);
            if (ec) std::cerr << "[VRManager] WebSocket send error: " << ec.message() << "\n";
        }
    }

private:
    uint16_t port_;
    websocketpp::server<websocketpp::config::asio> s_;
    std::weak_ptr<void> connection_;
    mutable std::mutex conn_mutex_;
    mutable std::mutex msg_mutex_;
    std::string last_message_;
    std::atomic<bool> connected_;
    std::thread thread_;
    uint64_t sequence_;

    uint16_t extract_port(const std::string& uri) {
        // Parse "ws://0.0.0.0:5432" → 5432
        auto colon_pos = uri.find_last_of(':');
        if (colon_pos == std::string::npos) return 5432;
        return static_cast<uint16_t>(std::stoi(uri.substr(colon_pos + 1)));
    }
};



VRManager::VRManager(Config config)
    : config_(config) {
    client_ = [this](const Config& cfg) { return create_default_client(cfg); };
    ensure_client();
}

VRManager::VRManager(Config config, Client client)
    : config_(config), client_(client) {
    ensure_client();
}

VRManager::~VRManager() {
    stop();
}

void VRManager::start() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (!running_) {
        stop_requested_ = false;
        running_ = true;
        io_thread_ = std::thread(&VRManager::run, this);
    }
}


void VRManager::stop() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    stop_requested_ = true;
    running_ = false;
    if (io_thread_.joinable()) io_thread_.join();
}

bool VRManager::is_active() const noexcept { return running_; }
bool VRManager::is_connected() const noexcept { return connected_; }

std::optional<VRState> VRManager::get_latest_frame() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_frame_;
}

std::optional<VRPose> VRManager::get_pose() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!latest_frame_) return std::nullopt;
    if (latest_frame_->right_pose) return latest_frame_->right_pose;
    if (latest_frame_->left_pose) return latest_frame_->left_pose;
    return std::nullopt;
}

std::optional<VRButtonValue> VRManager::get_button_state(const std::string& button) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!latest_frame_) return std::nullopt;
    auto it = latest_frame_->buttons.find(button);
    if (it != latest_frame_->buttons.end()) return it->second;
    return std::nullopt;
}

void VRManager::poll_teleop(Teleop& teleop) {
    auto frame_opt = get_latest_frame();
    if (!frame_opt) return;

    // if (frame_opt->left_pose) {
    //     teleop.handle_pose(*frame_opt->left_pose, "left");
    // }
    if (frame_opt->right_pose) {
        teleop.handle_pose(*frame_opt->right_pose, "right");
    }

    // Handle buttons
    teleop.evaluate_button_states(frame_opt->buttons);
}

std::optional<VRState> VRManager::poll_manual() {
    auto frame_opt = get_latest_frame();

    return frame_opt;

}


void VRManager::ensure_client() {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (!client_connection_) {
        client_connection_ = client_(config_);
        client_connection_->connect();
        connected_ = client_connection_->is_connected();
    }
}

std::unique_ptr<IWebsocketClient> VRManager::create_default_client(const Config& config) {
    return std::make_unique<WebsocketServerClient>(config);
}

void VRManager::handle_frame(VRState&& frame) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_frame_ = std::move(frame);
}

void VRManager::mark_disconnected() {
    connected_ = false;
}

void VRManager::run() {
    while (!stop_requested_) {
        if (!client_connection_ || !client_connection_->is_connected()) {
            mark_disconnected();
            std::this_thread::sleep_for(config_.reconnect_delay);
            ensure_client();
            continue;
        }

        auto frame = client_connection_->read_frame(config_.read_timeout);
        if (frame) {
            handle_frame(std::move(*frame));
        }
    }
}

} // namespace trossen_vr

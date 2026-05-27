#include "trossen_vr/network_manager.hpp"

#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include <iostream>
#include <stdexcept>

namespace trossen_vr {

NetworkManager::NetworkManager(const ReceiverConfig& config)
    : config_(config), buffer_(config.buffer_size) {

    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        throw std::runtime_error("Socket creation failed: " + std::string(std::strerror(errno)));
    }

    // Allow reuse of the port immediately after the process exits/crashes.
    // Without this, restarting the program quickly causes "address already in use".
    int reuse = 1;
    if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        int opt_errno = errno;
        close(sockfd_);
        throw std::runtime_error("setsockopt SO_REUSEADDR failed: " + std::string(std::strerror(opt_errno)));
    }

    sockaddr_in servaddr{};
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(config.port);

    if (bind(sockfd_, reinterpret_cast<const sockaddr*>(&servaddr), sizeof(servaddr)) < 0) {
        int bind_errno = errno;
        close(sockfd_);
        throw std::runtime_error("Bind failed on port " + std::to_string(config.port) +
                                 ": " + std::strerror(bind_errno));
    }
}

NetworkManager::~NetworkManager() {
    stop();
    if (sockfd_ >= 0) {
        close(sockfd_);
    }
}

void NetworkManager::start() {
    if (running_) return;
    running_ = true;
    thread_ = std::thread(&NetworkManager::run, this);
}

void NetworkManager::stop() {
    running_ = false;
    if (thread_.joinable()) {
        // Unblock the poll() call by shutting down the socket read side
        shutdown(sockfd_, SHUT_RD);
        thread_.join();
    }
}

std::optional<VRFrame> NetworkManager::latest_frame() const {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return latest_frame_;
}

bool NetworkManager::is_running() const noexcept {
    return running_;
}

ConnectionStatus NetworkManager::get_connection_status() const noexcept {
    return connection_status_.load();
}

double NetworkManager::get_message_frequency() const noexcept {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (total_messages_received_ < 2) return 0.0;

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - first_received_time_).count();
    if (elapsed <= 0.0) return 0.0;

    return static_cast<double>(total_messages_received_) / elapsed;
}

double NetworkManager::get_packet_loss_rate() const noexcept {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (expected_messages_ == 0) return 0.0;
    return static_cast<double>(lost_messages_) / static_cast<double>(expected_messages_);
}

void NetworkManager::send_ack() {
    if (client_addr_len_ == 0) return;  // No client address yet

    // Simple ACK packet with current stats
    nlohmann::json ack;
    ack["type"] = "ack";
    ack["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    ack["frequency_hz"] = get_message_frequency();
    ack["packet_loss"] = get_packet_loss_rate();

    // Send ACK to the client's IP but on the dedicated ACK port.
    // The client sends VR data from an ephemeral source port, but listens
    // for ACKs on a fixed port (ack_port, default 9001).
    sockaddr_in ack_addr = client_addr_;
    ack_addr.sin_port = htons(config_.ack_port);

    std::string ack_str = ack.dump();
    sendto(sockfd_, ack_str.c_str(), ack_str.size(), 0,
           reinterpret_cast<const sockaddr*>(&ack_addr), sizeof(ack_addr));
}

void NetworkManager::update_connection_status() {
    std::lock_guard<std::mutex> lock(status_mutex_);

    auto now = std::chrono::steady_clock::now();

    // No messages received yet
    if (total_messages_received_ == 0) {
        connection_status_ = ConnectionStatus::Disconnected;
        return;
    }

    auto elapsed = std::chrono::duration<double>(now - last_received_time_).count();

    // Check for timeout
    if (elapsed > config_.timeout_seconds) {
        connection_status_ = ConnectionStatus::Disconnected;
        return;
    }

    // First message received but still establishing
    if (total_messages_received_ < 5) {
        connection_status_ = ConnectionStatus::Connecting;
        return;
    }

    // Compute frequency inline — do NOT call get_message_frequency() here
    // because that method also acquires status_mutex_, causing a deadlock.
    double freq = 0.0;
    if (total_messages_received_ >= 2) {
        auto total_elapsed = std::chrono::duration<double>(now - first_received_time_).count();
        if (total_elapsed > 0.0) {
            freq = static_cast<double>(total_messages_received_) / total_elapsed;
        }
    }

    if (freq > 0.0 && freq < config_.min_frequency_hz) {
        connection_status_ = ConnectionStatus::Degraded;
        return;
    }

    // Connection is healthy
    connection_status_ = ConnectionStatus::Connected;
}

void NetworkManager::run() {
    pollfd pfd{};
    pfd.fd = sockfd_;
    pfd.events = POLLIN;

    while (running_) {
        // Check connection timeout periodically
        update_connection_status();

        // Wait up to 50ms for data (allows checking running_ flag periodically)
        int ret = poll(&pfd, 1, 50);
        if (ret <= 0) {
            expected_messages_++;
            if (expected_messages_ >= config_.loss_window) {
                std::lock_guard<std::mutex> lock(status_mutex_);
                expected_messages_ = 0;
                lost_messages_ = 0;
            }
            continue;
        }

        // Drain all queued packets, keep only the last one (newest-frame-wins)
        int last_n = -1;
        sockaddr_in temp_addr{};
        socklen_t temp_len = sizeof(temp_addr);

        while (true) {
            int n = recvfrom(sockfd_, buffer_.data(), buffer_.size(), MSG_DONTWAIT,
                           reinterpret_cast<sockaddr*>(&temp_addr), &temp_len);
            if (n <= 0) break;
            last_n = n;
            // Save client address for ACK replies
            client_addr_ = temp_addr;
            client_addr_len_ = temp_len;
        }

        if (last_n <= 0) {
            expected_messages_++;
            lost_messages_++;
            continue;
        }

        // Update connection tracking
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            auto now = std::chrono::steady_clock::now();
            last_received_time_ = now;
            if (total_messages_received_ == 0) {
                first_received_time_ = now;
            }
            total_messages_received_++;
            expected_messages_++;

            // Reset counters periodically
            if (expected_messages_ >= config_.loss_window) {
                expected_messages_ = 0;
                lost_messages_ = 0;
            }
        }

        try {
            auto data = nlohmann::json::parse(buffer_.data(), buffer_.data() + last_n);
            auto frame = parse_vr_frame(data);
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = std::move(frame);

            // Send ACK back to VR app
            send_ack();
        } catch (const nlohmann::json::exception& e) {
            std::cerr << "[VR] Skipping malformed UDP packet: " << e.what() << std::endl;
        }

        update_connection_status();
    }
}

} // namespace trossen_vr

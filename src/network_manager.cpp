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

    // Allow port reuse for quick restarts
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
    return message_frequency_hz_;
}

double NetworkManager::get_packet_loss_rate() const noexcept {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (expected_messages_ == 0) return 0.0;
    return static_cast<double>(lost_messages_) / static_cast<double>(expected_messages_);
}

void NetworkManager::send_ack() {
    if (client_addr_len_ == 0) return;

    // Build ACK packet with current statistics
    nlohmann::json ack;
    ack["type"] = "ack";
    ack["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    ack["frequency_hz"] = get_message_frequency();
    ack["packet_loss"] = get_packet_loss_rate();

    // Send to client's IP address on dedicated ACK port
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

    // Check if frequency is below minimum threshold
    if (message_frequency_hz_ > 0.0 && message_frequency_hz_ < config_.min_frequency_hz) {
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
        update_connection_status();

        // Poll with timeout to check running_ flag periodically
        int ret = poll(&pfd, 1, 50);
        if (ret <= 0) {
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                expected_messages_++;
                if (expected_messages_ >= config_.loss_window) {
                    expected_messages_ = 0;
                    lost_messages_ = 0;
                }
            }
            continue;
        }

        // Drain all queued packets, keep only newest
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
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                expected_messages_++;
                lost_messages_++;
            }
            continue;
        }

        // Update connection tracking
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            auto now = std::chrono::steady_clock::now();

            // Calculate moving average over 1 second window
            if (total_messages_received_ == 0) {
                freq_window_start_ = now;
                freq_window_count_ = 1;
                message_frequency_hz_ = 0.0;
            } else {
                freq_window_count_++;
                double window_duration = std::chrono::duration<double>(now - freq_window_start_).count();

                // Reset window every second
                if (window_duration >= 1.0) {
                    message_frequency_hz_ = freq_window_count_ / window_duration;
                    freq_window_start_ = now;
                    freq_window_count_ = 0;
                }
            }

            last_received_time_ = now;
            total_messages_received_++;
            expected_messages_++;

            // Reset loss tracking periodically
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

            send_ack();
        } catch (const nlohmann::json::exception& e) {
            std::cerr << "[VR] Skipping malformed UDP packet: " << e.what() << std::endl;
        }

        update_connection_status();
    }
}

} // namespace trossen_vr

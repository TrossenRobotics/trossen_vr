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

UDPReceiver::UDPReceiver(const ReceiverConfig& config)
    : buffer_(config.buffer_size) {

    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        throw std::runtime_error("Socket creation failed: " + std::string(std::strerror(errno)));
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

UDPReceiver::~UDPReceiver() {
    stop();
    if (sockfd_ >= 0) {
        close(sockfd_);
    }
}

void UDPReceiver::start() {
    if (running_) return;
    running_ = true;
    thread_ = std::thread(&UDPReceiver::run, this);
}

void UDPReceiver::stop() {
    running_ = false;
    if (thread_.joinable()) {
        // Unblock the poll() call by shutting down the socket read side
        shutdown(sockfd_, SHUT_RD);
        thread_.join();
    }
}

std::optional<VRFrame> UDPReceiver::latest_frame() const {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return latest_frame_;
}

bool UDPReceiver::is_running() const noexcept {
    return running_;
}

void UDPReceiver::run() {
    pollfd pfd{};
    pfd.fd = sockfd_;
    pfd.events = POLLIN;

    while (running_) {
        // Wait up to 50ms for data (allows checking running_ flag periodically)
        int ret = poll(&pfd, 1, 50);
        if (ret <= 0) continue;

        // Drain all queued packets, keep only the last one (newest-frame-wins)
        int last_n = -1;
        while (true) {
            int n = recvfrom(sockfd_, buffer_.data(), buffer_.size(), MSG_DONTWAIT, nullptr, nullptr);
            if (n <= 0) break;
            last_n = n;
        }

        if (last_n <= 0) continue;

        try {
            auto data = nlohmann::json::parse(buffer_.data(), buffer_.data() + last_n);
            auto frame = parse_vr_frame(data);
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = std::move(frame);
        } catch (const nlohmann::json::exception& e) {
            std::cerr << "[VR] Skipping malformed UDP packet: " << e.what() << std::endl;
        }
    }
}

} // namespace trossen_vr

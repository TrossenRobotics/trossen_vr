#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

#include "trossen_vr/network_manager.hpp"
#include "trossen_vr/vr_types.hpp"

static volatile bool running = true;

void signal_handler(int) {
    running = false;
}

void print_pose(const char* label, const trossen_vr::ControllerPose& pose) {
    std::cout << "  " << label << " pos: ["
              << pose.position.x() << ", "
              << pose.position.y() << ", "
              << pose.position.z() << "]  rot: ["
              << pose.rotation.w() << ", "
              << pose.rotation.x() << ", "
              << pose.rotation.y() << ", "
              << pose.rotation.z() << "]" << std::endl;
}

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    trossen_vr::ReceiverConfig config;
    config.port = 9000;

    trossen_vr::UDPReceiver receiver(config);
    receiver.start();

    std::cout << "Listening on port " << config.port << "..." << std::endl;

    while (running) {
        auto frame = receiver.latest_frame();
        if (!frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        std::cout << "--- VR Frame ---" << std::endl;

        if (frame->right) print_pose("Right", *frame->right);
        if (frame->left) print_pose("Left", *frame->left);

        for (const auto& [name, val] : frame->buttons) {
            std::cout << "  " << name << ": ";
            if (std::holds_alternative<bool>(val))
                std::cout << (std::get<bool>(val) ? "true" : "false");
            else
                std::cout << std::get<double>(val);
            std::cout << std::endl;
        }
    }

    receiver.stop();
    std::cout << "\nDone." << std::endl;
    return 0;
}
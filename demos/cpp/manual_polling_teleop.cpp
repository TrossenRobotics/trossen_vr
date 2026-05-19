#include <iostream>
#include <cmath>
#include <csignal>
#include <chrono>
#include <array>
#include <string>
#include <thread>
#include <unordered_map>

#include "libtrossen_arm/trossen_arm.hpp"
#include "trossen_vr/vr_types.hpp"
#include "trossen_vr/network_manager.hpp"
#include "trossen_vr/teleop.hpp"

static volatile bool running = true;

void signal_handler(int) {
    running = false;
}

class EdgeDetector {
public:
    bool pressed(const trossen_vr::VRFrame& frame, const std::string& name) {
        auto it = frame.buttons.find(name);
        if (it == frame.buttons.end()) return false;
        if (!std::holds_alternative<bool>(it->second)) return false;

        bool current = std::get<bool>(it->second);
        bool previous = prev_[name];
        prev_[name] = current;
        return (current && !previous);
    }

    double analog(const trossen_vr::VRFrame& frame, const std::string& name) {
        auto it = frame.buttons.find(name);
        if (it == frame.buttons.end()) return 0.0;
        if (!std::holds_alternative<double>(it->second)) return 0.0;
        return std::get<double>(it->second);
    }

private:
    std::unordered_map<std::string, bool> prev_;
};

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // --- Configuration ---
    trossen_vr::TeleopConfig config;
    config.right_arm_ip = "192.168.1.4";
    config.left_arm_ip = "192.168.1.2";
    config.send_rate_hz = 100.0;
    config.gripper_max_m = 0.04;
    config.cmd_goal_time = 0.15;

    trossen_vr::ReceiverConfig net_config;
    net_config.port = 9000;

    const std::vector<double> START_POSE = {0, M_PI/3, M_PI/6, M_PI/5, 0, 0};
    const std::vector<double> IDLE_POSE  = {0, 0, 0, 0, 0, 0};

    // --- Robot setup ---
    trossen_arm::TrossenArmDriver right_driver;
    trossen_arm::TrossenArmDriver left_driver;

    right_driver.configure(
        trossen_arm::Model::wxai_v0,
        trossen_arm::StandardEndEffector::wxai_v0_leader,
        config.right_arm_ip, false
    );
    right_driver.set_all_modes(trossen_arm::Mode::position);

    left_driver.configure(
        trossen_arm::Model::wxai_v0,
        trossen_arm::StandardEndEffector::wxai_v0_leader,
        config.left_arm_ip, false
    );
    left_driver.set_all_modes(trossen_arm::Mode::position);

    std::cout << "Moving both arms to start position" << std::endl;
    right_driver.set_arm_positions(START_POSE, 2.0, true);
    left_driver.set_arm_positions(START_POSE, 2.0, true);

    // --- Network setup ---
    trossen_vr::UDPReceiver receiver(net_config);
    receiver.start();

    // --- Teleop state ---
    bool teleop_active = false;
    Eigen::Matrix4d T_offset_right = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_offset_left = Eigen::Matrix4d::Identity();
    bool offset_captured = false;

    EdgeDetector edges;
    auto last_send_time = std::chrono::steady_clock::now();
    const double send_period = 1.0 / config.send_rate_hz;

    std::cout << "Waiting for VR data... Press A to start, B to exit." << std::endl;

    while (running) {
        auto frame_opt = receiver.latest_frame();
        if (!frame_opt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        const auto& frame = *frame_opt;

        if (edges.pressed(frame, "a")) {
            teleop_active = !teleop_active;
            if (teleop_active) {
                offset_captured = false;
                std::cout << "Teleop ENGAGED" << std::endl;
            } else {
                std::cout << "Teleop PAUSED" << std::endl;
            }
        }

        if (edges.pressed(frame, "b")) {
            std::cout << "Exit requested via B button" << std::endl;
            break;
        }

        double right_trig = edges.analog(frame, "rightTrigger");
        right_driver.set_gripper_position(right_trig * config.gripper_max_m, 0.0, false);

        double left_trig = edges.analog(frame, "leftTrigger");
        left_driver.set_gripper_position(left_trig * config.gripper_max_m, 0.0, false);

        if (!teleop_active) continue;

        trossen_vr::Vec6 vr_right = trossen_vr::Vec6::Zero();
        trossen_vr::Vec6 vr_left = trossen_vr::Vec6::Zero();
        bool right_valid = false, left_valid = false;

        if (frame.right) {
            vr_right = trossen_vr::unity_pose_to_vec6(frame.right->position, frame.right->rotation);
            right_valid = true;
        }
        if (frame.left) {
            vr_left = trossen_vr::unity_pose_to_vec6(frame.left->position, frame.left->rotation);
            left_valid = true;
        }

        if (!offset_captured) {
            if (right_valid) {
                auto rp = right_driver.get_cartesian_positions();
                Eigen::Map<Eigen::VectorXd> rs(rp.data(), 6);
                T_offset_right = trossen_vr::vec6_to_T(rs) * trossen_vr::vec6_to_T(vr_right).inverse();
            }
            if (left_valid) {
                auto lp = left_driver.get_cartesian_positions();
                Eigen::Map<Eigen::VectorXd> ls(lp.data(), 6);
                T_offset_left = trossen_vr::vec6_to_T(ls) * trossen_vr::vec6_to_T(vr_left).inverse();
            }
            if (right_valid || left_valid) {
                offset_captured = true;
                std::cout << "Offset captured — tracking active" << std::endl;
            }
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - last_send_time;
        if (elapsed.count() < send_period) continue;
        last_send_time = now;

        if (right_valid) {
            Eigen::Matrix4d T_cmd = T_offset_right * trossen_vr::vec6_to_T(vr_right);
            trossen_vr::Vec6 cmd = trossen_vr::T_to_vec6(T_cmd);
            std::array<double, 6> goal{};
            Eigen::VectorXd::Map(goal.data(), 6) = cmd;
            right_driver.set_cartesian_positions(
                goal, trossen_arm::InterpolationSpace::cartesian, config.cmd_goal_time, false);
        }

        if (left_valid) {
            Eigen::Matrix4d T_cmd = T_offset_left * trossen_vr::vec6_to_T(vr_left);
            trossen_vr::Vec6 cmd = trossen_vr::T_to_vec6(T_cmd);
            std::array<double, 6> goal{};
            Eigen::VectorXd::Map(goal.data(), 6) = cmd;
            left_driver.set_cartesian_positions(
                goal, trossen_arm::InterpolationSpace::cartesian, config.cmd_goal_time, false);
        }
    }

    // --- Graceful shutdown ---
    receiver.stop();
    std::cout << "\nShutting down..." << std::endl;
    std::cout << "Moving arms to idle position" << std::endl;
    right_driver.set_arm_positions(IDLE_POSE, 2.0, true);
    left_driver.set_arm_positions(IDLE_POSE, 2.0, true);

    return 0;
}

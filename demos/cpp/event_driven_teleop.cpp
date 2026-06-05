#include <cmath>
#include <csignal>

#include <array>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "libtrossen_arm/trossen_arm.hpp"

#include "trossen_vr/network_manager.hpp"
#include "trossen_vr/teleop.hpp"
#include "trossen_vr/vr_types.hpp"

static volatile std::sig_atomic_t running = 1;

void signal_handler(int) {
    running = 0;
}

struct ArmTeleopState {
    bool engaged = false;
    Eigen::Matrix4d T_offset = Eigen::Matrix4d::Identity();
    trossen_vr::Vec6 last_vr_vec6 = trossen_vr::Vec6::Zero();
    bool pose_valid = false;
};

void engage_arm(ArmTeleopState& state, trossen_arm::TrossenArmDriver& driver) {
    if (state.engaged || !state.pose_valid) return;

    auto current_p = driver.get_cartesian_positions();
    Eigen::Map<Eigen::VectorXd> robot_start(current_p.data(), 6);
    state.T_offset = trossen_vr::vec6_to_T(robot_start)
                   * trossen_vr::vec6_to_T(state.last_vr_vec6).inverse();
    state.engaged = true;
}

void disengage_arm(ArmTeleopState& state) {
    if (!state.engaged) return;
    state.engaged = false;
}

void send_arm_command(ArmTeleopState& state, trossen_arm::TrossenArmDriver& driver,
                      double cmd_goal_time) {
    if (!state.engaged || !state.pose_valid) return;

    Eigen::Matrix4d T_cmd = state.T_offset * trossen_vr::vec6_to_T(state.last_vr_vec6);
    trossen_vr::Vec6 cmd_vec6 = trossen_vr::T_to_vec6(T_cmd);

    std::array<double, 6> goal{};
    Eigen::VectorXd::Map(goal.data(), 6) = cmd_vec6;
    driver.set_cartesian_positions(
        goal, trossen_arm::InterpolationSpace::cartesian,
        cmd_goal_time, false
    );
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // --- Configuration ---
    const std::string right_arm_ip = "192.168.1.2";
    const std::string left_arm_ip = "192.168.1.3";
    const double send_rate_hz = 100.0;
    const double gripper_max_m = 0.04;
    const double cmd_goal_time = 0.15;

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
        right_arm_ip, false
    );
    right_driver.set_all_modes(trossen_arm::Mode::position);

    left_driver.configure(
        trossen_arm::Model::wxai_v0,
        trossen_arm::StandardEndEffector::wxai_v0_leader,
        left_arm_ip, false
    );
    left_driver.set_all_modes(trossen_arm::Mode::position);

    std::cout << "Moving arms to start position" << std::endl;
    right_driver.set_arm_positions(START_POSE, 2.0, true);
    left_driver.set_arm_positions(START_POSE, 2.0, true);

    // --- Network setup ---
    trossen_vr::NetworkManager receiver(net_config);
    receiver.start();

    // --- Arm state ---
    ArmTeleopState right_state, left_state;

    // --- Event-driven teleop setup ---
    trossen_vr::Teleop teleop;

    teleop.on_button(trossen_vr::ButtonNames::A, [&]() {
        if (!right_state.engaged && !left_state.engaged) {
            engage_arm(right_state, right_driver);
            engage_arm(left_state, left_driver);
            std::cout << "Teleop ENGAGED" << std::endl;
        } else {
            disengage_arm(right_state);
            disengage_arm(left_state);
            std::cout << "Teleop PAUSED" << std::endl;
        }
    });

    teleop.on_button(trossen_vr::ButtonNames::B, [&]() {
        std::cout << "Exit requested via B button" << std::endl;
        running = false;
    });

    teleop.on_analog(trossen_vr::ButtonNames::RightTrigger, [&](double val) {
        right_driver.set_gripper_position(val * gripper_max_m, 0.0, false);
    });

    teleop.on_analog(trossen_vr::ButtonNames::LeftTrigger, [&](double val) {
        left_driver.set_gripper_position(val * gripper_max_m, 0.0, false);
    });

    teleop.on_right_pose([&](const trossen_vr::ControllerPose& pose) {
        right_state.last_vr_vec6 = trossen_vr::unity_pose_to_vec6(pose.position, pose.rotation);
        right_state.pose_valid = true;
    });

    teleop.on_left_pose([&](const trossen_vr::ControllerPose& pose) {
        left_state.last_vr_vec6 = trossen_vr::unity_pose_to_vec6(pose.position, pose.rotation);
        left_state.pose_valid = true;
    });

    // --- Main loop ---
    const double send_period = 1.0 / send_rate_hz;
    auto last_send_time = std::chrono::steady_clock::now();

    std::cout << "Waiting for VR data... Press A to engage, B to exit" << std::endl;

    while (running) {
        auto frame = receiver.latest_frame();
        if (!frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // Only dispatch if we got a new frame (avoid re-processing same data)
        teleop.dispatch(*frame);

        // Rate-limited Cartesian commands
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - last_send_time;
        if (elapsed.count() >= send_period) {
            last_send_time = now;
            send_arm_command(right_state, right_driver, cmd_goal_time);
            send_arm_command(left_state, left_driver, cmd_goal_time);
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

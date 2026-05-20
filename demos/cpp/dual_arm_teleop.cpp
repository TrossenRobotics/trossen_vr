#include <iostream>
#include <cmath>
#include <csignal>
#include <chrono>
#include <array>
#include <thread>
#include <vector>

#include "libtrossen_arm/trossen_arm.hpp"
#include "trossen_vr/vr_types.hpp"
#include "trossen_vr/network_manager.hpp"
#include "trossen_vr/teleop.hpp"

static volatile bool running = true;

void signal_handler(int) {
    running = false;
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
    std::cout << "Teleop ENGAGED" << std::endl;
}

void disengage_arm(ArmTeleopState& state) {
    if (!state.engaged) return;
    state.engaged = false;
    std::cout << "Teleop PAUSED" << std::endl;
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

    // --- Arm state ---
    ArmTeleopState right_state, left_state;

    // --- Event-driven teleop setup ---
    trossen_vr::Teleop teleop;

    teleop.on_button("a", [&]() {
        if (!right_state.engaged && !left_state.engaged) {
            engage_arm(right_state, right_driver);
            engage_arm(left_state, left_driver);
        } else {
            disengage_arm(right_state);
            disengage_arm(left_state);
        }
    });

    teleop.on_button("b", [&]() {
        std::cout << "Exit requested via B button" << std::endl;
        running = false;
    });

    teleop.on_analog("rightTrigger", [&](double val) {
        right_driver.set_gripper_position(val * config.gripper_max_m, 0.0, false);
    });

    teleop.on_analog("leftTrigger", [&](double val) {
        left_driver.set_gripper_position(val * config.gripper_max_m, 0.0, false);
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
    const double send_period = 1.0 / config.send_rate_hz;
    auto last_send_time = std::chrono::steady_clock::now();

    std::cout << "Waiting for VR data... Press A to engage teleop." << std::endl;

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
            send_arm_command(right_state, right_driver, config.cmd_goal_time);
            send_arm_command(left_state, left_driver, config.cmd_goal_time);
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
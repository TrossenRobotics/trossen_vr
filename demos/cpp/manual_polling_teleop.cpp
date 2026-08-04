#include <cmath>
#include <csignal>

#include <array>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "libtrossen_arm/trossen_arm.hpp"
#include "trossen_vr/trossen_vr.hpp"

static volatile std::sig_atomic_t running = 1;

void signal_handler(int) {
    running = 0;
}

// Widen the arm joints' velocity/effort fault *tolerance* to their max so
// normal teleop motion doesn't spuriously trip a fault; genuine faults
// (e.g. a real position limit) still trip and are handled via the
// right_faulted/left_faulted flags in main() rather than crashing the
// process.
void relax_velocity_effort_limits(trossen_arm::TrossenArmDriver& driver) {
    auto limits = driver.get_joint_limits();
    for (std::size_t i = 0; i < 6; ++i) {
        limits[i].velocity_tolerance = limits[i].velocity_max;
        limits[i].effort_tolerance = limits[i].effort_max;
    }
    driver.set_joint_limits(limits);
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Configuration
    const std::string right_arm_ip = "192.168.1.4";
    const std::string left_arm_ip = "192.168.1.5";
    const double send_rate_hz = 100.0;
    const double gripper_max_m = 0.04;
    const double cmd_goal_time = 0.20;

    trossen_vr::ReceiverConfig net_config;
    net_config.port = 9000;

    const std::vector<double> START_POSE = {0, M_PI/3, M_PI/6, M_PI/5, 0, 0};
    const std::vector<double> IDLE_POSE  = {0, 0, 0, 0, 0, 0};

    // Robot setup
    trossen_arm::TrossenArmDriver right_driver;
    trossen_arm::TrossenArmDriver left_driver;

    right_driver.configure(
        trossen_arm::Model::wxai_v0,
        trossen_arm::StandardEndEffector::wxai_v0_leader,
        right_arm_ip, false
    );
    relax_velocity_effort_limits(right_driver);
    right_driver.set_all_modes(trossen_arm::Mode::position);

    left_driver.configure(
        trossen_arm::Model::wxai_v0,
        trossen_arm::StandardEndEffector::wxai_v0_leader,
        left_arm_ip, false
    );
    relax_velocity_effort_limits(left_driver);
    left_driver.set_all_modes(trossen_arm::Mode::position);

    std::cout << "Moving arms to start position" << std::endl;
    right_driver.set_arm_positions(START_POSE, 2.0, true);
    left_driver.set_arm_positions(START_POSE, 2.0, true);

    // Network setup
    trossen_vr::NetworkManager receiver(net_config);
    receiver.start();

    // Teleop state
    trossen_vr::Transform4D T_offset_right;
    trossen_vr::Transform4D T_offset_left;
    bool offset_captured = false;

    // Track previous tracking state for engage/disengage detection
    uint8_t prev_right_tracked = 0;
    uint8_t prev_left_tracked = 0;

    // Button state tracking for edge detection
    uint8_t prev_button_b_right = 0;
    uint8_t prev_button_a_right = 0;

    // Set when a driver call throws (e.g. a joint limit). While true, that
    // arm is skipped entirely until recovered (button A).
    bool right_faulted = false;
    bool left_faulted = false;

    auto last_send_time = std::chrono::steady_clock::now();
    const double send_period = 1.0 / send_rate_hz;
    trossen_vr::ConnectionStatus last_status = trossen_vr::ConnectionStatus::Disconnected;

    std::cout << "Waiting for VR data... Hand/Grip trigger to engage, release to pause. "
                 "Press A to recover from a fault, B to exit" << std::endl;

    while (running) {
        // Monitor connection status
        auto current_status = receiver.get_connection_status();
        if (current_status != last_status) {
            switch (current_status) {
                case trossen_vr::ConnectionStatus::Connecting:
                    std::cout << "Connecting..." << std::endl;
                    break;
                case trossen_vr::ConnectionStatus::Connected:
                    std::cout << "Connection established (";
                    std::cout << receiver.get_message_frequency() << " Hz)" << std::endl;
                    break;
                case trossen_vr::ConnectionStatus::Degraded:
                    std::cout << "Connection degraded (low frequency: ";
                    std::cout << receiver.get_message_frequency() << " Hz)" << std::endl;
                    break;
                case trossen_vr::ConnectionStatus::Disconnected:
                    std::cout << "Connection lost (timeout)" << std::endl;
                    break;
            }
            last_status = current_status;
        }

        auto frame_opt = receiver.latest_frame();
        if (!frame_opt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        const auto& frame = *frame_opt;

        bool right_tracked = frame.right_controller.is_tracked != 0;
        bool left_tracked = frame.left_controller.is_tracked != 0;

        // Detect engage (tracking transition 0→1)
        if (right_tracked && !prev_right_tracked) {
            std::cout << "Right controller ENGAGED" << std::endl;
            offset_captured = false;
        }
        if (left_tracked && !prev_left_tracked) {
            std::cout << "Left controller ENGAGED" << std::endl;
            offset_captured = false;
        }

        // Detect release (tracking transition 1→0)
        if (!right_tracked && prev_right_tracked) {
            std::cout << "Right controller PAUSED" << std::endl;
        }
        if (!left_tracked && prev_left_tracked) {
            std::cout << "Left controller PAUSED" << std::endl;
        }

        prev_right_tracked = right_tracked;
        prev_left_tracked = left_tracked;

        // Edge detect button B (exit)
        uint8_t button_b_right = frame.right_controller.buttons.two;
        if (button_b_right && !prev_button_b_right) {
            std::cout << "Exit requested via B button" << std::endl;
            break;
        }
        prev_button_b_right = button_b_right;

        // Edge detect button A - recover either/both arms from a fault
        uint8_t button_a_right = frame.right_controller.buttons.one;
        if (button_a_right && !prev_button_a_right) {
            if (!right_faulted && !left_faulted) {
                std::cout << "A pressed, but no arm is faulted" << std::endl;
            }
            if (right_faulted) {
                try {
                    right_driver.clear_error();
                    right_driver.set_all_modes(trossen_arm::Mode::position);
                    std::cout << "Right arm: moving to home before resuming teleop" << std::endl;
                    right_driver.set_arm_positions(START_POSE, 2.0, true);
                    right_faulted = false;
                    offset_captured = false;  // re-capture instead of jumping
                    std::cout << "Right arm: fault cleared" << std::endl;
                } catch (const std::exception& e) {
                    std::cout << "Right arm: clear_error failed, still faulted: " << e.what() << std::endl;
                }
            }
            if (left_faulted) {
                try {
                    left_driver.clear_error();
                    left_driver.set_all_modes(trossen_arm::Mode::position);
                    std::cout << "Left arm: moving to home before resuming teleop" << std::endl;
                    left_driver.set_arm_positions(START_POSE, 2.0, true);
                    left_faulted = false;
                    offset_captured = false;  // re-capture instead of jumping
                    std::cout << "Left arm: fault cleared" << std::endl;
                } catch (const std::exception& e) {
                    std::cout << "Left arm: clear_error failed, still faulted: " << e.what() << std::endl;
                }
            }
        }
        prev_button_a_right = button_a_right;

        // Update grippers
        if (!right_faulted) {
            try {
                right_driver.set_gripper_position(
                    frame.right_controller.triggers.index_trigger * gripper_max_m, 0.0, false);
            } catch (const std::exception& e) {
                std::cout << "Right arm: FAULT, press A to recover: " << e.what() << std::endl;
                right_faulted = true;
            }
        }
        if (!left_faulted) {
            try {
                left_driver.set_gripper_position(
                    frame.left_controller.triggers.index_trigger * gripper_max_m, 0.0, false);
            } catch (const std::exception& e) {
                std::cout << "Left arm: FAULT, press A to recover: " << e.what() << std::endl;
                left_faulted = true;
            }
        }

        // Capture offset on first valid frame after engage
        if (!offset_captured && (right_tracked || left_tracked)) {
            if (right_tracked && !right_faulted) {
                try {
                    auto rp = right_driver.get_cartesian_positions();
                    trossen_vr::Pose6D robot_pose_right;
                    robot_pose_right.x = rp[0]; robot_pose_right.y = rp[1]; robot_pose_right.z = rp[2];
                    robot_pose_right.ax = rp[3]; robot_pose_right.ay = rp[4]; robot_pose_right.az = rp[5];

                    trossen_vr::Transform4D T_robot_right = pose6d_to_transform4d(robot_pose_right);
                    trossen_vr::Transform4D T_vr_right = pose6d_to_transform4d(frame.right_controller.pose6d);
                    T_offset_right = T_robot_right * T_vr_right.inverse();
                } catch (const std::exception& e) {
                    std::cout << "Right arm: FAULT, press A to recover: " << e.what() << std::endl;
                    right_faulted = true;
                }
            }
            if (left_tracked && !left_faulted) {
                try {
                    auto lp = left_driver.get_cartesian_positions();
                    trossen_vr::Pose6D robot_pose_left;
                    robot_pose_left.x = lp[0]; robot_pose_left.y = lp[1]; robot_pose_left.z = lp[2];
                    robot_pose_left.ax = lp[3]; robot_pose_left.ay = lp[4]; robot_pose_left.az = lp[5];

                    trossen_vr::Transform4D T_robot_left = pose6d_to_transform4d(robot_pose_left);
                    trossen_vr::Transform4D T_vr_left = pose6d_to_transform4d(frame.left_controller.pose6d);
                    T_offset_left = T_robot_left * T_vr_left.inverse();
                } catch (const std::exception& e) {
                    std::cout << "Left arm: FAULT, press A to recover: " << e.what() << std::endl;
                    left_faulted = true;
                }
            }
            offset_captured = true;
            continue;
        }

        // Rate limiting
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - last_send_time;
        if (elapsed.count() < send_period) {
            // Sleep for remaining time
            double remaining = send_period - elapsed.count();
            std::this_thread::sleep_for(
                std::chrono::duration<double>(remaining));
            continue;
        }
        last_send_time = now;

        // Send commands only when controllers are tracked
        if (offset_captured && right_tracked && !right_faulted) {
            try {
                trossen_vr::Transform4D T_vr_right = pose6d_to_transform4d(frame.right_controller.pose6d);
                trossen_vr::Transform4D T_cmd_right = T_offset_right * T_vr_right;
                trossen_vr::Pose6D cmd_right = transform4d_to_pose6d(T_cmd_right);
                std::array<double, 6> goal{cmd_right.x, cmd_right.y, cmd_right.z,
                                          cmd_right.ax, cmd_right.ay, cmd_right.az};
                right_driver.set_cartesian_positions(
                    goal, trossen_arm::InterpolationSpace::cartesian, cmd_goal_time, false);
            } catch (const std::exception& e) {
                std::cout << "Right arm: FAULT, press A to recover: " << e.what() << std::endl;
                right_faulted = true;
            }
        }

        if (offset_captured && left_tracked && !left_faulted) {
            try {
                trossen_vr::Transform4D T_vr_left = pose6d_to_transform4d(frame.left_controller.pose6d);
                trossen_vr::Transform4D T_cmd_left = T_offset_left * T_vr_left;
                trossen_vr::Pose6D cmd_left = transform4d_to_pose6d(T_cmd_left);
                std::array<double, 6> goal{cmd_left.x, cmd_left.y, cmd_left.z,
                                          cmd_left.ax, cmd_left.ay, cmd_left.az};
                left_driver.set_cartesian_positions(
                    goal, trossen_arm::InterpolationSpace::cartesian, cmd_goal_time, false);
            } catch (const std::exception& e) {
                std::cout << "Left arm: FAULT, press A to recover: " << e.what() << std::endl;
                left_faulted = true;
            }
        }
    }

    // Shutdown
    receiver.stop();
    std::cout << "\nShutting down..." << std::endl;
    std::cout << "Moving arms to idle position" << std::endl;
    try {
        right_driver.set_arm_positions(IDLE_POSE, 2.0, true);
    } catch (const std::exception& e) {
        std::cout << "Right arm: could not move to idle (still faulted?): " << e.what() << std::endl;
    }
    try {
        left_driver.set_arm_positions(IDLE_POSE, 2.0, true);
    } catch (const std::exception& e) {
        std::cout << "Left arm: could not move to idle (still faulted?): " << e.what() << std::endl;
    }

    return 0;
}

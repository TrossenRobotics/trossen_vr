#include <cmath>
#include <csignal>

#include <array>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "libtrossen_arm/trossen_arm.hpp"
#include "trossen_vr/trossen_vr.hpp"

static volatile std::sig_atomic_t running = 1;

void signal_handler(int) {
    running = 0;
}

struct ArmModelConfig {
    trossen_arm::Model model;
    trossen_arm::EndEffector end_effector;
};

// Supported arm models and their matching end effector
const std::unordered_map<std::string, ArmModelConfig> kModelConfigs = {
    {"wxai_v0", {trossen_arm::Model::wxai_v0, trossen_arm::StandardEndEffector::wxai_v0_follower}},
    {"pro", {trossen_arm::Model::pro, trossen_arm::StandardEndEffector::pro_base}},
};

bool parse_model_arg(const std::string& value, ArmModelConfig& config) {
    auto it = kModelConfigs.find(value);
    if (it == kModelConfigs.end()) {
        return false;
    }
    config = it->second;
    return true;
}

struct ArmTeleopState {
    bool engaged = false;
    trossen_vr::Transform4D T_offset;
    trossen_vr::Pose6D last_vr_pose;
    bool pose_valid = false;
};

void engage_arm(ArmTeleopState& state, trossen_arm::TrossenArmDriver& driver) {
    if (state.engaged || !state.pose_valid) return;

    auto current_p = driver.get_cartesian_positions();
    trossen_vr::Pose6D robot_pose;
    robot_pose.x = current_p[0]; robot_pose.y = current_p[1]; robot_pose.z = current_p[2];
    robot_pose.ax = current_p[3]; robot_pose.ay = current_p[4]; robot_pose.az = current_p[5];

    trossen_vr::Transform4D T_robot = trossen_vr::pose6d_to_transform4d(robot_pose);
    trossen_vr::Transform4D T_vr = trossen_vr::pose6d_to_transform4d(state.last_vr_pose);
    state.T_offset = T_robot * T_vr.inverse();
    state.engaged = true;
}

void disengage_arm(ArmTeleopState& state) {
    if (!state.engaged) return;
    state.engaged = false;
    state.pose_valid = false;
}

void send_arm_command(ArmTeleopState& state, trossen_arm::TrossenArmDriver& driver,
                      double cmd_goal_time) {
    if (!state.engaged || !state.pose_valid) return;

    trossen_vr::Transform4D T_vr = trossen_vr::pose6d_to_transform4d(state.last_vr_pose);
    trossen_vr::Transform4D T_cmd = state.T_offset * T_vr;
    trossen_vr::Pose6D cmd_pose = trossen_vr::transform4d_to_pose6d(T_cmd);
    std::array<double, 6> goal{cmd_pose.x, cmd_pose.y, cmd_pose.z,
                              cmd_pose.ax, cmd_pose.ay, cmd_pose.az};
    driver.set_cartesian_positions(
        goal, trossen_arm::InterpolationSpace::cartesian,
        cmd_goal_time, false
    );
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Parse --model-left / --model-right (wxai_v0 or pro)
    std::string model_left_arg = "wxai_v0";
    std::string model_right_arg = "wxai_v0";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--model-left" && i + 1 < argc) {
            model_left_arg = argv[++i];
        } else if (arg == "--model-right" && i + 1 < argc) {
            model_right_arg = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0]
                      << " [--model-left wxai_v0|pro] [--model-right wxai_v0|pro]" << std::endl;
            return 0;
        }
    }

    ArmModelConfig left_arm_config, right_arm_config;
    if (!parse_model_arg(model_left_arg, left_arm_config)) {
        std::cerr << "Unknown model '" << model_left_arg << "' for --model-left. "
                  << "Valid options: wxai_v0, pro" << std::endl;
        return 1;
    }
    if (!parse_model_arg(model_right_arg, right_arm_config)) {
        std::cerr << "Unknown model '" << model_right_arg << "' for --model-right. "
                  << "Valid options: wxai_v0, pro" << std::endl;
        return 1;
    }

    // Configuration
    const std::string right_arm_ip = "192.168.1.4";
    const std::string left_arm_ip = "192.168.1.5";
    const double send_rate_hz = 100.0;
    const double gripper_max_m = 0.04;
    const double cmd_goal_time = 0.15;

    trossen_vr::ReceiverConfig net_config;
    net_config.port = 9000;

    const std::vector<double> START_POSE = {0, M_PI/3, M_PI/6, M_PI/5, 0, 0};
    const std::vector<double> IDLE_POSE  = {0, 0, 0, 0, 0, 0};

    // Robot setup
    trossen_arm::TrossenArmDriver right_driver;
    trossen_arm::TrossenArmDriver left_driver;

    right_driver.configure(
        right_arm_config.model,
        right_arm_config.end_effector,
        right_arm_ip, false
    );
    right_driver.set_all_modes(trossen_arm::Mode::position);

    left_driver.configure(
        left_arm_config.model,
        left_arm_config.end_effector,
        left_arm_ip, false
    );
    left_driver.set_all_modes(trossen_arm::Mode::position);

    std::cout << "Moving arms to start position" << std::endl;
    right_driver.set_arm_positions(START_POSE, 2.0, true);
    left_driver.set_arm_positions(START_POSE, 2.0, true);

    // Network setup
    trossen_vr::NetworkManager receiver(net_config);
    receiver.start();

    // Arm state
    ArmTeleopState right_state, left_state;

    // Track previous tracking state for grip engage/disengage detection
    uint8_t prev_right_tracked = 0;
    uint8_t prev_left_tracked = 0;

    // Event-driven teleop setup
    trossen_vr::Teleop teleop;

    // Button B - exit
    teleop.on_button_b([&]() {
        std::cout << "Exit requested via B button" << std::endl;
        running = false;
    });

    // Right trigger - right gripper
    teleop.on_right_trigger([&](double val) {
        right_driver.set_gripper_position(val * gripper_max_m, 0.0, false);
    });

    // Left trigger - left gripper
    teleop.on_left_trigger([&](double val) {
        left_driver.set_gripper_position(val * gripper_max_m, 0.0, false);
    });

    // Right controller pose - auto engage/disengage based on tracking
    teleop.on_right_pose([&](const trossen_vr::Pose6D& pose) {
        right_state.last_vr_pose = pose;
        right_state.pose_valid = true;

        if (!prev_right_tracked) {
            // Auto-engage arm
            engage_arm(right_state, right_driver);
            std::cout << "Right arm ENGAGED" << std::endl;
        }
        prev_right_tracked = 1;
    });

    // Left controller pose - auto engage/disengage based on tracking
    teleop.on_left_pose([&](const trossen_vr::Pose6D& pose) {
        left_state.last_vr_pose = pose;
        left_state.pose_valid = true;

        if (!prev_left_tracked) {
            // Auto-engage arm
            engage_arm(left_state, left_driver);
            std::cout << "Left arm ENGAGED" << std::endl;
        }
        prev_left_tracked = 1;
    });

    // Main loop
    const double send_period = 1.0 / send_rate_hz;
    auto last_send_time = std::chrono::steady_clock::now();
    trossen_vr::ConnectionStatus last_status = trossen_vr::ConnectionStatus::Disconnected;

    std::cout << "Waiting for VR data... Hand/Grip trigger to engage, release to pause. Press B to exit" << std::endl;

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

        auto frame = receiver.latest_frame();
        if (!frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // Dispatch to event handlers
        teleop.dispatch(*frame);

        // Check for tracking release - disengage
        bool right_tracked = frame->right_controller.is_tracked != 0;
        bool left_tracked = frame->left_controller.is_tracked != 0;

        if (!right_tracked && prev_right_tracked) {
            disengage_arm(right_state);
            std::cout << "Right arm PAUSED" << std::endl;
            prev_right_tracked = 0;
        }
        if (!left_tracked && prev_left_tracked) {
            disengage_arm(left_state);
            std::cout << "Left arm PAUSED" << std::endl;
            prev_left_tracked = 0;
        }

        // Rate-limited command sending
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - last_send_time;
        if (elapsed.count() >= send_period) {
            send_arm_command(right_state, right_driver, cmd_goal_time);
            send_arm_command(left_state, left_driver, cmd_goal_time);
            last_send_time = now;
        }
    }

    // Shutdown
    receiver.stop();
    std::cout << "\nShutting down..." << std::endl;
    std::cout << "Moving arms to idle position" << std::endl;
    right_driver.set_arm_positions(IDLE_POSE, 2.0, true);
    left_driver.set_arm_positions(IDLE_POSE, 2.0, true);

    return 0;
}

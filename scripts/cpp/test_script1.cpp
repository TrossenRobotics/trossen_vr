#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <trossen_vr/vr_manager.hpp>
#include <trossen_vr/teleop.hpp>
#include <trossen_vr/vr_types.hpp>

#include "libtrossen_arm/trossen_arm.hpp"

// ---------------------------
// Utility functions
// ---------------------------
Eigen::Matrix4d vec6_to_T(const std::array<double, 6>& v6) {
    Eigen::Vector3d p(v6[0], v6[1], v6[2]);
    Eigen::Vector3d rvec(v6[3], v6[4], v6[5]);
    double angle = rvec.norm();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    if (angle > 1e-8) {
        Eigen::Vector3d axis = rvec / angle;
        R = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    }
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = p;
    return T;
}

std::array<double, 6> T_to_vec6(const Eigen::Matrix4d& T) {
    std::array<double, 6> v6;
    Eigen::Vector3d p = T.block<3,1>(0,3);
    v6[0] = p.x(); v6[1] = p.y(); v6[2] = p.z();
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::AngleAxisd aa(R);
    Eigen::Vector3d rvec = aa.axis() * aa.angle();
    v6[3] = rvec.x(); v6[4] = rvec.y(); v6[5] = rvec.z();
    return v6;
}

std::optional<std::array<double,6>> parse_vr_pose(const std::optional<trossen_vr::VRPose>& pose) {
    if (!pose.has_value()) return std::nullopt;
    std::array<double, 6> v6;
    v6[0] = pose->position[0]; v6[1] = pose->position[1]; v6[2] = pose->position[2];
    v6[3] = pose->rotation[0]; v6[4] = pose->rotation[1]; v6[5] = pose->rotation[2];
    return v6;
}

// ---------------------------
// Main
// ---------------------------
int main() {
    using namespace trossen_vr;

    std::vector<double> START_POSE = {0, M_PI/12, M_PI/12, 0, 0, 0};
    std::vector<double> IDLE_POSE  = {0, 0, 0, 0, 0, 0};

    trossen_arm::TrossenArmDriver driver;
    driver.configure(trossen_arm::Model::wxai_v0,
                     trossen_arm::StandardEndEffector::wxai_v0_follower,
                     "192.168.1.2", true);

    driver.set_arm_modes(trossen_arm::Mode::position);
    driver.set_arm_positions(IDLE_POSE, 3.0, true);
    driver.set_gripper_mode(trossen_arm::Mode::position);
    driver.set_arm_positions(START_POSE, 3.0, true);

    std::cout << "Robot ready." << std::endl;

    // ---------------------------
    // VRManager setup
    // ---------------------------
    VRManager::Config cfg;
    cfg.server_port = 5432;
    VRManager vr_manager(cfg);
    vr_manager.start();
    // ---------------------------
    // Teleop setup
    // ---------------------------
    Teleop teleop;

    bool pause_teleop = true;
    std::optional<std::array<double,6>> init_right_pose;
    std::optional<std::array<double,6>> init_robot_pose;
    Eigen::Matrix4d T_offset_right = Eigen::Matrix4d::Identity();

    // ---------------------------
    // Handlers
    // ---------------------------

    // Right controller pose
    teleop.set_right_pose_handler([&](const VRPose& pose) {
        auto right_pose_opt = parse_vr_pose(pose);
        if (!right_pose_opt.has_value() || pause_teleop) return;

        auto right_pose_vec = right_pose_opt.value();

        // Initialization
        if (!init_right_pose.has_value()) {
            init_robot_pose = driver.get_cartesian_positions();
            init_right_pose = right_pose_vec;
            T_offset_right = vec6_to_T(*init_robot_pose) * vec6_to_T(right_pose_vec).inverse();
            std::cout << "Teleop initialized — right controller aligned." << std::endl;
        }

        Eigen::Matrix4d Tq = vec6_to_T(right_pose_vec);
        Eigen::Matrix4d Tt = T_offset_right * Tq;
        auto robot_cmd = T_to_vec6(Tt);
        
        // std::cout << "Sending robot command: ";
        // for (auto v : robot_cmd) std::cout << v << " ";
        // std::cout << std::endl;

        driver.set_cartesian_positions(robot_cmd,
                                       trossen_arm::InterpolationSpace::cartesian,
                                       0.15, false);
    });

    // Button handlers
    teleop.set_button_A_handler([&](){ pause_teleop = !pause_teleop; });
    teleop.set_button_B_handler([&](){ driver.set_arm_positions(IDLE_POSE, 3.0, true); });

    // Gripper handled in right pose handler using button map
    teleop.set_button_right_trigger_handler([&]() {

        auto trigger_val_opt = vr_manager.get_button_state("right_trigger");
        if (trigger_val_opt.has_value()) {
            double trigger_val = std::get<double>(*trigger_val_opt);
            driver.set_gripper_position(trigger_val * 0.04, 0.0, false);
        }
    });

    // ---------------------------
    // Main polling loop (~200 Hz)
    // ---------------------------
    while (true) {
        vr_manager.poll_teleop(teleop);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Cleanup (won't usually be reached)
    driver.set_arm_positions(IDLE_POSE, 3.0, true);
    return 0;
}
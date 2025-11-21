#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <trossen_vr/vr_manager.hpp>
#include <trossen_vr/vr_types.hpp>

#include "libtrossen_arm/trossen_arm.hpp"


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

std::optional<std::array<double,6>> pose_to_vec6(const std::optional<trossen_vr::VRPose>& pose) {
    if (!pose.has_value()) return std::nullopt;
    std::array<double, 6> v;
    v[0] = pose->position[0];
    v[1] = pose->position[1];
    v[2] = pose->position[2];
    v[3] = pose->rotation[0];
    v[4] = pose->rotation[1];
    v[5] = pose->rotation[2];
    return v;
}



// -------------------------------------
// MAIN — MANUAL MODE
// -------------------------------------
int main() {
    using namespace trossen_vr;

    std::vector<double> START_POSE = {0, M_PI/12, M_PI/12, 0, 0, 0};
    std::vector<double> IDLE_POSE  = {0, 0, 0, 0, 0, 0};

    // ---------------------------
    // Robot setup
    // ---------------------------
    trossen_arm::TrossenArmDriver driver;
    driver.configure(trossen_arm::Model::wxai_v0,
                     trossen_arm::StandardEndEffector::wxai_v0_follower,
                     "192.168.1.2", true);

    driver.set_arm_modes(trossen_arm::Mode::position);
    driver.set_gripper_mode(trossen_arm::Mode::position);
    driver.set_arm_positions(START_POSE, 3.0, true);

    std::cout << "Robot ready.\n";



    // ---------------------------
    // VRManager (manual mode)
    // ---------------------------
    VRManager::Config cfg;
    cfg.server_port = 5432;
    VRManager vr_manager(cfg);

    vr_manager.start();

    std::cout << "VR Manager started in MANUAL mode.\n";



    // ---------------------------
    // State for teleop alignment
    // ---------------------------
    bool pause_teleop = false;
    std::optional<std::array<double,6>> init_right_pose;
    std::optional<std::array<double,6>> init_robot_pose;
    Eigen::Matrix4d T_offset_right = Eigen::Matrix4d::Identity();



    // -------------------------------------
    // BUTTON HANDLING HELPER 
    // -------------------------------------
    // handles rising edge detection for buttons 
    std::unordered_map<std::string, bool> prev_buttons;
    auto button_pressed = [&](const VRState& st, const std::string& name) {
        auto it = st.buttons.find(name);
        if (it == st.buttons.end()) return false;
        if (!std::holds_alternative<bool>(it->second)) return false;

        bool current = std::get<bool>(it->second);
        bool previous = prev_buttons[name];
        prev_buttons[name] = current;
        return (!previous && current);  // rising edge
    };



    // ---------------------------
    // Main teleop loop (~200 Hz)
    // ---------------------------
    while (true) {

        // Read a full VRState packet manually
        auto frame_opt = vr_manager.poll_manual();
        if (!frame_opt.has_value()) {
            continue;
        }

        const VRState& frame = *frame_opt;


        // -------------------------------------
        // BUTTON HANDLING
        // -------------------------------------
        // Track previous button states (bool-type buttons only)
        // Toggle pause/resume on A
        if (button_pressed(frame, "a")) {
            pause_teleop = !pause_teleop;
            std::cout << (pause_teleop ? "Teleop PAUSED\n" : "Teleop RESUMED\n");
        }


        // B returns robot to idle pose
        if (button_pressed(frame, "b")) {
            std::cout << "Returning to IDLE pose.\n";
            driver.set_arm_positions(IDLE_POSE, 3.0, true);
            break;
        }





        // -------------------------------------
        // GRIPPER TRIGGER
        // -------------------------------------
        {
            auto it = frame.buttons.find("right_trigger");
            if (it != frame.buttons.end() && std::holds_alternative<double>(it->second)) {
                double trig_val = std::get<double>(it->second);
                driver.set_gripper_position(trig_val * 0.04, 0.0, false);
            }
        }


        // -------------------------------------
        // RIGHT CONTROLLER (main teleop)
        // -------------------------------------
        if (!pause_teleop) {
            auto right_v6 = pose_to_vec6(frame.right_pose);
            if (right_v6.has_value()) {

                // controller robot alignment
                if (!init_right_pose.has_value()) {
                    init_robot_pose = driver.get_cartesian_positions();
                    init_right_pose = right_v6;

                    T_offset_right =
                        vec6_to_T(*init_robot_pose) *
                        vec6_to_T(*right_v6).inverse();

                    std::cout << "Teleop initialized.\n";
                }

                Eigen::Matrix4d Tq = vec6_to_T(*right_v6);
                Eigen::Matrix4d Tt = T_offset_right * Tq;
                auto robot_cmd = T_to_vec6(Tt);

                driver.set_cartesian_positions(
                    robot_cmd,
                    trossen_arm::InterpolationSpace::cartesian,
                    0.15, false
                );
            }
        }


        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    driver.set_arm_positions(IDLE_POSE, 3.0, true);
    return 0;
}

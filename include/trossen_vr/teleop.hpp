#ifndef TROSSEN_VR_TELEOP_HPP
#define TROSSEN_VR_TELEOP_HPP

#include <functional>
#include <string>
#include <unordered_map>

#include "trossen_vr/vr_types.hpp"

namespace trossen_vr {

struct TeleopConfig {
    std::string right_arm_ip = "192.168.1.2";
    std::string left_arm_ip = "192.168.1.3";
    double send_rate_hz = 100.0;
    double gripper_max_m = 0.04;
    double cmd_goal_time = 0.15;
};

// Event-driven teleop dispatcher.
// Register handlers for buttons (rising-edge for bools, always-fire for analog)
// and poses, then call dispatch() each frame.
class Teleop {
public:
    using PoseHandler = std::function<void(const ControllerPose&)>;
    using ButtonHandler = std::function<void()>;
    using AnalogHandler = std::function<void(double)>;

    void on_button(const std::string& name, ButtonHandler handler);
    void on_analog(const std::string& name, AnalogHandler handler);
    void on_right_pose(PoseHandler handler);
    void on_left_pose(PoseHandler handler);

    // Dispatch a VR frame: detects edges, fires handlers
    void dispatch(const VRFrame& frame);

private:
    std::unordered_map<std::string, ButtonHandler> button_handlers_;
    std::unordered_map<std::string, AnalogHandler> analog_handlers_;
    std::unordered_map<std::string, bool> prev_button_states_;

    PoseHandler right_pose_handler_;
    PoseHandler left_pose_handler_;
};

} // namespace trossen_vr

#endif // TROSSEN_VR_TELEOP_HPP

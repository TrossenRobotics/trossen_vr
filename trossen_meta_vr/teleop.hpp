#pragma once

#include "vr_types.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>

namespace trossen::meta_vr {

// Base class that maps VR events to robot-specific callbacks.
class Teleop {
public:
    using PoseHandler = std::function<void(const VRPose&)>;
    using ActionHandler = std::function<void()>;
    using ExitPredicate = std::function<bool(const std::unordered_map<std::string, VRButtonState>&)>;

    virtual ~Teleop() = default;

    // Register button handlers for controls.
    void set_button_A(ActionHandler handler);
    void set_button_B(ActionHandler handler);
    void set_button_trigger(ActionHandler handler);
    void set_button_grip(ActionHandler handler);

    // Connection callbacks by VR commands.
    void set_start_handler(ActionHandler handler);
    void set_pause_handler(ActionHandler handler);
    void set_resume_handler(ActionHandler handler);

    // Pose callbacks
    void set_left_pose_handler(PoseHandler handler);
    void set_right_pose_handler(PoseHandler handler);
    void set_pose_handler(PoseHandler handler);
    void set_exit_condition(ExitPredicate predicate);

    // classes to add configuration flags (such as timecontrol/replay/etc.).
    void configure(std::uint32_t flags);
    std::uint32_t config_flags() const noexcept;

    // send connection events if the handler is present.
    void notify_start();
    void notify_pause();
    void notify_resume();

    // pose data and button states processes.
    void handle_pose(const VRPose& pose) const;
    bool evaluate_button_states(const std::unordered_map<std::string, VRButtonState>& buttons);

protected:
    virtual void on_configure(std::uint32_t flags) = 0;
    void register_button_handler(const std::string& button, ActionHandler handler);

private:
    std::unordered_map<std::string, ActionHandler> button_handlers_;
    std::unordered_map<std::string, bool> previous_button_states_;
    PoseHandler left_pose_handler_;
    PoseHandler right_pose_handler_;
    ActionHandler on_start_;
    ActionHandler on_pause_;
    ActionHandler on_resume_;
    ExitPredicate exit_predicate_;
};

}

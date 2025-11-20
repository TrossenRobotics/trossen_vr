#include "trossen_vr/teleop.hpp"

namespace trossen_vr {

// Button Handlers
void Teleop::set_button_A_handler(ActionHandler handler) { register_button_handler("a", std::move(handler)); }
void Teleop::set_button_B_handler(ActionHandler handler) { register_button_handler("b", std::move(handler)); }
void Teleop::set_button_trigger_handler(ActionHandler handler) { register_button_handler("trigger", std::move(handler)); }
void Teleop::set_button_grip_handler(ActionHandler handler) { register_button_handler("grip", std::move(handler)); }

// Start / Pause / Resume Handlers
void Teleop::set_start_handler(ActionHandler handler) { on_start_ = std::move(handler); }
void Teleop::set_pause_handler(ActionHandler handler) { on_pause_ = std::move(handler); }
void Teleop::set_resume_handler(ActionHandler handler) { on_resume_ = std::move(handler); }

// Pose Handlers
void Teleop::set_left_pose_handler(PoseHandler handler) { left_pose_handler_ = std::move(handler); }
void Teleop::set_right_pose_handler(PoseHandler handler) { right_pose_handler_ = std::move(handler); }

// Exit condition
void Teleop::set_exit_condition(ExitPredicate predicate) { exit_predicate_ = std::move(predicate); }

// Config
void Teleop::configure(const Config& config) {
    Config cfg = config;
    uint32_t flags = 0;
    if (!cfg.disable_left_controller) flags |= 0x01;
    if (!cfg.disable_right_controller) flags |= 0x02;
    if (cfg.join_information) flags |= 0x04;
    on_configure(flags);
}

Teleop::Config Teleop::get_config() const noexcept { return Config{}; }

void Teleop::notify_start() { if(on_start_) on_start_(); }
void Teleop::notify_pause() { if(on_pause_) on_pause_(); }
void Teleop::notify_resume() { if(on_resume_) on_resume_(); }

void Teleop::handle_pose(const VRPose& pose, const std::string& controller) const {
    if (controller == "left" && left_pose_handler_) left_pose_handler_(pose);
    else if (controller == "right" && right_pose_handler_) right_pose_handler_(pose);
}


bool Teleop::evaluate_button_states(const std::unordered_map<std::string, VRButtonValue>& buttons) {
    bool exit = false;
    for (const auto& [name, handler] : button_handlers_) {
        auto it = buttons.find(name);
        if (it != buttons.end()) {
            bool pressed = false;
            if (std::holds_alternative<bool>(it->second)) pressed = std::get<bool>(it->second);
            bool last = previous_button_states_[name];
            if (pressed && !last) handler();
            previous_button_states_[name] = pressed;
        }
    }

    if(exit_predicate_) exit = exit_predicate_(buttons);
    return exit;
}

void Teleop::register_button_handler(const std::string& button, ActionHandler handler) {
    button_handlers_[button] = std::move(handler);
}

} // namespace trossen_vr

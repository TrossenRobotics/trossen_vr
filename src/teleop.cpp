#include "trossen_vr/teleop.hpp"

#include <utility>

namespace trossen_vr {

void Teleop::on_button(const std::string& name, ButtonHandler handler) {
    button_handlers_[name] = std::move(handler);
}

void Teleop::on_analog(const std::string& name, AnalogHandler handler) {
    analog_handlers_[name] = std::move(handler);
}

void Teleop::on_right_pose(PoseHandler handler) {
    right_pose_handler_ = std::move(handler);
}

void Teleop::on_left_pose(PoseHandler handler) {
    left_pose_handler_ = std::move(handler);
}

void Teleop::dispatch(const VRFrame& frame) {
    // --- Button dispatch ---
    for (const auto& [name, value] : frame.buttons) {

        // Bool buttons → rising-edge detection
        if (std::holds_alternative<bool>(value)) {
            bool current = std::get<bool>(value);
            bool previous = prev_button_states_[name]; // defaults to false

            if (current && !previous) {
                auto it = button_handlers_.find(name);
                if (it != button_handlers_.end()) {
                    it->second();
                }
            }

            prev_button_states_[name] = current;
        }

        // Analog buttons → fire every frame with value
        else if (std::holds_alternative<double>(value)) {
            auto it = analog_handlers_.find(name);
            if (it != analog_handlers_.end()) {
                it->second(std::get<double>(value));
            }
        }
    }

    // --- Pose dispatch ---
    if (frame.right && right_pose_handler_) {
        right_pose_handler_(*frame.right);
    }
    if (frame.left && left_pose_handler_) {
        left_pose_handler_(*frame.left);
    }
}

} // namespace trossen_vr

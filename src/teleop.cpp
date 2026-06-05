#include "trossen_vr/teleop.hpp"

#include <utility>

namespace trossen_vr {

void Teleop::on_button_a(ButtonHandler handler) {
    button_a_handler_ = std::move(handler);
}

void Teleop::on_button_b(ButtonHandler handler) {
    button_b_handler_ = std::move(handler);
}

void Teleop::on_button_x(ButtonHandler handler) {
    button_x_handler_ = std::move(handler);
}

void Teleop::on_button_y(ButtonHandler handler) {
    button_y_handler_ = std::move(handler);
}

void Teleop::on_right_trigger(AnalogHandler handler) {
    right_trigger_handler_ = std::move(handler);
}

void Teleop::on_left_trigger(AnalogHandler handler) {
    left_trigger_handler_ = std::move(handler);
}

void Teleop::on_right_pose(PoseHandler handler) {
    right_pose_handler_ = std::move(handler);
}

void Teleop::on_left_pose(PoseHandler handler) {
    left_pose_handler_ = std::move(handler);
}

void Teleop::dispatch(const VRFrame& frame) {
    // Button A (right controller) - edge detection
    uint8_t curr_button_a = frame.right_controller.buttons.one;
    if (curr_button_a && !prev_button_a_) {
        if (button_a_handler_) {
            button_a_handler_();
        }
    }
    prev_button_a_ = curr_button_a;

    // Button B (right controller) - edge detection
    uint8_t curr_button_b = frame.right_controller.buttons.two;
    if (curr_button_b && !prev_button_b_) {
        if (button_b_handler_) {
            button_b_handler_();
        }
    }
    prev_button_b_ = curr_button_b;

    // Button X (left controller) - edge detection
    uint8_t curr_button_x = frame.left_controller.buttons.one;
    if (curr_button_x && !prev_button_x_) {
        if (button_x_handler_) {
            button_x_handler_();
        }
    }
    prev_button_x_ = curr_button_x;

    // Button Y (left controller) - edge detection
    uint8_t curr_button_y = frame.left_controller.buttons.two;
    if (curr_button_y && !prev_button_y_) {
        if (button_y_handler_) {
            button_y_handler_();
        }
    }
    prev_button_y_ = curr_button_y;

    // Analog inputs - fire every frame
    if (right_trigger_handler_) {
        right_trigger_handler_(frame.right_controller.triggers.index_trigger);
    }
    if (left_trigger_handler_) {
        left_trigger_handler_(frame.left_controller.triggers.index_trigger);
    }

    // Pose updates - fire when tracked
    if (frame.right_controller.is_tracked && right_pose_handler_) {
        right_pose_handler_(frame.right_controller.pose6d);
    }
    if (frame.left_controller.is_tracked && left_pose_handler_) {
        left_pose_handler_(frame.left_controller.pose6d);
    }
}

} // namespace trossen_vr

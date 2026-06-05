#ifndef TROSSEN_VR_TELEOP_HPP
#define TROSSEN_VR_TELEOP_HPP

#include <functional>

#include "trossen_vr/vr_types.hpp"

namespace trossen_vr {

/**
 * @brief Event-driven VR teleoperation dispatcher
 *
 * Register callback handlers for button presses, analog inputs, and controller poses. Digital
 * buttons use rising-edge detection (fire once per press). Analog inputs fire on every dispatch()
 * call; poses fire on every dispatch() call when the controller is tracked.
 *
 * @note Handler registration is not thread-safe - set all handlers before calling dispatch()
 */
class Teleop {
public:
    /// @brief Handler for controller pose updates
    using PoseHandler = std::function<void(const Pose6D&)>;

    /// @brief Handler for digital button presses (no arguments)
    using ButtonHandler = std::function<void()>;

    /// @brief Handler for analog inputs (receives value 0.0-1.0)
    using AnalogHandler = std::function<void(float)>;

    /**
     * @brief Register handler for button A (right controller)
     *
     * Handler fires once when button transitions from released to pressed (rising edge).
     *
     * @param handler Callback function, called on button press
     */
    void on_button_a(ButtonHandler handler);

    /**
     * @brief Register handler for button B (right controller)
     *
     * Handler fires once when button transitions from released to pressed (rising edge).
     *
     * @param handler Callback function, called on button press
     */
    void on_button_b(ButtonHandler handler);

    /**
     * @brief Register handler for button X (left controller)
     *
     * Handler fires once when button transitions from released to pressed (rising edge).
     *
     * @param handler Callback function, called on button press
     */
    void on_button_x(ButtonHandler handler);

    /**
     * @brief Register handler for button Y (left controller)
     *
     * Handler fires once when button transitions from released to pressed (rising edge).
     *
     * @param handler Callback function, called on button press
     */
    void on_button_y(ButtonHandler handler);

    /**
     * @brief Register handler for right index trigger
     *
     * Handler fires every dispatch() call with current trigger value.
     *
     * @param handler Callback function, receives value 0.0-1.0
     */
    void on_right_trigger(AnalogHandler handler);

    /**
     * @brief Register handler for left index trigger
     *
     * Handler fires every dispatch() call with current trigger value.
     *
     * @param handler Callback function, receives value 0.0-1.0
     */
    void on_left_trigger(AnalogHandler handler);

    /**
     * @brief Register handler for right controller pose updates
     *
     * Handler fires every dispatch() call when right controller is tracked.
     *
     * @param handler Callback function, receives Pose6D
     */
    void on_right_pose(PoseHandler handler);

    /**
     * @brief Register handler for left controller pose updates
     *
     * Handler fires every dispatch() call when left controller is tracked.
     *
     * @param handler Callback function, receives Pose6D
     */
    void on_left_pose(PoseHandler handler);

    /**
     * @brief Dispatch VR frame to all registered handlers
     *
     * Processes button state changes, analog inputs, and poses.
     * Call this once per frame with the latest VR data.
     *
     * @param frame VR frame data from NetworkManager
     */
    void dispatch(const VRFrame& frame);

private:
    // Button handlers
    ButtonHandler button_a_handler_;
    ButtonHandler button_b_handler_;
    ButtonHandler button_x_handler_;
    ButtonHandler button_y_handler_;

    // Analog handlers
    AnalogHandler right_trigger_handler_;
    AnalogHandler left_trigger_handler_;

    // Pose handlers
    PoseHandler right_pose_handler_;
    PoseHandler left_pose_handler_;

    // Previous button states for edge detection
    uint8_t prev_button_a_ = 0;
    uint8_t prev_button_b_ = 0;
    uint8_t prev_button_x_ = 0;
    uint8_t prev_button_y_ = 0;
};

} // namespace trossen_vr

#endif // TROSSEN_VR_TELEOP_HPP

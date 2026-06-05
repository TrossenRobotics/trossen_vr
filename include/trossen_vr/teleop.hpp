#ifndef TROSSEN_VR_TELEOP_HPP
#define TROSSEN_VR_TELEOP_HPP

#include <functional>
#include <string>
#include <unordered_map>

#include "trossen_vr/vr_types.hpp"

namespace trossen_vr {

/**
 * @brief Event-driven VR teleoperation dispatcher
 *
 * Register callback handlers for button presses, analog inputs, and controller poses.
 * Digital buttons use rising-edge detection (fire once per press).
 * Analog inputs and poses fire on every dispatch() call.
 *
 * @note Handler registration is not thread-safe - set all handlers before calling dispatch()
 */
class Teleop {
public:
    /// @brief Handler for controller pose updates
    using PoseHandler = std::function<void(const ControllerPose&)>;

    /// @brief Handler for digital button presses (no arguments)
    using ButtonHandler = std::function<void()>;

    /// @brief Handler for analog inputs (receives value 0.0-1.0)
    using AnalogHandler = std::function<void(double)>;

    /**
     * @brief Register handler for digital button press
     *
     * Handler fires once when button transitions from released to pressed (rising edge).
     *
     * @param name Button name (use ButtonNames constants)
     * @param handler Callback function, called on button press
     */
    void on_button(const std::string& name, ButtonHandler handler);

    /**
     * @brief Register handler for analog input
     *
     * Handler fires every dispatch() call with current analog value.
     *
     * @param name Input name (use ButtonNames constants)
     * @param handler Callback function, receives value 0.0-1.0
     */
    void on_analog(const std::string& name, AnalogHandler handler);

    /**
     * @brief Register handler for right controller pose updates
     *
     * Handler fires every dispatch() call when right controller is tracked.
     *
     * @param handler Callback function, receives ControllerPose
     */
    void on_right_pose(PoseHandler handler);

    /**
     * @brief Register handler for left controller pose updates
     *
     * Handler fires every dispatch() call when left controller is tracked.
     *
     * @param handler Callback function, receives ControllerPose
     */
    void on_left_pose(PoseHandler handler);

    /**
     * @brief Process VR frame and fire registered handlers
     *
     * Detects button state changes and invokes appropriate callbacks.
     * Safe to call from multiple threads, but handler registration is not thread-safe.
     *
     * @param frame VR frame to process
     */
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

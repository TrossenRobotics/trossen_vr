#ifndef Trossen_vr__include__teleop_HPP_
#define Trossen_vr__include__teleop_HPP_

#include "vr_types.hpp"

#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

namespace trossen_vr {
/**
 * @class Teleop
 * @brief Base class for mapping VR controller events to robot-specific actions.
 *
 * This abstract class provides a flexible framework for handling VR teleoperation by
 * registering callbacks for various VR events such as controller poses, button presses,
 * and connection state changes. Derived classes must implement robot-specific behavior
 * by overriding the on_configure method.
 *
 * The class supports:
 * - Button event handlers (A, B, trigger, grip)
 * - Pose tracking for left and right controllers, or unified pose tracking
 * - Connection state callbacks (start, pause, resume)
 * - Configurable controller settings via Config struct (enable/disable controllers, join information)
 * - Custom exit conditions based on button states
 * - Thread-safe outbound message queue for sending updates to VR system
 *
 * @note This is an abstract base class that requires derived classes to implement
 *       the on_configure method for robot-specific configuration.
 */
class Teleop {

public:
    using PoseHandler = std::function<void(const VRPose&)>;
    using ActionHandler = std::function<void()>;
    using ExitPredicate = std::function<bool(const std::unordered_map<std::string, VRButtonValue>&)>;

    virtual ~Teleop() = default;

    // Register button handlers for controls.
    void set_button_A_handler(ActionHandler handler);
    void set_button_B_handler(ActionHandler handler);
    void set_button_trigger_handler(ActionHandler handler);
    void set_button_grip_handler(ActionHandler handler);

    // Connection callbacks by VR commands.
    void set_start_handler(ActionHandler handler);
    void set_pause_handler(ActionHandler handler);
    void set_resume_handler(ActionHandler handler);

    // Pose callbacks
    void set_left_pose_handler(PoseHandler handler);
    void set_right_pose_handler(PoseHandler handler);
    void set_pose_handler(PoseHandler handler);
    void set_exit_condition(ExitPredicate predicate);

    struct Config {
        bool disable_left_controller = false;
        bool disable_right_controller = false;
        bool join_information = false;
    };

    void configure(const Config& config);
    Config get_config() const noexcept;

    // send connection events if the handler is present.
    void notify_start();
    void notify_pause();
    void notify_resume();

    // pose data and button states processes.
    void handle_pose(const VRPose& pose) const;
    bool evaluate_button_states(const std::unordered_map<std::string, VRButtonValue>& buttons);

    // Outbound mailbox polled by VRManager
    std::optional<std::string> get_pending_update() {
        std::lock_guard<std::mutex> lock(outbound_mutex_);
        return pending_update_;
    }

    void clear_pending_update() {
        std::lock_guard<std::mutex> lock(outbound_mutex_);
        pending_update_.reset();
    }

protected:
    virtual void on_configure(std::uint32_t flags) = 0;

    // Use this to post a JSON/serialized payload that VRManager will send.
    void post_update(const std::string& payload) {
        std::lock_guard<std::mutex> lock(outbound_mutex_);
        pending_update_ = payload;
    }

    void register_button_handler(const std::string& button, ActionHandler handler);

private:
    std::unordered_map<std::string, ActionHandler> button_handlers_;
    std::unordered_map<std::string, bool> previous_button_states_;
    PoseHandler left_pose_handler_;
    PoseHandler right_pose_handler_;
    PoseHandler pose_handler_;
    ActionHandler on_start_;
    ActionHandler on_pause_;
    ActionHandler on_resume_;
    ExitPredicate exit_predicate_;

    // Outbound mailbox
    std::optional<std::string> pending_update_;
    mutable std::mutex outbound_mutex_;
};

} // namespace trossen_vr

#endif // Trossen_vr__include__teleop_HPP_

#ifndef TROSSEN_VR__INCLUDE__TELEOP_HPP_
#define TROSSEN_VR__INCLUDE__TELEOP_HPP_


#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "trossen_vr/vr_types.hpp"

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

  /**
   * @brief Register a callback for the A button.
   * @param handler Function to invoke upon rising edge of the A button.
   */
  void set_button_A_handler(ActionHandler handler);

  /**
   * @brief Register a callback for the B button.
   * @param handler Function to invoke upon rising edge of the B button.
   */
  void set_button_B_handler(ActionHandler handler);

  /**
   * @brief Register a callback for the trigger button.
   * @param handler Function to invoke when the trigger is pressed.
   */
  void set_button_trigger_handler(ActionHandler handler);

  /**
   * @brief Register a callback for the grip button.
   * @param handler Function to invoke when the grip button is pressed.
   */
  void set_button_grip_handler(ActionHandler handler);


  /**
   * @brief Register callback invoked when a VR session begins.
   * @param handler Function to invoke on session start.
   */
  void set_start_handler(ActionHandler handler);

  /**
   * @brief Register callback invoked when teleoperation is paused.
   * @param handler Function to invoke when a pause command is detected.
   */
  void set_pause_handler(ActionHandler handler);

  /**
   * @brief Register callback invoked when teleoperation resumes.
   * @param handler Function to invoke when resuming from pause.
   */
  void set_resume_handler(ActionHandler handler);


  /**
   * @brief Set callback for left controller pose updates.
   * @param handler Function receiving VRPose for the left controller.
   */
  void set_left_pose_handler(PoseHandler handler);

  /**
   * @brief Set callback for right controller pose updates.
   * @param handler Function receiving VRPose for the right controller.
   */
  void set_right_pose_handler(PoseHandler handler);

  /**
   * @brief Set a callback invoked for all pose updates (left or right).
   * @param handler Function receiving every VRPose update.
   */
  void set_pose_handler(PoseHandler handler);

  /**
   * @brief Set user-defined exit condition predicate.
   * @param predicate Function evaluating button map and returning true
   *                  if teleoperation should terminate.
   */
  void set_exit_condition(ExitPredicate predicate);


  /**
   * @struct Config
   * @brief Configuration options for teleoperation behavior.
   *
   * @var disable_left_controller
   *      If true, left controller input will be ignored.
   *
   * @var disable_right_controller
   *      If true, right controller input will be ignored.
   *
   * @var join_information
   *      If true, left and right controller data may be treated as combined
   *      input by derived implementations.
   */
  struct Config {
    bool disable_left_controller{false};
    bool disable_right_controller{false};
    bool join_information{false};
  };

  /**
  * @brief Configure Teleop behavior with provided controller flags.
  *
  * Calls the derived class's on_configure() method with the appropriate bitmask
  * representing enabled/disabled controllers and join mode.
  *
  * @param config User-specified configuration settings.
  */
  void configure(const Config& config);

  /**
  * @brief Retrieve current Teleop configuration.
  * @return Config structure representing current settings.
  */
  Config get_config() const noexcept;


  // send connection events if the handler is present.
  void notify_start();
  void notify_pause();
  void notify_resume();

  /**
  * @brief Process an incoming VRPose update.
  *
  * Dispatches the pose to left/right/unified handlers depending on configuration.
  *
  * @param pose Pose data from the VR system.
  */
  void handle_pose(const VRPose& pose) const;

  /**
  * @brief Evaluate button states and dispatch handlers.
  *
  * Detects rising edges on all registered buttons, invokes associated handlers,
  * and evaluates the exit predicate if present.
  *
  * @param buttons Map of button names to VRButtonValue structs.
  * @return True if exit condition was met; otherwise false.
  */
  bool evaluate_button_states(
    const std::unordered_map<std::string, VRButtonValue>& buttons
  );


  /**
  * @brief Retrieve any pending outbound update.
  *
  * Thread-safe. The returned string is a snapshot; callers must
  * clear the update using clear_pending_update().
  *
  * @return Optional containing the pending message, or empty if none.
  */
  std::optional<std::string> get_pending_update() {
    std::lock_guard<std::mutex> lock(outbound_mutex_);
    return pending_update_;
  }

  /**
  * @brief Clear the outbound update buffer.
  *
  * Thread-safe. Removes any previously posted update.
  */
  void clear_pending_update() {
    std::lock_guard<std::mutex> lock(outbound_mutex_);
    pending_update_.reset();
  }

protected:
  /**
  * @brief Called during configuration to allow derived classes to set up
  *        robot-specific teleop behavior.
  *
  * @param flags Bitmask encoding configuration options.
  */
  virtual void on_configure(std::uint32_t flags) = 0;

  /**
  * @brief Post a serialized update to the outbound message buffer.
  *
  * Thread-safe. Overwrites any existing pending update.
  *
  * @param payload Serialized data (typically JSON).
  */
  void post_update(const std::string& payload) {
    std::lock_guard<std::mutex> lock(outbound_mutex_);
    pending_update_ = payload;
  }

  /**
  * @brief Internal helper to register a button handler under a string key.
  *
  * @param button Button identifier (e.g., "a", "b").
  * @param handler Callback to associate with the button.
  */
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

#endif // TROSSEN_VR__INCLUDE__TELEOP_HPP_

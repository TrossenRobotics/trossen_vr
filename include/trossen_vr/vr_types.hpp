#ifndef TROSSEN_VR__INCLUDE__VR_TYPES_HPP_
#define TROSSEN_VR__INCLUDE__VR_TYPES_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>

namespace trossen_vr {


/**
 * @struct VRPose
 * @brief Represents a 6-DoF pose consisting of position and rotation vectors.
 *
 * This structure stores the spatial pose of a VR controller or tracked object.
 * The `position` field contains a three-element array representing X, Y, Z 
 * translation in meters. The `rotation` field contains a three-element rotation 
 * vector, typically interpreted as an axis-angle representation where the vector 
 * direction encodes the axis of rotation and the magnitude encodes the angle.
 */
struct VRPose {
  std::array<double, 3> position{};
  std::array<double, 3> rotation{};
};

/**
 * @typedef VRButtonValue
 * @brief Variant type representing the state of a VR button.
 *
 * A button may report:
 * - A boolean value indicating pressed/released.
 * - A double value representing an analog value (e.g., trigger pressure).
 */
using VRButtonValue = std::variant<bool, double>;

/**
 * @enum VRCommand
 * @brief Enumerates high-level control commands emitted by the VR system.
 *
 * These commands are used for signaling teleoperation lifecycle changes, such as
 * starting, pausing, or resuming VR control.
 */
enum class VRCommand {
  Start,
  Pause,
  Resume
};

/**
 * @struct VRState
 * @brief Complete snapshot of VR input at a given moment.
 *
 * This structure represents the full set of data arriving from the VR system for
 * a single frame of input. It may contain controller poses, button states, system
 * commands, and a timestamp.
 *
 * Fields:
 * - `left_pose`: Optional pose of the left controller.
 * - `right_pose`: Optional pose of the right controller.
 * - `buttons`: Mapping of button names to their boolean or analog states.
 * - `command`: Optional high-level VR command (start, pause, resume).
 * - `timestamp`: Time at which the frame was generated.
 * - `sequence`: Monotonically increasing sequence ID for ordering frames.
 */
struct VRState {
  std::optional<VRPose> left_pose;
  std::optional<VRPose> right_pose;

  // button name - either a bool or a double
  std::unordered_map<std::string, VRButtonValue> buttons;

  std::optional<VRCommand> command;
  std::chrono::steady_clock::time_point timestamp{};
  std::uint64_t sequence{0};
};

}

#endif // TROSSEN_VR__INCLUDE__VR_TYPES_HPP_

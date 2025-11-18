#ifndef Trossen_vr__include__vr_types_HPP_  
#define Trossen_vr__include__vr_types_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>

namespace trossen_vr {

// Pose expressed as XYZ translation plus rotation vector.
struct VRPose {
    std::array<double, 3> position{};
    std::array<double, 3> rotation{};
};

// Button state carries value for press and optional analog value.
using VRButtonValue = std::variant<bool, double>;

enum class VRCommand {
    Start,
    Pause,
    Resume
};

struct VRInputFrame {
    std::optional<VRPose> left_pose;
    std::optional<VRPose> right_pose;

    // button name → either a bool or a double
    std::unordered_map<std::string, VRButtonValue> buttons;

    std::optional<VRCommand> command;
    std::chrono::steady_clock::time_point timestamp{};
    std::uint64_t sequence = 0;
};

}

#endif // Trossen_vr__include__vr_types_HPP_
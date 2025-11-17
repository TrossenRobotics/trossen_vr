#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

namespace trossen::meta_vr {

// Pose expressed as XYZ translation plus rotation vector.
struct VRPose {
    std::array<double, 3> position{};
    std::array<double, 3> rotation{};
};

// Button state carries value for press and optional analog value.
struct VRButtonState {
    bool pressed = false;
    double analog_value = 0.0;
};

// Packet received from VR rig.
struct VRInputFrame {
    std::optional<VRPose> left_pose;
    std::optional<VRPose> right_pose;
    std::unordered_map<std::string, VRButtonState> buttons;
    std::optional<std::string> command; // e.g., "start", "pause", "resume" (commands can include anything that needs to be done)
    std::chrono::steady_clock::time_point timestamp{};
    std::uint64_t sequence = 0;
};

}

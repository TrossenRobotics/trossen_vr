#ifndef TROSSEN_VR_VR_CONVERSIONS_HPP
#define TROSSEN_VR_VR_CONVERSIONS_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>

#include <nlohmann/json.hpp>

#include "trossen_vr/vr_types.hpp"

namespace trossen_vr {

/// @brief Constant pi for rotation conversions
constexpr double pi = 3.14159265358979323846;

/**
 * @brief Parse Pose6D from JSON object
 *
 * Expects JSON with fields: x, y, z, ax, ay, az (all floats).
 *
 * @param j JSON object containing pose data
 * @param p Output Pose6D struct
 * @throws nlohmann::json::exception if required fields are missing
 */
inline void from_json(const nlohmann::json& j, Pose6D& p) {
    p.x = static_cast<double>(j.at("x").get<float>());
    p.y = static_cast<double>(j.at("y").get<float>());
    p.z = static_cast<double>(j.at("z").get<float>());
    p.ax = static_cast<double>(j.at("ax").get<float>());
    p.ay = static_cast<double>(j.at("ay").get<float>());
    p.az = static_cast<double>(j.at("az").get<float>());
}

/**
 * @brief Parse Triggers from JSON object
 *

 * Expects JSON with fields: index_trigger (float).
 * Note: hand_trigger is parsed separately for deadman switch logic.
 *
 * @param j JSON object containing trigger data
 * @param t Output Triggers struct
 * @throws nlohmann::json::exception if required fields are missing
 */
inline void from_json(const nlohmann::json& j, Triggers& t) {
    t.index_trigger = static_cast<double>(j.at("index_trigger").get<float>());
}

/**
 * @brief Parse Thumbstick from JSON object
 *
 * Expects JSON with fields: x_axis, y_axis (floats).
 *
 * @param j JSON object containing thumbstick data
 * @param ts Output Thumbstick struct
 * @throws nlohmann::json::exception if required fields are missing
 */
inline void from_json(const nlohmann::json& j, Thumbstick& ts) {
    ts.x_axis = j.at("x_axis").get<float>();
    ts.y_axis = j.at("y_axis").get<float>();
}

/**
 * @brief Parse Buttons from JSON object
 *
 * Expects JSON with fields: one, two (int8/uint8).
 *
 * @param j JSON object containing button data
 * @param b Output Buttons struct
 * @throws nlohmann::json::exception if required fields are missing
 */
inline void from_json(const nlohmann::json& j, Buttons& b) {
    b.one = j.at("one").get<uint8_t>();
    b.two = j.at("two").get<uint8_t>();
}

/**
 * @brief Parse ControllerFrame from JSON object
 *
 * Parses tracking status and all controller inputs. If controller is not
 * tracked (is_tracked == 0), only tracking status is parsed to avoid
 * exceptions from missing fields.
 *
 * Expected JSON format:
 * {
 *   "is_tracked": 1,
 *   "pose6d": {...},
 *   "triggers": {...},
 *   "thumbstick": {...},
 *   "buttons": {...}
 * }
 *
 * @param j JSON object containing controller frame data
 * @param cf Output ControllerFrame struct
 * @throws nlohmann::json::exception if required fields are missing
 */
inline void from_json(const nlohmann::json& j, ControllerFrame& cf) {
    cf.is_tracked = j.at("is_tracked").get<uint8_t>();

    if (cf.is_tracked == 0) {
        return;
    }

    j.at("pose6d").get_to(cf.pose6d);
    j.at("triggers").get_to(cf.triggers);
    j.at("thumbstick").get_to(cf.thumbstick);
    j.at("buttons").get_to(cf.buttons);
}

/**
 * @brief Parse complete VRFrame from JSON packet data
 *
 * Parses both left and right controller states from JSON and applies
 * deadman switch logic based on grip threshold.
 * Handles missing or malformed controller data gracefully by logging
 * errors and leaving controller state at defaults.
 *
 * @param data JSON object containing VR frame data
 * @param grip_threshold Minimum grip trigger value required for tracking
 * @return Parsed VRFrame with controller states
 */
VRFrame parse_vr_frame(const nlohmann::json& data, double grip_threshold = 0.9);

/**
 * @brief Convert Pose6D to 4x4 transformation matrix
 *
 * Converts position and axis-angle rotation to homogeneous transform.
 *
 * @param pose6d Input 6-DOF pose (position + axis-angle)
 * @return 4x4 transformation matrix in SE(3)
 */
Transform4D pose6d_to_transform4d(const Pose6D& pose6d);

/**
 * @brief Convert 4x4 transformation matrix to Pose6D
 *
 * Extracts position and converts rotation matrix to axis-angle.
 *
 * @param transform4d Input 4x4 transformation matrix
 * @return 6-DOF pose (position + axis-angle)
 */
Pose6D transform4d_to_pose6d(const Transform4D& transform4d);

} // namespace trossen_vr

#endif // TROSSEN_VR_VR_CONVERSIONS_HPP

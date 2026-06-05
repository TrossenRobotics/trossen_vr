#ifndef TROSSEN_VR_VR_TYPES_HPP
#define TROSSEN_VR_VR_TYPES_HPP

#include <iostream>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

namespace trossen_vr {

/// @brief 6D pose vector: [x, y, z, rx, ry, rz]
/// Position in meters, rotation as axis-angle in radians
using Vec6 = Eigen::Matrix<double, 6, 1>;

/// @brief Button value variant - either bool (digital) or double (analog 0.0-1.0)
using ButtonValue = std::variant<bool, double>;

/// @brief VR controller pose (position + orientation)
struct ControllerPose {
    /// @brief Position vector in meters (x, y, z)
    Eigen::Vector3d position = Eigen::Vector3d::Zero();

    /// @brief Orientation as quaternion (w, x, y, z)
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
};

/// @brief Standard button names from Unity VR controller mapping
namespace ButtonNames {
    /// @brief Digital button A (bool)
    constexpr const char* A = "a";

    /// @brief Digital button B (bool)
    constexpr const char* B = "b";

    /// @brief Digital button X (bool)
    constexpr const char* X = "x";

    /// @brief Digital button Y (bool)
    constexpr const char* Y = "y";

    /// @brief Right index trigger (analog 0.0-1.0)
    constexpr const char* RightTrigger = "rightTrigger";

    /// @brief Left index trigger (analog 0.0-1.0)
    constexpr const char* LeftTrigger = "leftTrigger";

    /// @brief Right grip (analog 0.0-1.0)
    constexpr const char* RightGrip = "rightGrip";

    /// @brief Left grip (analog 0.0-1.0)
    constexpr const char* LeftGrip = "leftGrip";
}

/// @brief Complete VR frame with controller poses and button states
struct VRFrame {
    /// @brief Right controller pose (empty if not tracked)
    std::optional<ControllerPose> right;

    /// @brief Left controller pose (empty if not tracked)
    std::optional<ControllerPose> left;

    /// @brief Button states map (name -> ButtonValue)
    std::unordered_map<std::string, ButtonValue> buttons;

    /**
     * @brief Get digital button state
     *
     * @param name Button name (use ButtonNames constants)
     * @return Button pressed state, false if not found or wrong type
     */
    bool get_button(const std::string& name) const {
        auto it = buttons.find(name);
        if (it == buttons.end()) return false;
        if (const bool* val = std::get_if<bool>(&it->second)) {
            return *val;
        }
        return false;
    }

    /**
     * @brief Get analog input value
     *
     * @param name Input name (use ButtonNames constants)
     * @return Analog value 0.0-1.0, or 0.0 if not found or wrong type
     */
    double get_analog(const std::string& name) const {
        auto it = buttons.find(name);
        if (it == buttons.end()) return 0.0;
        if (const double* val = std::get_if<double>(&it->second)) {
            return *val;
        }
        return 0.0;
    }
};

/**
 * @brief Convert JSON {x, y, z} to Eigen::Vector3d
 *
 * @param j JSON object with x, y, z fields
 * @param v Output vector
 */
inline void from_json(const nlohmann::json& j, Eigen::Vector3d& v) {
    v.x() = j.at("x").get<double>();
    v.y() = j.at("y").get<double>();
    v.z() = j.at("z").get<double>();
}

/**
 * @brief Convert JSON {w, x, y, z} to Eigen::Quaterniond
 *
 * @param j JSON object with w, x, y, z fields
 * @param q Output quaternion
 */
inline void from_json(const nlohmann::json& j, Eigen::Quaterniond& q) {
    q.w() = j.at("w").get<double>();
    q.x() = j.at("x").get<double>();
    q.y() = j.at("y").get<double>();
    q.z() = j.at("z").get<double>();
}

/**
 * @brief Convert JSON button object to ButtonValue map
 *
 * @param j JSON object with button names as keys
 * @param buttons Output button state map
 */
inline void from_json(const nlohmann::json& j,
                     std::unordered_map<std::string, trossen_vr::ButtonValue>& buttons) {
    for (const auto& [key, val] : j.items()) {
        if (val.is_boolean()) {
            buttons[key] = val.get<bool>();
        } else if (val.is_number()) {
            buttons[key] = val.get<double>();
        }
    }
}

/**
 * @brief Convert 6D pose vector to 4x4 homogeneous transform
 *
 * @param v6 Pose vector [x, y, z, rx, ry, rz] in meters and radians
 * @return 4x4 transformation matrix
 */
inline Eigen::Matrix4d vec6_to_T(const Vec6& v6) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    T.block<3, 1>(0, 3) = v6.head<3>();

    Eigen::Vector3d rvec = v6.tail<3>();
    double angle = rvec.norm();
    if (angle > 1e-8) {
        Eigen::AngleAxisd aa(angle, rvec / angle);
        T.block<3, 3>(0, 0) = aa.toRotationMatrix();
    }

    return T;
}

/**
 * @brief Convert 4x4 homogeneous transform to 6D pose vector
 *
 * @param T 4x4 transformation matrix
 * @return Pose vector [x, y, z, rx, ry, rz] in meters and radians
 */
inline Vec6 T_to_vec6(const Eigen::Matrix4d& T) {
    Vec6 v6;
    v6.head<3>() = T.block<3, 1>(0, 3);

    Eigen::AngleAxisd aa(T.block<3, 3>(0, 0));
    v6.tail<3>() = aa.axis() * aa.angle();

    return v6;
}

/**
 * @brief Convert Unity VR pose to robot coordinate frame
 *
 * Transforms from Unity coordinates (right, up, forward) to robot frame (forward, left, up)
 *
 * @param pos Position in Unity frame (meters)
 * @param rot Rotation as quaternion in Unity frame
 * @return 6D pose in robot frame [x, y, z, rx, ry, rz]
 */
inline Vec6 unity_pose_to_vec6(const Eigen::Vector3d& pos,
                               const Eigen::Quaterniond& rot) {
    Vec6 v6;

    // Position remap
    v6[0] =  pos.z();   // forward
    v6[1] = -pos.x();   // left
    v6[2] =  pos.y();   // up

    // Rotation remap
    Eigen::AngleAxisd aa(rot);
    double angle = aa.angle();

    if (angle < 1e-8) {
        v6.tail<3>().setZero();
    } else {
        Eigen::Vector3d axis_unity = aa.axis();
        Eigen::Vector3d axis_robot(-axis_unity.z(), axis_unity.x(), -axis_unity.y());
        v6.tail<3>() = axis_robot * angle;
    }

    return v6;
}

/**
 * @brief Parse VR frame from JSON data
 *
 * Extracts controller poses and button states from Unity VR app UDP packet
 *
 * @param data JSON object from Unity app
 * @return Parsed VR frame
 */
inline VRFrame parse_vr_frame(const nlohmann::json& data) {
    VRFrame frame;

    // Parse right controller
    try {
        if (data.contains("rightPosition") && data.contains("rightRotation")) {
            ControllerPose pose;
            from_json(data["rightPosition"], pose.position);
            from_json(data["rightRotation"], pose.rotation);
            frame.right = pose;
        }
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "[VR] Failed to parse right controller: " << e.what() << std::endl;
    }

    // Parse left controller
    try {
        if (data.contains("leftPosition") && data.contains("leftRotation")) {
            ControllerPose pose;
            from_json(data["leftPosition"], pose.position);
            from_json(data["leftRotation"], pose.rotation);
            frame.left = pose;
        }
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "[VR] Failed to parse left controller: " << e.what() << std::endl;
    }

    // Parse button states
    try {
        if (data.contains("buttons") && data["buttons"].is_object()) {
            from_json(data["buttons"], frame.buttons);
        }
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "[VR] Failed to parse buttons: " << e.what() << std::endl;
    }

    return frame;
}

} // namespace trossen_vr

#endif // TROSSEN_VR_VR_TYPES_HPP

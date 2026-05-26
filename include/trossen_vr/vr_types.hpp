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

// Pose represented as 6-vector: [x, y, z, rx, ry, rz]
// Position in meters, rotation as axis-angle
using Vec6 = Eigen::Matrix<double, 6, 1>;

// A button value is either a bool (digital) or double (analog)
using ButtonValue = std::variant<bool, double>;

// Per-hand controller pose (position + rotation only)
struct ControllerPose {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
};

// Full VR frame: both controller poses + generic button map
struct VRFrame {
    std::optional<ControllerPose> right;
    std::optional<ControllerPose> left;
    std::unordered_map<std::string, ButtonValue> buttons;
};

// JSON conversion functions
// Convert JSON object {x, y, z} to Eigen::Vector3d
inline void from_json(const nlohmann::json& j, Eigen::Vector3d& v) {
    v.x() = j.at("x").get<double>();
    v.y() = j.at("y").get<double>();
    v.z() = j.at("z").get<double>();
}

// Convert JSON object {w, x, y, z} to Eigen::Quaterniond
inline void from_json(const nlohmann::json& j, Eigen::Quaterniond& q) {
    q.w() = j.at("w").get<double>();
    q.x() = j.at("x").get<double>();
    q.y() = j.at("y").get<double>();
    q.z() = j.at("z").get<double>();
}

// Convert JSON button object to button state map
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

// Convert a 6-vector to a 4x4 homogeneous transform
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

// Convert a 4x4 homogeneous transform to a 6-vector
inline Vec6 T_to_vec6(const Eigen::Matrix4d& T) {
    Vec6 v6;
    v6.head<3>() = T.block<3, 1>(0, 3);

    Eigen::AngleAxisd aa(T.block<3, 3>(0, 0));
    v6.tail<3>() = aa.axis() * aa.angle();

    return v6;
}

// Convert Unity pose to robot frame
// Position:  Unity (right, up, forward) to Robot (forward, left, up)
// Rotation:  axis remapped with same convention
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

// Parse a full VR frame from JSON sent by the Unity app
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

#pragma once

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>

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

    if (data.contains("rightPosition") && data.contains("rightRotation")) {
        const auto& rp = data["rightPosition"];
        const auto& rr = data["rightRotation"];
        if (rp.is_object() && rr.is_object()) {
            ControllerPose pose;
            pose.position = Eigen::Vector3d(rp["x"], rp["y"], rp["z"]);
            pose.rotation = Eigen::Quaterniond(rr["w"], rr["x"], rr["y"], rr["z"]);
            frame.right = pose;
        }
    }

    if (data.contains("leftPosition") && data.contains("leftRotation")) {
        const auto& lp = data["leftPosition"];
        const auto& lr = data["leftRotation"];
        if (lp.is_object() && lr.is_object()) {
            ControllerPose pose;
            pose.position = Eigen::Vector3d(lp["x"], lp["y"], lp["z"]);
            pose.rotation = Eigen::Quaterniond(lr["w"], lr["x"], lr["y"], lr["z"]);
            frame.left = pose;
        }
    }

    if (data.contains("buttons") && data["buttons"].is_object()) {
        for (auto& [key, val] : data["buttons"].items()) {
            if (val.is_boolean()) {
                frame.buttons[key] = val.get<bool>();
            } else if (val.is_number()) {
                frame.buttons[key] = val.get<double>();
            }
        }
    }

    return frame;
}

} // namespace trossen_vr

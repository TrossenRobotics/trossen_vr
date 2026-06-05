#include "trossen_vr/vr_conversions.hpp"

#include <algorithm>
#include <cmath>

namespace trossen_vr {

VRFrame parse_vr_frame(const nlohmann::json& data, double grip_threshold) {
    VRFrame frame;

    try {
        if (data.contains("right_controller") && data["right_controller"].is_object()) {
            data["right_controller"].get_to(frame.right_controller);

            // Deadman switch: check hand_trigger against threshold
            if (frame.right_controller.is_tracked && data["right_controller"].contains("triggers")) {
                double hand_trigger = static_cast<double>(
                    data["right_controller"]["triggers"].at("hand_trigger").get<float>()
                );
                if (hand_trigger < grip_threshold) {
                    frame.right_controller.is_tracked = 0;
                }
            }
        }
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "[VR] Failed to parse right_controller: " << e.what() << std::endl;
    }

    try {
        if (data.contains("left_controller") && data["left_controller"].is_object()) {
            data["left_controller"].get_to(frame.left_controller);

            // Deadman switch: check hand_trigger against threshold
            if (frame.left_controller.is_tracked && data["left_controller"].contains("triggers")) {
                double hand_trigger = static_cast<double>(
                    data["left_controller"]["triggers"].at("hand_trigger").get<float>()
                );
                if (hand_trigger < grip_threshold) {
                    frame.left_controller.is_tracked = 0;
                }
            }
        }
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "[VR] Failed to parse left_controller: " << e.what() << std::endl;
    }

    return frame;
}

Transform4D pose6d_to_transform4d(const Pose6D& pose6d) {
    Transform4D transform4d;

    double x = pose6d.x;
    double y = pose6d.y;
    double z = pose6d.z;
    double ax = pose6d.ax;
    double ay = pose6d.ay;
    double az = pose6d.az;

    double theta = std::sqrt(ax*ax + ay*ay + az*az);

    double *r11 = &transform4d.transform4d[0],  *r12 = &transform4d.transform4d[1],  *r13 = &transform4d.transform4d[2],  *r14 = &transform4d.transform4d[3];
    double *r21 = &transform4d.transform4d[4],  *r22 = &transform4d.transform4d[5],  *r23 = &transform4d.transform4d[6],  *r24 = &transform4d.transform4d[7];
    double *r31 = &transform4d.transform4d[8],  *r32 = &transform4d.transform4d[9],  *r33 = &transform4d.transform4d[10], *r34 = &transform4d.transform4d[11];
    double *r41 = &transform4d.transform4d[12], *r42 = &transform4d.transform4d[13], *r43 = &transform4d.transform4d[14], *r44 = &transform4d.transform4d[15];

    if (theta > 1e-9) {
        double ux = ax / theta;
        double uy = ay / theta;
        double uz = az / theta;

        double c = std::cos(theta);
        double s = std::sin(theta);
        double v = 1.0 - c;

        *r11 = ux*ux*v + c;    *r12 = ux*uy*v - uz*s; *r13 = ux*uz*v + uy*s;
        *r21 = uy*ux*v + uz*s; *r22 = uy*uy*v + c;    *r23 = uy*uz*v - ux*s;
        *r31 = uz*ux*v - uy*s; *r32 = uz*uy*v + ux*s; *r33 = uz*uz*v + c;
    }

    *r14 = x; *r24 = y; *r34 = z;
    return transform4d;
}

Pose6D transform4d_to_pose6d(const Transform4D& transform4d) {
    Pose6D pose6d;

    double r11 = transform4d.transform4d[0],  r12 = transform4d.transform4d[1],  r13 = transform4d.transform4d[2],  r14 = transform4d.transform4d[3];
    double r21 = transform4d.transform4d[4],  r22 = transform4d.transform4d[5],  r23 = transform4d.transform4d[6],  r24 = transform4d.transform4d[7];
    double r31 = transform4d.transform4d[8],  r32 = transform4d.transform4d[9],  r33 = transform4d.transform4d[10], r34 = transform4d.transform4d[11];

    pose6d.x = r14; pose6d.y = r24; pose6d.z = r34;

    double trace = r11 + r22 + r33;

    double cos_theta = (trace - 1.0) / 2.0;
    if (cos_theta > 1.0) cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;

    double theta = std::acos(cos_theta);

    if(theta > 1e-9) {
        if (theta < M_PI - 1e-6) {
            double sin_theta_2 = 2.0 * std::sin(theta);
            pose6d.ax = (r32 - r23) / sin_theta_2 * theta;
            pose6d.ay = (r13 - r31) / sin_theta_2 * theta;
            pose6d.az = (r21 - r12) / sin_theta_2 * theta;
        } else {
            // Singularity at 180 degrees: sin(theta) ≈ 0
            // Use diagonal elements to recover rotation axis
            double xx = (r11 + 1.0) / 2.0;
            double yy = (r22 + 1.0) / 2.0;
            double zz = (r33 + 1.0) / 2.0;

            double x_axis = std::sqrt(std::max(0.0, xx));
            double y_axis = std::sqrt(std::max(0.0, yy));
            double z_axis = std::sqrt(std::max(0.0, zz));

            if (r12 < 0.0) y_axis = -y_axis;
            if (r13 < 0.0) z_axis = -z_axis;

            pose6d.ax = x_axis * theta;
            pose6d.ay = y_axis * theta;
            pose6d.az = z_axis * theta;
        }
    }

    return pose6d;
}

} // namespace trossen_vr

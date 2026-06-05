#ifndef TROSSEN_VR_VR_TYPES_HPP
#define TROSSEN_VR_VR_TYPES_HPP

#include <array>
#include <cstdint>

namespace trossen_vr {

/**
 * @brief Connection status for VR data stream
 *
 * Represents the current state of bidirectional communication
 * between PC and VR headset.
 */
enum class ConnectionStatus {
    /// @brief No connection established or timed out
    Disconnected,

    /// @brief Initial connection attempt in progress
    Connecting,

    /// @brief Connection established and healthy
    Connected,

    /// @brief Connection active but message frequency is low
    Degraded
};

/**
 * @brief 6-DOF pose in robot coordinate system
 *
 * Position in meters and rotation as axis-angle in radians.
 * Axis-angle format: rotation axis direction scaled by rotation angle.
 */
struct Pose6D {
    /// @brief X-axis position in meters
    double x = 0.0;

    /// @brief Y-axis position in meters
    double y = 0.0;

    /// @brief Z-axis position in meters
    double z = 0.0;

    /// @brief X component of axis-angle rotation in radians
    double ax = 0.0;

    /// @brief Y component of axis-angle rotation in radians
    double ay = 0.0;

    /// @brief Z component of axis-angle rotation in radians
    double az = 0.0;
};

/**
 * @brief VR controller trigger states
 *
 * Index trigger value in [0.0, 1.0] range.
 * Note: Hand/grip trigger is used internally for deadman switch and not exposed.
 */
struct Triggers {
    /// @brief Index finger trigger
    double index_trigger = 0.0;
};

/**
 * @brief VR controller analog thumbstick states
 *
 * 2D analog stick axes values in [-1.0, 1.0] range.
 */
struct Thumbstick {
    /// @brief x axis
    float x_axis = 0.0f;

    /// @brief y axis
    float y_axis = 0.0f;
};

/**
 * @brief VR controller digital button states
 *
 * Binary button states. Button mapping depends on controller side:
 * - Right controller: one = A, two = B
 * - Left controller: one = X, two = Y
 */
struct Buttons {
    /// @brief Primary button: A (right) or X (left)
    uint8_t one = 0;

    /// @brief Secondary button: B (right) or Y (left)
    uint8_t two = 0;
};

/**
 * @brief Complete state of a single VR controller
 *
 * Contains tracking status, 6-DOF pose, and all input states
 * for controller.
 */
struct ControllerFrame {
    /// @brief Tracking status: 1 = tracked, 0 = not tracked
    uint8_t is_tracked = 0;

    /// @brief 6-DOF pose (position + rotation)
    Pose6D pose6d;

    /// @brief Analog Trigger states
    Triggers triggers;

    /// @brief Analog Thumbstick states
    Thumbstick thumbstick;

    /// @brief Digital button states
    Buttons buttons;
};

/**
 * @brief Complete VR frame with both controllers
 *
 * Represents the full state received from the VR headset in a single
 * network packet, including both left and right controller states.
 */
struct VRFrame {
    /// @brief Right controller state
    ControllerFrame right_controller;

    /// @brief Left controller state
    ControllerFrame left_controller;
};

/**
 * @brief 4x4 homogeneous transformation matrix
 *
 * Storage for SE(3) transformations.
 * Provides inverse() and matrix multiplication operations.
 */
struct Transform4D {
    std::array<double, 16> transform4d = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };

    /**
     * @brief Access matrix element at (row, col)
     */
    double& operator()(std::size_t row, std::size_t col) { return transform4d[row * 4 + col]; }

    /**
     * @brief Access matrix element at (row, col) - const version
     */
    const double& operator()(std::size_t row, std::size_t col) const { return transform4d[row * 4 + col]; }

    /**
     * @brief Compute matrix inverse
     *
     * Computes inverse of SE(3) matrix using structure:
     * inv([R t; 0 1]) = [R' -R't; 0 1]
     *
     * @return Inverted transformation matrix
     */
    Transform4D inverse() const {
        Transform4D inv;

        inv(0,0) = (*this)(0,0); inv(0,1) = (*this)(1,0); inv(0,2) = (*this)(2,0);
        inv(1,0) = (*this)(0,1); inv(1,1) = (*this)(1,1); inv(1,2) = (*this)(2,1);
        inv(2,0) = (*this)(0,2); inv(2,1) = (*this)(1,2); inv(2,2) = (*this)(2,2);

        double tx = (*this)(0,3);
        double ty = (*this)(1,3);
        double tz = (*this)(2,3);

        inv(0,3) = -(inv(0,0) * tx + inv(0,1) * ty + inv(0,2) * tz);
        inv(1,3) = -(inv(1,0) * tx + inv(1,1) * ty + inv(1,2) * tz);
        inv(2,3) = -(inv(2,0) * tx + inv(2,1) * ty + inv(2,2) * tz);

        inv(3,0) = 0.0; inv(3,1) = 0.0; inv(3,2) = 0.0; inv(3,3) = 1.0;

        return inv;
    }

    /**
     * @brief Matrix multiplication operator
     *
     * Computes (this * rhs) using standard 4x4 matrix multiplication.
     *
     * @param rhs Right-hand side transformation matrix
     * @return Result of matrix multiplication
     */
    Transform4D operator*(const Transform4D& rhs) const {
        Transform4D result;
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = 0; c < 4; ++c) {
                result(r, c) = (*this)(r, 0) * rhs(0, c) +
                               (*this)(r, 1) * rhs(1, c) +
                               (*this)(r, 2) * rhs(2, c) +
                               (*this)(r, 3) * rhs(3, c);
            }
        }
        result(3, 0) = 0.0; result(3, 1) = 0.0; result(3, 2) = 0.0; result(3, 3) = 1.0;
        return result;
    }
};

} // namespace trossen_vr

#endif // TROSSEN_VR_VR_TYPES_HPP

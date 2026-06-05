#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "trossen_vr/network_manager.hpp"
#include "trossen_vr/teleop.hpp"
#include "trossen_vr/vr_conversions.hpp"
#include "trossen_vr/vr_types.hpp"

namespace py = pybind11;

PYBIND11_MODULE(trossen_vr, m) {
    m.doc() = "Trossen VR teleoperation library for Meta Quest controllers";

    // ConnectionStatus enum
    py::enum_<trossen_vr::ConnectionStatus>(m, "ConnectionStatus",
        "Connection status for VR data stream")
        .value("Disconnected", trossen_vr::ConnectionStatus::Disconnected,
            "No connection established or timed out")
        .value("Connecting", trossen_vr::ConnectionStatus::Connecting,
            "Initial connection attempt in progress")
        .value("Connected", trossen_vr::ConnectionStatus::Connected,
            "Connection established and healthy")
        .value("Degraded", trossen_vr::ConnectionStatus::Degraded,
            "Connection active but message frequency is low")
        .export_values();

    // Pose6D
    py::class_<trossen_vr::Pose6D>(m, "Pose6D",
        "6D pose: position (x, y, z) + axis-angle rotation (ax, ay, az)")
        .def(py::init<>())
        .def_readwrite("x", &trossen_vr::Pose6D::x, "X position in meters")
        .def_readwrite("y", &trossen_vr::Pose6D::y, "Y position in meters")
        .def_readwrite("z", &trossen_vr::Pose6D::z, "Z position in meters")
        .def_readwrite("ax", &trossen_vr::Pose6D::ax, "Rotation axis-angle X component in radians")
        .def_readwrite("ay", &trossen_vr::Pose6D::ay, "Rotation axis-angle Y component in radians")
        .def_readwrite("az", &trossen_vr::Pose6D::az, "Rotation axis-angle Z component in radians");

    // Triggers
    py::class_<trossen_vr::Triggers>(m, "Triggers",
        "Controller trigger values (0.0 to 1.0)")
        .def(py::init<>())
        .def_readwrite("index_trigger", &trossen_vr::Triggers::index_trigger,
            "Index finger trigger value (0.0 = released, 1.0 = fully pressed)");

    // Buttons
    py::class_<trossen_vr::Buttons>(m, "Buttons",
        "Controller button states")
        .def(py::init<>())
        .def_readwrite("one", &trossen_vr::Buttons::one, "Primary button: A (right) or X (left)")
        .def_readwrite("two", &trossen_vr::Buttons::two, "Secondary button: B (right) or Y (left)");

    // ControllerFrame
    py::class_<trossen_vr::ControllerFrame>(m, "ControllerFrame",
        "Complete state of a single VR controller")
        .def(py::init<>())
        .def_readwrite("pose6d", &trossen_vr::ControllerFrame::pose6d,
            "Controller 6D pose")
        .def_readwrite("triggers", &trossen_vr::ControllerFrame::triggers,
            "Trigger values")
        .def_readwrite("buttons", &trossen_vr::ControllerFrame::buttons,
            "Button states")
        .def_readwrite("is_tracked", &trossen_vr::ControllerFrame::is_tracked,
            "Tracking status: 1 = tracked (hand trigger held), 0 = not tracked");

    // VRFrame
    py::class_<trossen_vr::VRFrame>(m, "VRFrame",
        "Complete VR frame with both controller states")
        .def(py::init<>())
        .def_readwrite("right_controller", &trossen_vr::VRFrame::right_controller,
            "Right controller state")
        .def_readwrite("left_controller", &trossen_vr::VRFrame::left_controller,
            "Left controller state");

    // Transform4D
    py::class_<trossen_vr::Transform4D>(m, "Transform4D",
        "4x4 homogeneous transformation matrix (SE(3))")
        .def(py::init<>())
        .def_readwrite("transform4d", &trossen_vr::Transform4D::transform4d,
            "16-element array in row-major order")
        .def("inverse", &trossen_vr::Transform4D::inverse,
            "Compute inverse transformation\\n\\n"
            "Returns:\\n"
            "    Transform4D: Inverse transform")
        .def("__mul__", &trossen_vr::Transform4D::operator*,
            py::arg("other"),
            "Multiply two transforms (composition)\\n\\n"
            "Args:\\n"
            "    other: Transform to compose with\\n\\n"
            "Returns:\\n"
            "    Transform4D: Composed transformation");

    // ReceiverConfig
    py::class_<trossen_vr::ReceiverConfig>(m, "ReceiverConfig",
        "Network receiver configuration")
        .def(py::init<>())
        .def_readwrite("port", &trossen_vr::ReceiverConfig::port,
            "Network port to listen on (default: 9000)")
        .def_readwrite("buffer_size", &trossen_vr::ReceiverConfig::buffer_size,
            "Receive buffer size in bytes (default: 2048)")
        .def_readwrite("timeout_seconds", &trossen_vr::ReceiverConfig::timeout_seconds,
            "Connection timeout in seconds (default: 2.0)")
        .def_readwrite("min_frequency_hz", &trossen_vr::ReceiverConfig::min_frequency_hz,
            "Minimum expected message frequency in Hz (default: 30.0)")
        .def_readwrite("ack_port", &trossen_vr::ReceiverConfig::ack_port,
            "Port to send ACK packets to VR app (default: 9001)")
        .def_readwrite("grip_threshold", &trossen_vr::ReceiverConfig::grip_threshold,
            "Hand trigger threshold for tracking deadman switch (default: 0.9)");

    // NetworkManager
    py::class_<trossen_vr::NetworkManager>(m, "NetworkManager",
        "Network manager for VR controller data\\n\\n"
        "Runs a background thread that continuously receives packets and maintains\\n"
        "the latest VR frame. Thread-safe access via latest_frame().")
        .def(py::init<const trossen_vr::ReceiverConfig&>(),
             py::arg("config") = trossen_vr::ReceiverConfig{},
             "Construct network manager\\n\\n"
             "Args:\\n"
             "    config: Receiver configuration")
        .def("start", &trossen_vr::NetworkManager::start,
            "Start receiving VR data on background thread\\n\\n"
            "Binds to the configured port and starts packet reception.\\n"
            "Raises RuntimeError if socket creation or binding fails.")
        .def("stop", &trossen_vr::NetworkManager::stop,
            "Stop receiving and shutdown background thread\\n\\n"
            "Safe to call multiple times.")
        .def("latest_frame", &trossen_vr::NetworkManager::latest_frame,
            "Get the most recent VR frame\\n\\n"
            "Thread-safe. Returns None if no frame received yet.\\n\\n"
            "Returns:\\n"
            "    VRFrame or None: Latest VR frame")
        .def("is_running", &trossen_vr::NetworkManager::is_running,
            "Check if receiver thread is running\\n\\n"
            "Returns:\\n"
            "    bool: True if background thread is active")
        .def("get_connection_status", &trossen_vr::NetworkManager::get_connection_status,
            "Get current connection status\\n\\n"
            "Thread-safe. Status is automatically updated based on message reception\\n"
            "timing and frequency.\\n\\n"
            "Returns:\\n"
            "    ConnectionStatus: Current connection status")
        .def("get_message_frequency", &trossen_vr::NetworkManager::get_message_frequency,
            "Get current message reception frequency in Hz\\n\\n"
            "Thread-safe. Calculated over a rolling 1-second window.\\n\\n"
            "Returns:\\n"
            "    float: Frequency in Hz, or 0.0 if no messages received yet");

    // Teleop
    py::class_<trossen_vr::Teleop>(m, "Teleop",
        "Event-driven VR teleoperation dispatcher\\n\\n"
        "Register callback handlers for button presses, analog inputs, and controller poses.\\n"
        "Digital buttons use rising-edge detection (fire once per press).\\n"
        "Analog inputs and poses fire on every dispatch() call when controller is tracked.\\n\\n"
        "Note: Handler registration is not thread-safe - set all handlers before calling dispatch()")
        .def(py::init<>())
        .def("on_button_a", &trossen_vr::Teleop::on_button_a,
            py::arg("handler"),
            "Register handler for A button press (right controller)\\n\\n"
            "Handler fires once when button transitions from released to pressed.\\n\\n"
            "Args:\\n"
            "    handler: Callback function with no arguments")
        .def("on_button_b", &trossen_vr::Teleop::on_button_b,
            py::arg("handler"),
            "Register handler for B button press (right controller)\\n\\n"
            "Handler fires once when button transitions from released to pressed.\\n\\n"
            "Args:\\n"
            "    handler: Callback function with no arguments")
        .def("on_button_x", &trossen_vr::Teleop::on_button_x,
            py::arg("handler"),
            "Register handler for X button press (left controller)\\n\\n"
            "Handler fires once when button transitions from released to pressed.\\n\\n"
            "Args:\\n"
            "    handler: Callback function with no arguments")
        .def("on_button_y", &trossen_vr::Teleop::on_button_y,
            py::arg("handler"),
            "Register handler for Y button press (left controller)\\n\\n"
            "Handler fires once when button transitions from released to pressed.\\n\\n"
            "Args:\\n"
            "    handler: Callback function with no arguments")
        .def("on_right_trigger", &trossen_vr::Teleop::on_right_trigger,
            py::arg("handler"),
            "Register handler for right index trigger\\n\\n"
            "Handler fires every dispatch() call with current trigger value.\\n\\n"
            "Args:\\n"
            "    handler: Callback function receiving float value 0.0-1.0")
        .def("on_left_trigger", &trossen_vr::Teleop::on_left_trigger,
            py::arg("handler"),
            "Register handler for left index trigger\\n\\n"
            "Handler fires every dispatch() call with current trigger value.\\n\\n"
            "Args:\\n"
            "    handler: Callback function receiving float value 0.0-1.0")
        .def("on_right_pose", &trossen_vr::Teleop::on_right_pose,
            py::arg("handler"),
            "Register handler for right controller pose updates\\n\\n"
            "Handler fires every dispatch() call when right controller is tracked (deadman switch held).\\n\\n"
            "Args:\\n"
            "    handler: Callback function receiving Pose6D")
        .def("on_left_pose", &trossen_vr::Teleop::on_left_pose,
            py::arg("handler"),
            "Register handler for left controller pose updates\\n\\n"
            "Handler fires every dispatch() call when left controller is tracked (deadman switch held).\\n\\n"
            "Args:\\n"
            "    handler: Callback function receiving Pose6D")
        .def("dispatch", &trossen_vr::Teleop::dispatch,
            py::arg("frame"),
            "Process VR frame and fire registered handlers\\n\\n"
            "Processes button state changes (rising edge detection), analog inputs (continuous),\\n"
            "and controller poses (continuous, only when tracked via deadman switch).\\n"
            "Call this once per frame with the latest VR data.\\n\\n"
            "Args:\\n"
            "    frame: VR frame to process");

    // Conversion functions
    m.def("pose6d_to_transform4d", &trossen_vr::pose6d_to_transform4d,
        py::arg("pose"),
        "Convert 6D pose to 4x4 homogeneous transformation matrix\\n\\n"
        "Uses Rodrigues formula for axis-angle to rotation matrix conversion.\\n\\n"
        "Args:\\n"
        "    pose: Pose6D with position and axis-angle rotation\\n\\n"
        "Returns:\\n"
        "    Transform4D: 4x4 transformation matrix");

    m.def("transform4d_to_pose6d", &trossen_vr::transform4d_to_pose6d,
        py::arg("transform"),
        "Convert 4x4 homogeneous transformation matrix to 6D pose\\n\\n"
        "Extracts position and converts rotation matrix to axis-angle.\\n\\n"
        "Args:\\n"
        "    transform: Transform4D (4x4 matrix)\\n\\n"
        "Returns:\\n"
        "    Pose6D: 6D pose with position and axis-angle rotation");
}

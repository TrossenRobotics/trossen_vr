#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "trossen_vr/network_manager.hpp"
#include "trossen_vr/teleop.hpp"
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

    // ControllerPose
    py::class_<trossen_vr::ControllerPose>(m, "ControllerPose",
        "VR controller pose (position + orientation)")
        .def(py::init<>())
        .def_readwrite("position", &trossen_vr::ControllerPose::position,
            "Position vector in meters (x, y, z)")
        .def_property("rotation",
            [](const trossen_vr::ControllerPose& self) -> Eigen::Vector4d {
                const auto& q = self.rotation;
                return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
            },
            [](trossen_vr::ControllerPose& self, const Eigen::Vector4d& v) {
                self.rotation = Eigen::Quaterniond(v[0], v[1], v[2], v[3]);
            },
            "Orientation as quaternion [w, x, y, z]");

    // VRFrame
    py::class_<trossen_vr::VRFrame>(m, "VRFrame",
        "Complete VR frame with controller poses and button states")
        .def(py::init<>())
        .def_readwrite("right", &trossen_vr::VRFrame::right,
            "Right controller pose (None if not tracked)")
        .def_readwrite("left", &trossen_vr::VRFrame::left,
            "Left controller pose (None if not tracked)")
        .def_readwrite("buttons", &trossen_vr::VRFrame::buttons,
            "Button states map: name -> bool or float")
        .def("get_button", &trossen_vr::VRFrame::get_button,
            py::arg("name"),
            "Get digital button state\\n\\n"
            "Args:\\n"
            "    name: Button name (use ButtonNames constants)\\n\\n"
            "Returns:\\n"
            "    bool: Button pressed state, False if not found or wrong type")
        .def("get_analog", &trossen_vr::VRFrame::get_analog,
            py::arg("name"),
            "Get analog input value\\n\\n"
            "Args:\\n"
            "    name: Input name (use ButtonNames constants)\\n\\n"
            "Returns:\\n"
            "    float: Analog value 0.0-1.0, or 0.0 if not found or wrong type");

    // ReceiverConfig
    py::class_<trossen_vr::ReceiverConfig>(m, "ReceiverConfig",
        "UDP receiver configuration")
        .def(py::init<>())
        .def_readwrite("port", &trossen_vr::ReceiverConfig::port,
            "UDP port to listen on (default: 9000)")
        .def_readwrite("buffer_size", &trossen_vr::ReceiverConfig::buffer_size,
            "UDP receive buffer size in bytes (default: 2048)")
        .def_readwrite("timeout_seconds", &trossen_vr::ReceiverConfig::timeout_seconds,
            "Connection timeout in seconds (default: 2.0)")
        .def_readwrite("min_frequency_hz", &trossen_vr::ReceiverConfig::min_frequency_hz,
            "Minimum expected message frequency in Hz (default: 30.0)")
        .def_readwrite("loss_window", &trossen_vr::ReceiverConfig::loss_window,
            "Window size for packet loss tracking (default: 100)")
        .def_readwrite("ack_port", &trossen_vr::ReceiverConfig::ack_port,
            "UDP port to send ACK packets to VR app (default: 9001)");

    // NetworkManager
    py::class_<trossen_vr::NetworkManager>(m, "NetworkManager",
        "Network manager for VR controller data\n\n"
        "Runs a background thread that continuously receives UDP packets and maintains\n"
        "the latest VR frame. Thread-safe access via latest_frame().")
        .def(py::init<const trossen_vr::ReceiverConfig&>(),
             py::arg("config") = trossen_vr::ReceiverConfig{},
             "Construct network manager\n\n"
             "Args:\n"
             "    config: Receiver configuration (port and buffer size)")
        .def("start", &trossen_vr::NetworkManager::start,
            "Start receiving VR data on background thread\\n\\n"
            "Binds to the configured UDP port and starts packet reception.\\n"
            "Raises RuntimeError if socket creation or binding fails.")
        .def("stop", &trossen_vr::NetworkManager::stop,
            "Stop receiving and shutdown background thread\n\n"
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
            "Thread-safe. Calculated over a rolling window.\\n\\n"
            "Returns:\\n"
            "    float: Frequency in Hz, or 0.0 if no messages received yet")
        .def("get_packet_loss_rate", &trossen_vr::NetworkManager::get_packet_loss_rate,
            "Get packet loss rate over recent window\\n\\n"
            "Thread-safe. Based on expected vs received message count.\\n\\n"
            "Returns:\\n"
            "    float: Loss rate between 0.0 (no loss) and 1.0 (100% loss)");

    // Teleop
    py::class_<trossen_vr::Teleop>(m, "Teleop",
        "Event-driven VR teleoperation dispatcher\\n\\n"
        "Register callback handlers for button presses, analog inputs, and controller poses.\\n"
        "Digital buttons use rising-edge detection (fire once per press).\\n"
        "Analog inputs and poses fire on every dispatch() call.\n\n"
        "Note: Handler registration is not thread-safe - set all handlers before calling dispatch()")
        .def(py::init<>())
        .def("on_button", &trossen_vr::Teleop::on_button,
            py::arg("name"), py::arg("handler"),
            "Register handler for digital button press\\n\\n"
            "Handler fires once when button transitions from released to pressed (rising edge).\\n\\n"
            "Args:\\n"
            "    name: Button name (use ButtonNames constants)\\n"
            "    handler: Callback function with no arguments")
        .def("on_analog", &trossen_vr::Teleop::on_analog,
            py::arg("name"), py::arg("handler"),
            "Register handler for analog input\\n\\n"
            "Handler fires every dispatch() call with current analog value.\\n\\n"
            "Args:\\n"
            "    name: Input name (use ButtonNames constants)\\n"
            "    handler: Callback function receiving float value 0.0-1.0")
        .def("on_right_pose", &trossen_vr::Teleop::on_right_pose,
            py::arg("handler"),
            "Register handler for right controller pose updates\\n\\n"
            "Handler fires every dispatch() call when right controller is tracked.\\n\\n"
            "Args:\\n"
            "    handler: Callback function receiving ControllerPose")
        .def("on_left_pose", &trossen_vr::Teleop::on_left_pose,
            py::arg("handler"),
            "Register handler for left controller pose updates\\n\\n"
            "Handler fires every dispatch() call when left controller is tracked.\\n\\n"
            "Args:\\n"
            "    handler: Callback function receiving ControllerPose")
        .def("dispatch", &trossen_vr::Teleop::dispatch,
            py::arg("frame"),
            "Process VR frame and fire registered handlers\\n\\n"
            "Detects button state changes and invokes appropriate callbacks.\\n"
            "Safe to call from multiple threads, but handler registration is not thread-safe.\\n\\n"
            "Args:\\n"
            "    frame: VR frame to process");

    // ButtonNames constants
    auto button_names = m.def_submodule("ButtonNames",
        "Standard button names from Unity VR controller mapping");
    button_names.attr("A") = trossen_vr::ButtonNames::A;
    button_names.attr("B") = trossen_vr::ButtonNames::B;
    button_names.attr("X") = trossen_vr::ButtonNames::X;
    button_names.attr("Y") = trossen_vr::ButtonNames::Y;
    button_names.attr("RightTrigger") = trossen_vr::ButtonNames::RightTrigger;
    button_names.attr("LeftTrigger") = trossen_vr::ButtonNames::LeftTrigger;
    button_names.attr("RightGrip") = trossen_vr::ButtonNames::RightGrip;
    button_names.attr("LeftGrip") = trossen_vr::ButtonNames::LeftGrip;
    button_names.attr("RightThumbstickX") = trossen_vr::ButtonNames::RightThumbstickX;
    button_names.attr("RightThumbstickY") = trossen_vr::ButtonNames::RightThumbstickY;
    button_names.attr("LeftThumbstickX") = trossen_vr::ButtonNames::LeftThumbstickX;
    button_names.attr("LeftThumbstickY") = trossen_vr::ButtonNames::LeftThumbstickY;

    // Free functions
    m.def("vec6_to_T", &trossen_vr::vec6_to_T,
        py::arg("v6"),
        "Convert 6D pose vector to 4x4 homogeneous transform\\n\\n"
        "Args:\\n"
        "    v6: Pose vector [x, y, z, rx, ry, rz] in meters and radians\\n\\n"
        "Returns:\\n"
        "    numpy.ndarray: 4x4 transformation matrix");
    m.def("T_to_vec6", &trossen_vr::T_to_vec6,
        py::arg("T"),
        "Convert 4x4 homogeneous transform to 6D pose vector\\n\\n"
        "Args:\\n"
        "    T: 4x4 transformation matrix\\n\\n"
        "Returns:\\n"
        "    numpy.ndarray: Pose vector [x, y, z, rx, ry, rz] in meters and radians");
    m.def("unity_pose_to_vec6",
        [](const Eigen::Vector3d& pos, const Eigen::Vector4d& rot) {
            return trossen_vr::unity_pose_to_vec6(
                pos, Eigen::Quaterniond(rot[0], rot[1], rot[2], rot[3]));
        },
        py::arg("pos"), py::arg("rot"),
        "Convert Unity VR pose to robot coordinate frame\\n\\n"
        "Transforms from Unity coordinates (right, up, forward) to robot frame (forward, left, up)\\n\\n"
        "Args:\\n"
        "    pos: Position in Unity frame (meters)\\n"
        "    rot: Rotation as quaternion [w, x, y, z] in Unity frame\\n\\n"
        "Returns:\\n"
        "    numpy.ndarray: 6D pose in robot frame [x, y, z, rx, ry, rz]");
}

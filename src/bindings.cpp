#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>

#include "trossen_vr/vr_types.hpp"
#include "trossen_vr/network_manager.hpp"
#include "trossen_vr/teleop.hpp"

namespace py = pybind11;

PYBIND11_MODULE(trossen_vr, m) {
    m.doc() = "Trossen VR teleoperation library";

    // ControllerPose
    py::class_<trossen_vr::ControllerPose>(m, "ControllerPose")
        .def(py::init<>())
        .def_readwrite("position", &trossen_vr::ControllerPose::position)
        .def_readwrite("rotation", &trossen_vr::ControllerPose::rotation);

    // VRFrame
    py::class_<trossen_vr::VRFrame>(m, "VRFrame")
        .def(py::init<>())
        .def_readwrite("right", &trossen_vr::VRFrame::right)
        .def_readwrite("left", &trossen_vr::VRFrame::left)
        .def_readwrite("buttons", &trossen_vr::VRFrame::buttons);

    // ReceiverConfig
    py::class_<trossen_vr::ReceiverConfig>(m, "ReceiverConfig")
        .def(py::init<>())
        .def_readwrite("port", &trossen_vr::ReceiverConfig::port)
        .def_readwrite("buffer_size", &trossen_vr::ReceiverConfig::buffer_size);

    // UDPReceiver
    py::class_<trossen_vr::UDPReceiver>(m, "UDPReceiver")
        .def(py::init<const trossen_vr::ReceiverConfig&>(),
             py::arg("config") = trossen_vr::ReceiverConfig{})
        .def("start", &trossen_vr::UDPReceiver::start)
        .def("stop", &trossen_vr::UDPReceiver::stop)
        .def("latest_frame", &trossen_vr::UDPReceiver::latest_frame)
        .def("is_running", &trossen_vr::UDPReceiver::is_running);

    // TeleopConfig
    py::class_<trossen_vr::TeleopConfig>(m, "TeleopConfig")
        .def(py::init<>())
        .def_readwrite("right_arm_ip", &trossen_vr::TeleopConfig::right_arm_ip)
        .def_readwrite("left_arm_ip", &trossen_vr::TeleopConfig::left_arm_ip)
        .def_readwrite("send_rate_hz", &trossen_vr::TeleopConfig::send_rate_hz)
        .def_readwrite("gripper_max_m", &trossen_vr::TeleopConfig::gripper_max_m)
        .def_readwrite("cmd_goal_time", &trossen_vr::TeleopConfig::cmd_goal_time);

    // Teleop
    py::class_<trossen_vr::Teleop>(m, "Teleop")
        .def(py::init<>())
        .def("on_button", &trossen_vr::Teleop::on_button)
        .def("on_analog", &trossen_vr::Teleop::on_analog)
        .def("on_right_pose", &trossen_vr::Teleop::on_right_pose)
        .def("on_left_pose", &trossen_vr::Teleop::on_left_pose)
        .def("dispatch", &trossen_vr::Teleop::dispatch);

    // Free functions
    m.def("vec6_to_T", &trossen_vr::vec6_to_T);
    m.def("T_to_vec6", &trossen_vr::T_to_vec6);
    m.def("unity_pose_to_vec6", &trossen_vr::unity_pose_to_vec6);
}

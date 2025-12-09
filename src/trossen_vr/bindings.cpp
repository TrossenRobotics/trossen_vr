#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/stl_bind.h>

#include "trossen_vr/vr_types.hpp"
#include "trossen_vr/vr_manager.hpp"
#include "trossen_vr/teleop.hpp"

namespace py = pybind11;
using namespace trossen_vr;

PYBIND11_MODULE(pytrossen_vr, m) {
    m.doc() = "Python bindings for Trossen VR SDK";

    // -------------------------
    // VRPose
    // -------------------------
    py::class_<VRPose>(m, "VRPose")
        .def(py::init<>())
        .def_readwrite("position", &VRPose::position)
        .def_readwrite("rotation", &VRPose::rotation);

    // -------------------------
    // VRCommand
    // -------------------------
    py::enum_<VRCommand>(m, "VRCommand")
        .value("Start", VRCommand::Start)
        .value("Pause", VRCommand::Pause)
        .value("Resume", VRCommand::Resume)
        .export_values();

    // -------------------------
    // VRState
    // -------------------------
    py::class_<VRState>(m, "VRState")
        .def(py::init<>())
        .def_readwrite("left_pose", &VRState::left_pose)
        .def_readwrite("right_pose", &VRState::right_pose)
        .def_readwrite("buttons", &VRState::buttons)
        .def_readwrite("command", &VRState::command)
        .def_readwrite("timestamp", &VRState::timestamp)
        .def_readwrite("sequence", &VRState::sequence);

    // -------------------------
    // VRButtonValue variant helper
    // -------------------------
    // py::implicitly_convertible<bool, VRButtonValue>();
    // py::implicitly_convertible<double, VRButtonValue>();

    // -------------------------
    // VRManager::Config
    // -------------------------
    py::class_<VRManager::Config>(m, "VRManagerConfig")
        .def(py::init<>())
        .def_readwrite("server_port", &VRManager::Config::server_port)
        .def_readwrite("read_timeout", &VRManager::Config::read_timeout)
        .def_readwrite("reconnect_delay", &VRManager::Config::reconnect_delay);

    // -------------------------
    // VRManager
    // -------------------------
    py::class_<VRManager>(m, "VRManager")
        .def(py::init<const VRManager::Config&>())
        .def("start", &VRManager::start)
        .def("stop", &VRManager::stop)
        .def("is_active", &VRManager::is_active)
        .def("is_connected", &VRManager::is_connected)
        .def("get_pose", &VRManager::get_pose)
        .def("get_button_state", &VRManager::get_button_state)
        .def("get_latest_frame", &VRManager::get_latest_frame)
        .def("get_current_state", &VRManager::get_current_state)
        .def("poll_teleop", &VRManager::poll_teleop);

    // -------------------------
    // Teleop
    // -------------------------
    py::class_<Teleop>(m, "Teleop")
        .def(py::init<>())
        // Buttons
        .def("set_button_A_handler", &Teleop::set_button_A_handler)
        .def("set_button_B_handler", &Teleop::set_button_B_handler)
        .def("set_button_X_handler", &Teleop::set_button_X_handler)
        .def("set_button_Y_handler", &Teleop::set_button_Y_handler)
        // Triggers / Grips
        .def("set_button_right_trigger_handler", &Teleop::set_button_right_trigger_handler)
        .def("set_button_right_grip_handler", &Teleop::set_button_right_grip_handler)
        .def("set_button_left_trigger_handler", &Teleop::set_button_left_trigger_handler)
        .def("set_button_left_grip_handler", &Teleop::set_button_left_grip_handler)
        // Pose handlers
        .def("set_left_pose_handler", &Teleop::set_left_pose_handler)
        .def("set_right_pose_handler", &Teleop::set_right_pose_handler)
        // Other
        .def("evaluate_button_states", &Teleop::evaluate_button_states)
        .def("set_exit_condition", &Teleop::set_exit_condition)
        .def("notify_start", &Teleop::notify_start)
        .def("notify_pause", &Teleop::notify_pause)
        .def("notify_resume", &Teleop::notify_resume)
        .def("get_pending_update", &Teleop::get_pending_update)
        .def("clear_pending_update", &Teleop::clear_pending_update);
}

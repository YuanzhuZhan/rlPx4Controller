#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "Px4AttitudeController.hpp"
#include "SimplePositionController.hpp"
#include "Px4Mixer.hpp"
#include "parallelRateController.hpp"
#include "parallelVelController.hpp"
#include "parallelAttiController.hpp"
#include "parallelPosController.hpp"
#include "parallelHexaPosController.hpp"
namespace py = pybind11;

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

PYBIND11_MODULE(pyParallelControl, m)
{
    m.doc() = R"pbdoc(
        Quadrotor Parallel Controller Utils
        -----------------------
    )pbdoc";

    py::class_<HexaMixer>(m, "HexaMixer")
        .def(py::init<>())
        .def("configure", &HexaMixer::configure,
             py::arg("positions_b"), py::arg("axes_b"), py::arg("reaction_signed_m"),
             py::arg("max_thrust_n"), py::arg("motor_tau") = 0.0)
        .def("update", &HexaMixer::update, py::arg("normalized_wrench"), py::arg("dt"))
        .def("reset", &HexaMixer::reset)
        .def_property_readonly("wrench_scale", &HexaMixer::wrenchScale)
        .def_property_readonly("allocation_matrix", &HexaMixer::allocationMatrix)
        .def_property_readonly("wrench_error", &HexaMixer::wrenchError);

    py::class_<pyParallelHexaPositionControl>(m, "ParallelHexaPosControl")
        .def(py::init<int>(), py::arg("envs_num"))
        .def("configure", &pyParallelHexaPositionControl::configure,
             py::arg("positions_b"), py::arg("axes_b"), py::arg("reaction_signed_m"),
             py::arg("max_thrust_n"), py::arg("masses"), py::arg("motor_tau") = 0.0)
        .def("set_status", &pyParallelHexaPositionControl::set_status,
             py::arg("pos"), py::arg("q_matrix"), py::arg("vel"), py::arg("ang_vel"), py::arg("dt"))
        .def("reset", &pyParallelHexaPositionControl::reset, py::arg("env_ids"))
        .def("update", &pyParallelHexaPositionControl::update, py::arg("actions"));

    py::class_<pyParallelRateControl>(m, "ParallelRateControl")
        .def(py::init<int>(), "batch calc")
        .def("set_q_world", &pyParallelRateControl::set_q_world, py::arg("q_world").none())
        .def("update", &pyParallelRateControl::update,
             py::arg("rate_sp").none(), py::arg("rate").none(), py::arg("dt").none());

    py::class_<pyParallelVelocityControl>(m, "ParallelVelControl")
        .def(py::init<int>(), "b")
        .def("set_status", &pyParallelVelocityControl::set_status, py::arg("pos").none(), py::arg("q_matrix").none(), py::arg("vel").none(), py::arg("ang_vel").none(), py::arg("dt").none())
        .def("update", &pyParallelVelocityControl::update, py::arg("actions").none());
    py::class_<pyParallelPositionControl>(m, "ParallelPosControl")
        .def(py::init<int>(), "b")
        .def("set_status", &pyParallelPositionControl::set_status, py::arg("pos").none(), py::arg("q_matrix").none(), py::arg("vel").none(), py::arg("ang_vel").none(), py::arg("dt").none())
        .def("update", &pyParallelPositionControl::update, py::arg("actions").none());
        
    py::class_<pyParallelAttiControl>(m, "ParallelAttiControl")
        .def(py::init<int>(), "b")
        .def("set_status", &pyParallelAttiControl::set_status, py::arg("pos").none(), py::arg("q_matrix").none(), py::arg("vel").none(), py::arg("ang_vel").none(), py::arg("dt").none())
        .def("update", &pyParallelAttiControl::update, py::arg("actions").none());

    ;
#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}

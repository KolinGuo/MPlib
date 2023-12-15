#pragma once

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "ompl_planner.h"

namespace py = pybind11;

#ifdef USE_SINGLE
using S = float;
#else
using S = double;
#endif

namespace mplib {

using OMPLPlanner = ompl::OMPLPlannerTpl<S>;

inline void build_pyompl(py::module &m_all) {
  auto m = m_all.def_submodule("ompl");

  auto PyOMPLPlanner =
      py::class_<OMPLPlanner, std::shared_ptr<OMPLPlanner>>(m, "OMPLPlanner");
  PyOMPLPlanner
      .def(py::init<PlanningWorldTplPtr<S> const &>(), py::arg("world"))
      .def("plan", &OMPLPlanner::plan, py::arg("start_state"),
           py::arg("goal_states"), py::arg("planner_name") = "RRTConnect",
           py::arg("time") = 1.0, py::arg("range") = 0.0,
           py::arg("verbose") = false);
}

}  // namespace mplib

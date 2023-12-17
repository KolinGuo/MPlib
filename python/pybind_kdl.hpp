#pragma once

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>
#include <vector>

#include "kdl_model.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib {

using KDLModel = KDLModelTpl<S>;

inline void build_pykdl(py::module &m_all) {
  auto m = m_all.def_submodule("kdl");

  auto PyKDLModel =
      py::class_<KDLModel, std::shared_ptr<KDLModel>>(m, "KDLModel");
  PyKDLModel
      .def(py::init<std::string const &, std::vector<std::string> const &,
                    std::vector<std::string> const &, bool>(),
           py::arg("urdf_filename"), py::arg("joint_names"),
           py::arg("link_names"), py::arg("verbose"))
      .def("get_tree_root_name", &KDLModel::getTreeRootName)
      .def("chain_IK_LMA", &KDLModel::chainIKLMA, py::arg("index"),
           py::arg("q_init"), py::arg("goal_pose"))
      .def("chain_IK_NR", &KDLModel::chainIKNR, py::arg("index"),
           py::arg("q_init"), py::arg("goal_pose"))
      .def("chain_IK_NR_JL", &KDLModel::chainIKNRJL, py::arg("index"),
           py::arg("q_init"), py::arg("goal_pose"), py::arg("q_min"),
           py::arg("q_max"))
      .def("tree_IK_NR_JL", &KDLModel::TreeIKNRJL, py::arg("endpoints"),
           py::arg("q_init"), py::arg("goal_poses"), py::arg("q_min"),
           py::arg("q_max"));
}

}  // namespace mplib

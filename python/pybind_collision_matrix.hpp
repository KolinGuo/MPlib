#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "collision_matrix.h"

namespace py = pybind11;

namespace mplib {

inline void build_collision_matrix(py::module &m_all) {
  auto m = m_all.def_submodule("collision_matrix");

  auto PyAllowedCollision = py::enum_<AllowedCollision>(m, "AllowedCollision");
  PyAllowedCollision.value("NEVER", AllowedCollision::NEVER)
      .value("ALWAYS", AllowedCollision::ALWAYS)
      .value("CONDITIONAL", AllowedCollision::CONDITIONAL);

  auto PyAllowedCollisionMatrix =
      py::class_<AllowedCollisionMatrix,
                 std::shared_ptr<AllowedCollisionMatrix>>(
          m, "AllowedCollisionMatrix");
  PyAllowedCollisionMatrix.def(py::init<>())
      .def("get_entry", &AllowedCollisionMatrix::getEntry, py::arg("name1"),
           py::arg("name2"))
      .def("has_entry",
           py::overload_cast<const std::string &>(
               &AllowedCollisionMatrix::hasEntry, py::const_),
           py::arg("name"))
      .def("has_entry",
           py::overload_cast<const std::string &, const std::string &>(
               &AllowedCollisionMatrix::hasEntry, py::const_),
           py::arg("name1"), py::arg("name2"))
      .def("set_entry",
           py::overload_cast<const std::string &, const std::string &, bool>(
               &AllowedCollisionMatrix::setEntry),
           py::arg("name1"), py::arg("name2"), py::arg("allowed"))
      .def("set_entry",
           py::overload_cast<const std::string &,
                             const std::vector<std::string> &, bool>(
               &AllowedCollisionMatrix::setEntry),
           py::arg("name"), py::arg("other_names"), py::arg("allowed"))
      .def("set_entry",
           py::overload_cast<const std::vector<std::string> &,
                             const std::vector<std::string> &, bool>(
               &AllowedCollisionMatrix::setEntry),
           py::arg("names1"), py::arg("names2"), py::arg("allowed"))
      .def("set_entry",
           py::overload_cast<const std::string &, bool>(
               &AllowedCollisionMatrix::setEntry),
           py::arg("name"), py::arg("allowed"))
      .def("set_entry",
           py::overload_cast<bool>(&AllowedCollisionMatrix::setEntry),
           py::arg("allowed"))
      .def("remove_entry",
           py::overload_cast<const std::string &, const std::string &>(
               &AllowedCollisionMatrix::removeEntry),
           py::arg("name1"), py::arg("name2"))
      .def("remove_entry",
           py::overload_cast<const std::string &>(
               &AllowedCollisionMatrix::removeEntry),
           py::arg("name"))
      .def("__len__",
           [](const AllowedCollisionMatrix &acm) { return acm.getSize(); })

      .def("get_default_entry",
           py::overload_cast<const std::string &>(
               &AllowedCollisionMatrix::getDefaultEntry, py::const_),
           py::arg("name"))
      .def("has_default_entry", &AllowedCollisionMatrix::hasDefaultEntry,
           py::arg("name"))
      .def("set_default_entry", &AllowedCollisionMatrix::setDefaultEntry,
           py::arg("name"), py::arg("allowed"))
      .def("remove_default_entry", &AllowedCollisionMatrix::removeDefaultEntry,
           py::arg("name"))

      .def("get_allowed_collision",
           &AllowedCollisionMatrix::getAllowedCollision, py::arg("name1"),
           py::arg("name2"))

      .def("clear", &AllowedCollisionMatrix::clear)

      .def("get_all_entry_names", &AllowedCollisionMatrix::getAllEntryNames)
      .def("__str__", [](const AllowedCollisionMatrix &acm) {
        std::stringstream ss;
        acm.print(ss);
        return ss.str();
      });
}

}  // namespace mplib

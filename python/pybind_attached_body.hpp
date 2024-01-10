#pragma once

#include <memory>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "attached_body.h"
#include "pybind_macros.hpp"
#include "types.h"

namespace py = pybind11;

namespace mplib {

using AttachedBody = AttachedBodyTpl<S>;
using CollisionObjectPtr = fcl::CollisionObjectPtr<S>;
using ArticulatedModelPtr = ArticulatedModelTplPtr<S>;

inline void build_attached_body(py::module &m_all) {
  auto m = m_all.def_submodule("attached_body");

  auto PyAttachedBody =
      py::class_<AttachedBody, std::shared_ptr<AttachedBody>>(m, "AttachedBody");
  PyAttachedBody
      .def(py::init([](const std::string &name, const CollisionObjectPtr &object,
                       const ArticulatedModelPtr &attached_articulation,
                       int attached_link_id, const Vector7<S> &posevec,
                       const std::vector<std::string> &touch_links) {
             Transform3<S> pose;
             pose.linear() =
                 Quaternion<S>(posevec[3], posevec[4], posevec[5], posevec[6]).matrix();
             pose.translation() = posevec.head(3);
             return AttachedBody(name, object, attached_articulation, attached_link_id,
                                 pose, touch_links);
           }),
           py::arg("name"), py::arg("object"), py::arg("attached_articulation"),
           py::arg("attached_link_id"), py::arg("pose"),
           py::arg("touch_links") = std::vector<std::string>())
      .def("get_name", &AttachedBody::getName)
      .def("get_object", &AttachedBody::getObject)
      .def("get_attached_articulation", &AttachedBody::getAttachedArticulation)
      .def("get_attached_link_id", &AttachedBody::getAttachedLinkId)
      .def("get_pose", &AttachedBody::getPose)
      .def(
          "set_pose",
          [](AttachedBody &self, const Vector7<S> &posevec) {
            Transform3<S> pose;
            pose.linear() =
                Quaternion<S>(posevec[3], posevec[4], posevec[5], posevec[6]).matrix();
            pose.translation() = posevec.head(3);
            self.setPose(pose);
          },
          py::arg("pose"))
      .def("get_global_pose", &AttachedBody::getGlobalPose)
      .def("update_pose", &AttachedBody::updatePose)
      .def("get_touch_links", &AttachedBody::getTouchLinks)
      .def("set_touch_links", &AttachedBody::setTouchLinks, py::arg("touch_links"));
}

}  // namespace mplib

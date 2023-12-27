#pragma once

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>
#include <vector>

#include "planning_world.h"
#include "pybind_macros.hpp"
#include "types.h"

namespace py = pybind11;

namespace mplib {

using PlanningWorld = PlanningWorldTpl<S>;
using WorldCollisionResult = WorldCollisionResultTpl<S>;

using ArticulatedModelPtr = ArticulatedModelTplPtr<S>;
using CollisionRequest = fcl::CollisionRequest<S>;
using CollisionGeometryPtr = fcl::CollisionGeometryPtr<S>;
using CollisionObjectPtr = fcl::CollisionObjectPtr<S>;

inline void build_planning_world(py::module &m_all) {
  auto m = m_all.def_submodule("planning_world");

  auto PyPlanningWorld =
      py::class_<PlanningWorld, std::shared_ptr<PlanningWorld>>(
          m, "PlanningWorld");
  PyPlanningWorld
      .def(py::init<std::vector<ArticulatedModelPtr> const &,
                    std::vector<std::string> const &,
                    std::vector<CollisionObjectPtr> const &,
                    std::vector<std::string> const &>(),
           py::arg("articulations"), py::arg("articulation_names"),
           py::arg("normal_objects") = std::vector<CollisionObjectPtr>(),
           py::arg("normal_object_names") = std::vector<std::string>())

      .def("get_articulation_names", &PlanningWorld::getArticulationNames)
      .def("get_planned_articulations", &PlanningWorld::getPlannedArticulations)
      .def("get_articulation", &PlanningWorld::getArticulation, py::arg("name"))
      .def("has_articulation", &PlanningWorld::hasArticulation, py::arg("name"))
      .def("add_articulation", &PlanningWorld::addArticulation, py::arg("name"),
           py::arg("model"), py::arg("planned") = false)
      .def("remove_articulation", &PlanningWorld::removeArticulation,
           py::arg("name"))
      .def("is_articulation_planned", &PlanningWorld::isArticulationPlanned,
           py::arg("name"))
      .def("set_articulation_planned", &PlanningWorld::setArticulationPlanned,
           py::arg("name"), py::arg("planned"))

      .def("get_normal_object_names", &PlanningWorld::getNormalObjectNames)
      .def("get_normal_object", &PlanningWorld::getNormalObject,
           py::arg("name"))
      .def("has_normal_object", &PlanningWorld::hasNormalObject,
           py::arg("name"))
      .def("add_normal_object", &PlanningWorld::addNormalObject,
           py::arg("name"), py::arg("collision_object"))
      .def("add_point_cloud", &PlanningWorld::addPointCloud, py::arg("name"),
           py::arg("vertices"), py::arg("resolution") = 0.01)
      .def("remove_normal_object", &PlanningWorld::removeNormalObject,
           py::arg("name"))

      .def("is_normal_object_attached", &PlanningWorld::isNormalObjectAttached,
           py::arg("name"))
      .def("attach_object",
           py::overload_cast<std::string const &, std::string const &, int,
                             Vector7<S> const &,
                             std::vector<std::string> const &>(
               &PlanningWorld::attachObject),
           py::arg("name"), py::arg("art_name"), py::arg("link_id"),
           py::arg("pose"), py::arg("touch_links"))
      .def("attach_object",
           py::overload_cast<std::string const &, std::string const &, int,
                             Vector7<S> const &>(&PlanningWorld::attachObject),
           py::arg("name"), py::arg("art_name"), py::arg("link_id"),
           py::arg("pose"))
      .def("attach_object",
           py::overload_cast<std::string const &, CollisionGeometryPtr const &,
                             std::string const &, int, Vector7<S> const &,
                             std::vector<std::string> const &>(
               &PlanningWorld::attachObject),
           py::arg("name"), py::arg("p_geom"), py::arg("art_name"),
           py::arg("link_id"), py::arg("pose"), py::arg("touch_links"))
      .def("attach_object",
           py::overload_cast<std::string const &, CollisionGeometryPtr const &,
                             std::string const &, int, Vector7<S> const &>(
               &PlanningWorld::attachObject),
           py::arg("name"), py::arg("p_geom"), py::arg("art_name"),
           py::arg("link_id"), py::arg("pose"))
      .def("attach_sphere", &PlanningWorld::attachSphere, py::arg("radius"),
           py::arg("art_name"), py::arg("link_id"), py::arg("pose"))
      .def("attach_box", &PlanningWorld::attachBox, py::arg("size"),
           py::arg("art_name"), py::arg("link_id"), py::arg("pose"))
      .def("attach_mesh", &PlanningWorld::attachMesh, py::arg("mesh_path"),
           py::arg("art_name"), py::arg("link_id"), py::arg("pose"))
      .def("detach_object", &PlanningWorld::detachObject, py::arg("name"),
           py::arg("also_remove") = false)
      .def("print_attached_body_pose", &PlanningWorld::printAttachedBodyPose)

      .def("set_qpos", &PlanningWorld::setQpos, py::arg("name"),
           py::arg("qpos"))
      .def("set_qpos_all", &PlanningWorld::setQposAll, py::arg("state"))

      .def("get_allowed_collision_matrix",
           &PlanningWorld::getAllowedCollisionMatrix)

      .def("collide", &PlanningWorld::collide,
           py::arg("request") = CollisionRequest())
      .def("self_collide", &PlanningWorld::selfCollide,
           py::arg("request") = CollisionRequest())
      .def("collide_with_others", &PlanningWorld::collideWithOthers,
           py::arg("request") = CollisionRequest())
      .def("collide_full", &PlanningWorld::collideFull,
           py::arg("request") = CollisionRequest());

  auto PyWorldCollisionResult =
      py::class_<WorldCollisionResult, std::shared_ptr<WorldCollisionResult>>(
          m, "WorldCollisionResult");
  PyWorldCollisionResult.def_readonly("res", &WorldCollisionResult::res)
      .def_readonly("object_name1", &WorldCollisionResult::object_name1)
      .def_readonly("object_name2", &WorldCollisionResult::object_name2)
      .def_readonly("collision_type", &WorldCollisionResult::collision_type)
      .def_readonly("link_name1", &WorldCollisionResult::link_name1)
      .def_readonly("link_name2", &WorldCollisionResult::link_name2);
}

}  // namespace mplib

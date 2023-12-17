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
using CollisionObjectPtr = fcl::CollisionObjectPtr<S>;
using CollisionRequest = fcl::CollisionRequest<S>;

inline void build_planning_world(py::module &m_all) {
  auto m = m_all.def_submodule("planning_world");

  auto PyPlanningWorld =
      py::class_<PlanningWorld, std::shared_ptr<PlanningWorld>>(
          m, "PlanningWorld");
  PyPlanningWorld
      .def(py::init<std::vector<ArticulatedModelPtr> const &,
                    std::vector<std::string> const &,
                    std::vector<CollisionObjectPtr> const &,
                    std::vector<std::string> const &, int>(),
           py::arg("articulations"), py::arg("articulation_names"),
           py::arg("normal_objects") = std::vector<CollisionObjectPtr>(),
           py::arg("normal_object_names") = std::vector<std::string>(),
           py::arg("move_articulation_id") = 0)

      .def("get_normal_object_names", &PlanningWorld::getNormalObjectNames)
      .def("get_normal_object", &PlanningWorld::getNormalObject,
           py::arg("name"))
      .def("has_normal_object", &PlanningWorld::hasNormalObject,
           py::arg("name"))
      .def("add_normal_object", &PlanningWorld::addNormalObject,
           py::arg("name"), py::arg("collision_object"))
      .def("remove_normal_object", &PlanningWorld::removeNormalObject,
           py::arg("name"))

      .def("get_articulation_names", &PlanningWorld::getArticulationNames)
      .def("get_articulation", &PlanningWorld::getArticulation, py::arg("name"))
      .def("has_articulation", &PlanningWorld::hasArticulation, py::arg("name"))
      .def("add_articulation", &PlanningWorld::addArticulation, py::arg("name"),
           py::arg("model"))
      .def("remove_articulation", &PlanningWorld::removeArticulation,
           py::arg("name"))

      .def("set_qpos", &PlanningWorld::setQpos, py::arg("index"),
           py::arg("qpos"))
      .def("set_qpos_all", &PlanningWorld::setQposAll, py::arg("qpos"))

      .def("collide", &PlanningWorld::collide)
      .def("self_collide", &PlanningWorld::selfCollide, py::arg("index") = 0,
           py::arg("request") = CollisionRequest())
      .def("collide_with_others", &PlanningWorld::collideWithOthers,
           py::arg("index") = 0, py::arg("request") = CollisionRequest())
      .def("collide_full", &PlanningWorld::collideFull, py::arg("index") = 0,
           py::arg("request") = CollisionRequest())

      .def("set_use_point_cloud", &PlanningWorld::setUsePointCloud,
           py::arg("use") = false)
      .def("update_point_cloud", &PlanningWorld::updatePointCloud,
           py::arg("vertices"), py::arg("resolution") = 0.01)
      .def("set_use_attach", &PlanningWorld::setUseAttach,
           py::arg("use") = false)
      .def("remove_attach", &PlanningWorld::removeAttach)

      .def("update_attached_tool", &PlanningWorld::updateAttachedTool,
           py::arg("p_geom"), py::arg("link_id"), py::arg("pose"))
      .def("update_attached_sphere", &PlanningWorld::updateAttachedSphere,
           py::arg("radius"), py::arg("link_id"), py::arg("pose"))
      .def("update_attached_box", &PlanningWorld::updateAttachedBox,
           py::arg("size"), py::arg("link_id"), py::arg("pose"))
      .def("update_attached_mesh", &PlanningWorld::updateAttachedMesh,
           py::arg("mesh_path"), py::arg("link_id"), py::arg("pose"))
      .def("print_attached_tool_pose", &PlanningWorld::printAttachedToolPose);

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

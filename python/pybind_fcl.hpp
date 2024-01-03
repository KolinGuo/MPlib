#pragma once

#include <memory>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "fcl_model.h"
#include "pybind_macros.hpp"
#include "types.h"
#include "urdf_utils.h"

namespace py = pybind11;

namespace mplib {

using FCLModel = fcl::FCLModelTpl<S>;

using Triangle = fcl::Triangle;
using CollisionGeometry = fcl::CollisionGeometry<S>;
using Box = fcl::Box<S>;
using Capsule = fcl::Capsule<S>;
using Cone = fcl::Cone<S>;
using Convex = fcl::Convex<S>;
using Cylinder = fcl::Cylinder<S>;
using Plane = fcl::Plane<S>;
using Sphere = fcl::Sphere<S>;
using BVHModel_OBBRSS = fcl::BVHModel<fcl::OBBRSS<S>>;
using OcTree = fcl::OcTree<S>;

using CollisionObject = fcl::CollisionObject<S>;
using GJKSolverType = fcl::GJKSolverType;
using CollisionRequest = fcl::CollisionRequest<S>;
using CollisionResult = fcl::CollisionResult<S>;
using DistanceRequest = fcl::DistanceRequest<S>;
using DistanceResult = fcl::DistanceResult<S>;
using Contact = fcl::Contact<S>;
using ContactPoint = fcl::ContactPoint<S>;
using CostSource = fcl::CostSource<S>;

inline void build_pyfcl(py::module &m_all) {
  auto m = m_all.def_submodule("fcl");

  // Data type
  auto PyTriangle =
      py::class_<Triangle, std::shared_ptr<Triangle>>(m, "Triangle");
  PyTriangle.def(py::init())
      .def(py::init<unsigned long, unsigned long, unsigned long>())
      .def("set", [](Triangle &a, const int &p1, const int &p2,
                     const int &p3) { a.set(p1, p2, p3); })
      .def("get", [](Triangle &a, int i) { return a[i]; })
      .def("__getitem__", [](Triangle &a, int i) { return a[i]; });

  // Collision Geometry type
  auto PyCollisionGeometry =
      py::class_<CollisionGeometry, std::shared_ptr<CollisionGeometry>>(
          m, "CollisionGeometry");
  PyCollisionGeometry
      .def("computeLocalAABB", &CollisionGeometry::computeLocalAABB)
      .def("isOccupied", &CollisionGeometry::isOccupied)
      .def("isFree", &CollisionGeometry::isFree)
      .def("isUncertain", &CollisionGeometry::isUncertain)
      .def("computeCOM", &CollisionGeometry::computeCOM)
      .def("computeMomentofInertia", &CollisionGeometry::computeMomentofInertia)
      .def("computeVolume", &CollisionGeometry::computeVolume)
      .def("computeMomentofInertiaRelatedToCOM",
           &CollisionGeometry::computeMomentofInertiaRelatedToCOM)
      .def_readwrite("aabb_center", &CollisionGeometry::aabb_center)
      .def_readwrite("aabb_radius", &CollisionGeometry::aabb_radius)
      .def_readwrite("cost_density", &CollisionGeometry::cost_density);

  auto PyBox =
      py::class_<Box, std::shared_ptr<Box>>(m, "Box", PyCollisionGeometry);
  PyBox.def(py::init<const Vector3<S> &>(), py::arg("side"))
      .def(py::init<S, S, S>(), py::arg("x"), py::arg("y"), py::arg("z"))
      .def_readwrite("side", &Box::side);

  auto PyCapsule = py::class_<Capsule, std::shared_ptr<Capsule>>(
      m, "Capsule", PyCollisionGeometry);
  PyCapsule.def(py::init<S, S>(), py::arg("radius"), py::arg("lz"))
      .def_readwrite("radius", &Capsule::radius)
      .def_readwrite("lz", &Capsule::lz);

  auto PyCone =
      py::class_<Cone, std::shared_ptr<Cone>>(m, "Cone", PyCollisionGeometry);
  PyCone.def(py::init<S, S>(), py::arg("radius"), py::arg("lz"))
      .def_readwrite("radius", &Cone::radius)
      .def_readwrite("lz", &Cone::lz);

  auto PyConvex = py::class_<Convex, std::shared_ptr<Convex>>(
      m, "Convex", PyCollisionGeometry);
  PyConvex
      .def(py::init<const std::shared_ptr<const std::vector<Vector3<S>>> &, int,
                    const std::shared_ptr<const std::vector<int>> &, bool>(),
           py::arg("vertices"), py::arg("num_faces"), py::arg("faces"),
           py::arg("throw_if_invalid") = true)
      .def(py::init([](MatrixX3<S> const &vertices, MatrixX3i const &faces,
                       bool const &throw_if_invalid) {
             auto vertices_new = std::make_shared<std::vector<Vector3<S>>>();
             auto faces_new = std::make_shared<std::vector<int>>();
             for (size_t i = 0; i < static_cast<size_t>(vertices.rows()); i++) {
               Vector3<S> tmp_i;
               tmp_i(0) = vertices(i, 0);
               tmp_i(1) = vertices(i, 1);
               tmp_i(2) = vertices(i, 2);
               vertices_new->push_back(tmp_i);
             }
             for (size_t i = 0; i < static_cast<size_t>(faces.rows()); i++) {
               faces_new->push_back(3);
               for (size_t j = 0; j < 3; j++) faces_new->push_back(faces(i, j));
             }
             return Convex(vertices_new, faces.rows(), faces_new,
                           throw_if_invalid);
           }),
           py::arg("vertices"), py::arg("faces"),
           py::arg("throw_if_invalid") = true)
      .def("get_face_count", &Convex::getFaceCount)
      .def("get_faces", &Convex::getFaces)
      .def("get_vertices", &Convex::getVertices)
      .def("compute_volume", &Convex::computeVolume)
      .def("get_interior_point", &Convex::getInteriorPoint);

  auto PyCylinder = py::class_<Cylinder, std::shared_ptr<Cylinder>>(
      m, "Cylinder", PyCollisionGeometry);
  PyCylinder.def(py::init<S, S>(), py::arg("radius"), py::arg("lz"))
      .def_readwrite("radius", &Cylinder::radius)
      .def_readwrite("lz", &Cylinder::lz);

  auto PyPlane = py::class_<Plane, std::shared_ptr<Plane>>(m, "Plane",
                                                           PyCollisionGeometry);
  PyPlane.def(py::init<const Vector3<S> &, S>(), py::arg("n"), py::arg("d"))
      .def(py::init<S, S, S, S>(), py::arg("a"), py::arg("b"), py::arg("c"),
           py::arg("d"))
      .def_readwrite("n", &Plane::n)
      .def_readwrite("d", &Plane::d);

  auto PySphere = py::class_<Sphere, std::shared_ptr<Sphere>>(
      m, "Sphere", PyCollisionGeometry);
  PySphere.def(py::init<S>(), py::arg("radius"))
      .def_readwrite("radius", &Sphere::radius);

  auto PyBVHModel_OBBRSS =
      py::class_<BVHModel_OBBRSS, std::shared_ptr<BVHModel_OBBRSS>>(
          m, "BVHModel", PyCollisionGeometry);

  PyBVHModel_OBBRSS.def(py::init<>())
      .def("beginModel", &BVHModel_OBBRSS::beginModel, py::arg("num_faces") = 0,
           py::arg("num_vertices") = 0)
      .def("endModel", &BVHModel_OBBRSS::endModel)
      .def("addSubModel",
           py::overload_cast<const std::vector<Vector3<S>> &>(
               &BVHModel_OBBRSS::addSubModel),
           py::arg("vertices"))
      .def("addSubModel",
           py::overload_cast<const std::vector<Vector3<S>> &,
                             const std::vector<Triangle> &>(
               &BVHModel_OBBRSS::addSubModel),
           py::arg("vertices"), py::arg("faces"))
      .def(
          "addSubModel",
          [](BVHModel_OBBRSS &a, const std::vector<Vector3<S>> &vertices,
             const std::vector<Vector3i> &faces) {
            std::vector<Triangle> face_list;
            for (size_t i = 0; i < faces.size(); i++)
              face_list.push_back(
                  Triangle(faces[i][0], faces[i][1], faces[i][2]));
            a.addSubModel(vertices, face_list);
          },
          py::arg("vertices"), py::arg("faces"))
      .def("get_vertices",
           [](BVHModel_OBBRSS &a) {
             std::vector<Vector3<S>> ret;
             for (size_t i = 0; i < static_cast<size_t>(a.num_vertices); i++)
               ret.push_back(*(a.vertices + i));
             return ret;
           })
      .def("get_faces",
           [](BVHModel_OBBRSS &a) {
             std::vector<Triangle> ret;
             for (size_t i = 0; i < static_cast<size_t>(a.num_tris); i++)
               ret.push_back(*(a.tri_indices + i));
             return ret;
           })
      .def_readonly("num_faces", &BVHModel_OBBRSS::num_tris)
      .def_readonly("num_vertices", &BVHModel_OBBRSS::num_vertices);

  auto PyOcTree = py::class_<OcTree, std::shared_ptr<OcTree>>(
      m, "OcTree", PyCollisionGeometry);
  PyOcTree.def(py::init<S>(), py::arg("resolution"))
      .def(py::init([](MatrixX3<S> const &vertices, double const &resolution) {
             octomap::OcTree *tree = new octomap::OcTree(resolution);

             // insert some measurements of occupied cells
             for (size_t i = 0; i < static_cast<size_t>(vertices.rows()); i++)
               tree->updateNode(octomap::point3d(vertices(i, 0), vertices(i, 1),
                                                 vertices(i, 2)),
                                true);

             auto tree_ptr = std::shared_ptr<const octomap::OcTree>(tree);
             OcTree *octree = new OcTree(tree_ptr);
             return octree;
           }),
           py::arg("vertices"), py::arg("resolution"));

  // Collision Object = Geometry + Transformation
  auto PyCollisionObject =
      py::class_<CollisionObject, std::shared_ptr<CollisionObject>>(
          m, "CollisionObject");
  PyCollisionObject
      .def(py::init([](const std::shared_ptr<CollisionGeometry> &a,
                       const Vector3<S> &p, const Vector4<S> &q) {
        auto q_mat = Quaternion<S>(q(0), q(1), q(2), q(3)).matrix();
        return CollisionObject(a, q_mat, p);
      }))
      .def("get_collision_geometry", &CollisionObject::collisionGeometry)
      .def("get_translation", &CollisionObject::getTranslation)
      .def("get_rotation", &CollisionObject::getRotation)
      .def("set_transformation",
           [](CollisionObject &a, const Vector7<S> &pose) {
             Transform3<S> trans;
             trans.linear() =
                 Quaternion<S>(pose[3], pose[4], pose[5], pose[6]).matrix();
             trans.translation() = pose.head(3);
             a.setTransform(trans);
           });

  /**********    narrowphase    *******/
  auto PyGJKSolverType = py::enum_<GJKSolverType>(m, "GJKSolverType");
  PyGJKSolverType.value("GST_LIBCCD", GJKSolverType::GST_LIBCCD)
      .value("GST_INDEP", GJKSolverType::GST_INDEP)
      .export_values();

  // CollisionRequest
  auto PyCollisionRequest =
      py::class_<CollisionRequest, std::shared_ptr<CollisionRequest>>(
          m, "CollisionRequest");
  PyCollisionRequest
      .def(py::init<size_t, bool, size_t, bool, bool, GJKSolverType, S>(),
           py::arg("num_max_contacts") = 1, py::arg("enable_contact") = false,
           py::arg("num_max_cost_sources") = 1, py::arg("enable_cost") = false,
           py::arg("use_approximate_cost") = true,
           py::arg("gjk_solver_type") = GJKSolverType::GST_LIBCCD,
           py::arg("gjk_tolerance") = 1e-6)
      .def("isSatisfied", &CollisionRequest::isSatisfied, py::arg("result"));

  // CollisionResult
  auto PyCollisionResult =
      py::class_<CollisionResult, std::shared_ptr<CollisionResult>>(
          m, "CollisionResult");
  PyCollisionResult.def(py::init<>())
      .def("add_contact", &CollisionResult::addContact, py::arg("c"))
      .def("add_cost_source", &CollisionResult::addCostSource, py::arg("c"),
           py::arg("num_max_cost_sources"))
      .def("is_collision", &CollisionResult::isCollision)
      .def("num_contacts", &CollisionResult::numContacts)
      .def("num_cost_sources", &CollisionResult::numCostSources)
      .def("get_contact", &CollisionResult::getContact, py::arg("i"))
      .def("get_contacts",
           [](CollisionResult &a) {
             std::vector<Contact> contacts;
             a.getContacts(contacts);
             return contacts;
           })
      .def("get_cost_sources",
           [](CollisionResult &a) {
             std::vector<CostSource> cost_sources;
             a.getCostSources(cost_sources);
             return cost_sources;
           })
      .def("clear", &CollisionResult::clear);

  // DistanceRequest
  auto PyDistanceRequest =
      py::class_<DistanceRequest, std::shared_ptr<DistanceRequest>>(
          m, "DistanceRequest");
  PyDistanceRequest
      .def(py::init<bool, bool, S, S, S, GJKSolverType>(),
           py::arg("enable_nearest_points") = false,
           py::arg("enable_signed_distance") = false, py::arg("rel_err") = 0.0,
           py::arg("abs_err") = 0.0, py::arg("distance_tolerance") = 1e-6,
           py::arg("gjk_solver_type") = GJKSolverType::GST_LIBCCD)
      .def("isSatisfied", &DistanceRequest::isSatisfied, py::arg("result"));

  // DistanceResult Not full suuport
  auto PyDistanceResult =
      py::class_<DistanceResult, std::shared_ptr<DistanceResult>>(
          m, "DistanceResult");
  PyDistanceResult
      .def(py::init<S>(),
           py::arg("min_distance") = std::numeric_limits<S>::max())
      .def_readonly("nearest_points", &DistanceResult::nearest_points)
      .def_readonly("min_distance", &DistanceResult::min_distance)
      .def("clear", &DistanceResult::clear);

  // Contact Not full support
  auto PyContact = py::class_<Contact, std::shared_ptr<Contact>>(m, "Contact");
  PyContact.def(py::init<>())
      .def(py::init<const CollisionGeometry *, const CollisionGeometry *, int,
                    int>(),
           py::arg("o1"), py::arg("o2"), py::arg("b1"), py::arg("b2"))
      .def(py::init<const CollisionGeometry *, const CollisionGeometry *, int,
                    int, const Vector3<S> &, const Vector3<S> &, S>(),
           py::arg("o1"), py::arg("o2"), py::arg("b1"), py::arg("b2"),
           py::arg("pos"), py::arg("normal"), py::arg("depth"))
      .def_readonly("normal", &Contact::normal)
      .def_readonly("pos", &Contact::pos)
      .def_readonly("penetration_depth", &Contact::penetration_depth);

  // ContactPoint Not full support
  auto PyContactPoint = py::class_<ContactPoint, std::shared_ptr<ContactPoint>>(
      m, "ContactPoint");
  PyContactPoint.def(py::init<>())
      .def(py::init<const Vector3<S> &, const Vector3<S> &, S>(),
           py::arg("normal"), py::arg("pos"), py::arg("penetration_depth"))
      .def_readonly("normal", &ContactPoint::normal)
      .def_readonly("pos", &ContactPoint::pos)
      .def_readonly("penetration_depth", &ContactPoint::penetration_depth);

  // CostSource Not full support
  auto PyCostSource =
      py::class_<CostSource, std::shared_ptr<CostSource>>(m, "CostSource");
  PyCostSource.def(py::init<>())
      .def(py::init<const Vector3<S> &, const Vector3<S> &, S>(),
           py::arg("aabb_min"), py::arg("aabb_max"), py::arg("cost_density"))
      .def_readonly("aabb_min", &CostSource::aabb_min)
      .def_readonly("aabb_max", &CostSource::aabb_max)
      .def_readonly("cost_density", &CostSource::cost_density)
      .def_readonly("total_cost", &CostSource::total_cost);

  // Collision function Not full suuport
  m.def("collide",
        [](const CollisionObject *o1, const CollisionObject *o2,
           const CollisionRequest &request) {
          CollisionResult result;
          ::fcl::collide(o1, o2, request, result);
          return result;
        })
      .def("distance", [](const CollisionObject *o1, const CollisionObject *o2,
                          const DistanceRequest &request) {
        DistanceResult result;
        ::fcl::distance(o1, o2, request, result);
        return result;
      });

  // FCL model
  auto PyFCLModel =
      py::class_<FCLModel, std::shared_ptr<FCLModel>>(m, "FCLModel");
  PyFCLModel
      .def(py::init<std::string const &, bool, bool>(),
           py::arg("urdf_filename"), py::arg("verbose") = true,
           py::arg("convex") = false)
      .def("get_collision_pairs", &FCLModel::getCollisionPairs)
      .def("get_collision_objects", &FCLModel::getCollisionObjects)
      .def("get_collision_link_names", &FCLModel::getCollisionLinkNames)
      .def("set_link_order", &FCLModel::setLinkOrder, py::arg("names"))
      .def("remove_collision_pairs_from_srdf",
           &FCLModel::removeCollisionPairsFromSrdf, py::arg("srdf_filename"))
      .def("update_collision_objects",
           py::overload_cast<std::vector<Vector7<S>> const &>(
               &FCLModel::updateCollisionObjects, py::const_),
           py::arg("link_poses"))
      .def("collide", &FCLModel::collide,
           py::arg("request") = CollisionRequest())
      .def("collide_full", &FCLModel::collideFull,
           py::arg("request") = CollisionRequest());

  // Extra function
  m.def("load_mesh_as_BVH", load_mesh_as_BVH<S>, py::arg("mesh_path"),
        py::arg("scale"));
  m.def("load_mesh_as_Convex", load_mesh_as_Convex<S>, py::arg("mesh_path"),
        py::arg("scale"));
}

}  // namespace mplib

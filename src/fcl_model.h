#pragma once
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/common/types.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/math/constants.h"
#include "fcl/math/triangle.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/collision_request.h"
#include "fcl/narrowphase/collision_result.h"
#include "fcl/narrowphase/gjk_solver_type.h"
#include "macros_utils.hpp"

template <typename DATATYPE>
class FCLModelTpl {
 private:
  DEFINE_TEMPLATE_FCL(DATATYPE)
  DEFINE_TEMPLATE_EIGEN(DATATYPE);

  urdf::ModelInterfaceSharedPtr urdf_model;

  std::vector<CollisionObject_ptr> collision_objects;
  std::vector<Transform3> collision_origin2link_poses;
  std::vector<std::string> collision_link_names;
  std::vector<std::string> parent_link_names;
  std::vector<std::pair<size_t, size_t>> collision_pairs;

  std::vector<std::string> user_link_names;
  std::vector<size_t> collision_link_user_indices;
  std::string package_dir;
  bool have_link_order, use_convex, verbose;

  void init(urdf::ModelInterfaceSharedPtr const &urdfTree,
            std::string const &package_dir_);
  void dfs_parse_tree(urdf::LinkConstSharedPtr const &link, std::string);

 public:
  FCLModelTpl(urdf::ModelInterfaceSharedPtr const &urdfTree,
              std::string const &package_dir, bool const &verbose = true,
              bool const &convex = false);

  FCLModelTpl(std::string const &urdf_filename, bool const &verbose = true,
              bool const &convex = false);

  inline std::vector<std::pair<size_t, size_t>> &getCollisionPairs() {
    return collision_pairs;
  }

  inline std::vector<CollisionObject_ptr> &getCollisionObjects() {
    return collision_objects;
  }

  inline std::vector<std::string> getCollisionLinkNames() {
    return collision_link_names;
  }

  inline std::vector<std::string> getUserLinkNames() { return user_link_names; }

  inline std::vector<size_t> getCollisionLinkUserIndices() {
    return collision_link_user_indices;
  }

  void setLinkOrder(const std::vector<std::string> &names);

  void printCollisionPairs(void);

  void removeCollisionPairsFromSrdf(std::string const &srdf_filename);

  void updateCollisionObjects(std::vector<Transform3> const &link_pose);

  void updateCollisionObjects(std::vector<Vector7> const &link_pose);

  bool collide(CollisionRequest const &request = CollisionRequest());

  std::vector<fcl::CollisionResult<DATATYPE>> collideFull(
      CollisionRequest const &request = CollisionRequest(
          1, false, 1, false, true, fcl::GJKSolverType::GST_INDEP, 1e-6));
};

template <typename DATATYPE>
using FCLModelTpl_ptr = std::shared_ptr<FCLModelTpl<DATATYPE>>;

using FCLModeld = FCLModelTpl<double>;
using FCLModelf = FCLModelTpl<float>;
using FCLModeld_ptr = FCLModelTpl_ptr<double>;
using FCLModelf_ptr = FCLModelTpl_ptr<float>;

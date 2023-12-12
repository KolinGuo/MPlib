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

  urdf::ModelInterfaceSharedPtr urdf_model_;

  std::vector<CollisionObjectPtr> collision_objects_;
  std::vector<Transform3> collision_origin2link_poses_;
  std::vector<std::string> collision_link_names_;
  std::vector<std::string> parent_link_names_;
  std::vector<std::pair<size_t, size_t>> collision_pairs_;

  std::vector<std::string> user_link_names_;
  std::vector<size_t> collision_link_user_indices_;
  std::string package_dir_;
  bool have_link_order_, use_convex_, verbose_;

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
    return collision_pairs_;
  }

  inline std::vector<CollisionObjectPtr> &getCollisionObjects() {
    return collision_objects_;
  }

  inline std::vector<std::string> getCollisionLinkNames() {
    return collision_link_names_;
  }

  inline std::vector<std::string> getUserLinkNames() { return user_link_names_; }

  inline std::vector<size_t> getCollisionLinkUserIndices() {
    return collision_link_user_indices_;
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
using FCLModelTplPtr = std::shared_ptr<FCLModelTpl<DATATYPE>>;

using FCLModeld = FCLModelTpl<double>;
using FCLModelf = FCLModelTpl<float>;
using FCLModeldPtr = FCLModelTplPtr<double>;
using FCLModelfPtr = FCLModelTplPtr<float>;

#pragma once

#include <urdf_model/types.h>
#include <urdf_world/types.h>

#include "macros_utils.h"
#include "types.h"

namespace mplib::fcl {

// FCLModelTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(FCLModelTpl);

template <typename S>
class FCLModelTpl {
 public:
  FCLModelTpl(urdf::ModelInterfaceSharedPtr const &urdfTree,
              std::string const &package_dir, bool verbose = true,
              bool convex = false);

  FCLModelTpl(std::string const &urdf_filename, bool verbose = true,
              bool convex = false);

  const std::vector<std::pair<size_t, size_t>> &getCollisionPairs() const {
    return collision_pairs_;
  }

  const std::vector<CollisionObjectPtr<S>> &getCollisionObjects() const {
    return collision_objects_;
  }

  const std::vector<std::string> &getCollisionLinkNames() const {
    return collision_link_names_;
  }

  const std::vector<std::string> &getUserLinkNames() const {
    return user_link_names_;
  }

  const std::vector<size_t> &getCollisionLinkUserIndices() const {
    return collision_link_user_indices_;
  }

  void setLinkOrder(const std::vector<std::string> &names);

  void printCollisionPairs() const;

  void removeCollisionPairsFromSrdf(std::string const &srdf_filename);

  void updateCollisionObjects(
      std::vector<Transform3<S>> const &link_pose) const;

  void updateCollisionObjects(std::vector<Vector7<S>> const &link_pose) const;

  bool collide(
      CollisionRequest<S> const &request = CollisionRequest<S>()) const;

  std::vector<CollisionResult<S>> collideFull(
      CollisionRequest<S> const &request = CollisionRequest<S>(
          1, false, 1, false, true, GJKSolverType::GST_INDEP, 1e-6)) const;

 private:
  urdf::ModelInterfaceSharedPtr urdf_model_;

  std::vector<CollisionObjectPtr<S>> collision_objects_;
  std::vector<Transform3<S>> collision_origin2link_poses_;
  std::vector<std::string> collision_link_names_;
  std::vector<std::string> parent_link_names_;
  std::vector<std::pair<size_t, size_t>> collision_pairs_;

  std::vector<std::string> user_link_names_;
  std::vector<size_t> collision_link_user_indices_;
  std::string package_dir_;
  bool have_link_order_, use_convex_, verbose_;

  void dfs_parse_tree(urdf::LinkConstSharedPtr const &link,
                      std::string const &parent_link_name);

  void init(urdf::ModelInterfaceSharedPtr const &urdfTree,
            std::string const &package_dir);
};

// Common Type Alias ==========================================================
using FCLModelf = FCLModelTpl<float>;
using FCLModeld = FCLModelTpl<double>;
using FCLModelfPtr = FCLModelTplPtr<float>;
using FCLModeldPtr = FCLModelTplPtr<double>;

// Explicit Template Instantiation Declaration ================================
#define DECLARE_TEMPLATE_FCL_MODEL(S) extern template class FCLModelTpl<S>

DECLARE_TEMPLATE_FCL_MODEL(float);
DECLARE_TEMPLATE_FCL_MODEL(double);

}  // namespace mplib::fcl

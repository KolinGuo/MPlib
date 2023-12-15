#pragma once
#include <urdf_model/types.h>
#include <urdf_world/types.h>

#include "types.h"

namespace mplib::fcl {

template <typename S>
class FCLModelTpl {
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

  inline std::vector<CollisionObjectPtr<S>> &getCollisionObjects() {
    return collision_objects_;
  }

  inline std::vector<std::string> getCollisionLinkNames() {
    return collision_link_names_;
  }

  inline std::vector<std::string> getUserLinkNames() {
    return user_link_names_;
  }

  inline std::vector<size_t> getCollisionLinkUserIndices() {
    return collision_link_user_indices_;
  }

  void setLinkOrder(const std::vector<std::string> &names);

  void printCollisionPairs(void);

  void removeCollisionPairsFromSrdf(std::string const &srdf_filename);

  void updateCollisionObjects(std::vector<Transform3<S>> const &link_pose);

  void updateCollisionObjects(std::vector<Vector7<S>> const &link_pose);

  bool collide(CollisionRequest<S> const &request = CollisionRequest<S>());

  std::vector<CollisionResult<S>> collideFull(
      CollisionRequest<S> const &request = CollisionRequest<S>(
          1, false, 1, false, true, GJKSolverType::GST_INDEP, 1e-6));
};

template <typename S>
using FCLModelTplPtr = std::shared_ptr<FCLModelTpl<S>>;

using FCLModeld = FCLModelTpl<double>;
using FCLModelf = FCLModelTpl<float>;
using FCLModeldPtr = FCLModelTplPtr<double>;
using FCLModelfPtr = FCLModelTplPtr<float>;

}  // namespace mplib::fcl

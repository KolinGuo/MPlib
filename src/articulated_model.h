#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "fcl_model.h"
#include "macros_utils.h"
#include "pinocchio_model.h"
#include "types.h"

namespace mplib {

// ArticulatedModelTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(ArticulatedModelTpl);

template <typename S>
class ArticulatedModelTpl {
 private:
  // Used for protecting public default constructor (passkey idiom)
  struct Secret;

 public:
  /// @brief Constructs an ArticulatedModel from URDF/SRDF files
  ArticulatedModelTpl(const std::string &urdf_filename,
                      const std::string &srdf_filename, const Vector3<S> &gravity,
                      const std::vector<std::string> &joint_names = {},
                      const std::vector<std::string> &link_names = {},
                      bool verbose = true, bool convex = false);

  /**
   * @brief Dummy default constructor that is protected by Secret.
   *  Used by createFromURDFString only
   */
  ArticulatedModelTpl(Secret secret) {}

  /**
   * @brief Constructs a ArticulatedModel from URDF/SRDF strings and collision links
   * @param urdf_string: URDF string (without visual/collision elements for links)
   * @param srdf_string: SRDF string (only disable_collisions element)
   * @param collision_links: Collision link names and the vector of CollisionObjectPtr
   *                         [(link_name, [CollisionObjectPtr, ...]), ...]
   *                         The collision objects are at the shape's local_pose
   * @returns a unique_ptr to ArticulatedModel
   */
  static std::unique_ptr<ArticulatedModelTpl<S>> createFromURDFString(
      const std::string &urdf_string, const std::string &srdf_string,
      const std::vector<std::pair<std::string, std::vector<fcl::CollisionObjectPtr<S>>>>
          &collision_links,
      const Vector3<S> &gravity, const std::vector<std::string> &joint_names = {},
      const std::vector<std::string> &link_names = {}, bool verbose = true);

  const std::string &getName() const { return name_; }

  void setName(const std::string &name) { name_ = name; }

  pinocchio::PinocchioModelTplPtr<S> getPinocchioModel() const {
    return pinocchio_model_;
  }

  fcl::FCLModelTplPtr<S> getFCLModel() const { return fcl_model_; }

  const std::vector<std::string> &getUserLinkNames() const { return user_link_names_; }

  const std::vector<std::string> &getUserJointNames() const {
    return user_joint_names_;
  }

  const std::vector<size_t> &getMoveGroupJointIndices() const {
    return move_group_user_joints_;
  }

  const std::vector<std::string> &getMoveGroupEndEffectors() const {
    return move_group_end_effectors_;
  }

  std::vector<std::string> getMoveGroupJointNames() const;

  void setMoveGroup(const std::string &end_effector);

  void setMoveGroup(const std::vector<std::string> &end_effectors);

  const VectorX<S> &getQpos() const { return current_qpos_; }

  void setQpos(const VectorX<S> &qpos, bool full = false);

  size_t getQposDim() const { return qpos_dim_; }

  void updateSRDF(const std::string &srdf_filename) {
    fcl_model_->removeCollisionPairsFromSRDF(srdf_filename);
  }

 private:
  // Used for protecting public default constructor (passkey idiom)
  struct Secret {
    explicit Secret() = default;
  };

  std::string name_;
  pinocchio::PinocchioModelTplPtr<S> pinocchio_model_;
  fcl::FCLModelTplPtr<S> fcl_model_;

  // all links and joints you want to control. order matters
  std::vector<std::string> user_link_names_;
  std::vector<std::string> user_joint_names_;

  // The planning world only update the state in planning group.
  std::vector<size_t> move_group_user_joints_;
  std::vector<std::string> move_group_end_effectors_;
  VectorX<S> current_qpos_;

  size_t qpos_dim_ {};
  bool verbose_ {};
};

// Common Type Alias ==========================================================
using ArticulatedModelf = ArticulatedModelTpl<float>;
using ArticulatedModeld = ArticulatedModelTpl<double>;
using ArticulatedModelfPtr = ArticulatedModelTplPtr<float>;
using ArticulatedModeldPtr = ArticulatedModelTplPtr<double>;

// Explicit Template Instantiation Declaration ================================
#define DECLARE_TEMPLATE_ARTICULATED_MODEL(S) \
  extern template class ArticulatedModelTpl<S>

DECLARE_TEMPLATE_ARTICULATED_MODEL(float);
DECLARE_TEMPLATE_ARTICULATED_MODEL(double);

}  // namespace mplib

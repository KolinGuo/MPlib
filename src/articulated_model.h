#pragma once

#include "fcl_model.h"
#include "macros_utils.h"
#include "pinocchio_model.h"
#include "types.h"

namespace mplib {

// ArticulatedModelTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(ArticulatedModelTpl);

template <typename S>
class ArticulatedModelTpl {
 public:
  ArticulatedModelTpl(std::string const &urdf_filename,
                      std::string const &srdf_filename,
                      Vector3<S> const &gravity,
                      std::vector<std::string> const &joint_names = {},
                      std::vector<std::string> const &link_names = {},
                      bool verbose = true, bool convex = false);

  pinocchio::PinocchioModelTplPtr<S> getPinocchioModel() const {
    return pinocchio_model_;
  }

  fcl::FCLModelTplPtr<S> getFCLModel() const { return fcl_model_; }

  const std::vector<std::string> &getUserLinkNames() const {
    return user_link_names_;
  }

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

  void setMoveGroup(std::string const &end_effector);

  void setMoveGroup(std::vector<std::string> const &end_effectors);

  const VectorX<S> &getQpos() const { return current_qpos_; }

  void setQpos(VectorX<S> const &qpos, bool full = false);

  size_t getQposDim() const { return qpos_dim_; }

  void updateSRDF(std::string const &srdf) {
    fcl_model_->removeCollisionPairsFromSrdf(srdf);
  }

 private:
  pinocchio::PinocchioModelTplPtr<S> pinocchio_model_;
  fcl::FCLModelTplPtr<S> fcl_model_;

  // all links and joints you want to control. order matters
  std::vector<std::string> user_link_names_;
  std::vector<std::string> user_joint_names_;

  // The planning world only update the state in planning group.
  std::vector<size_t> move_group_user_joints_;
  std::vector<std::string> move_group_end_effectors_;
  VectorX<S> current_qpos_;

  size_t qpos_dim_;
  bool verbose_;
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

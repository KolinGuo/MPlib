#pragma once

#include "fcl_model.h"
#include "macros_utils.h"
#include "pinocchio_model.h"

template <typename S>
class ArticulatedModelTpl {
 private:
  DEFINE_TEMPLATE_EIGEN(S);
  using PinocchioModel = PinocchioModelTpl<S>;
  using FCLModel = FCLModelTpl<S>;

  PinocchioModel pinocchio_model_;
  FCLModel fcl_model_;

  std::vector<std::string> user_link_names_;
  std::vector<std::string> user_joint_names_;  // all links and joints you want
                                               // to control. order matters

  std::vector<size_t> move_group_user_joints_;
  std::vector<std::string> move_group_end_effectors_;
  VectorX current_qpos_;  // The planning world only update the state in
                          // planning group.

  int qpos_dim_;
  bool verbose_;

 public:
  ArticulatedModelTpl(std::string const &urdf_filename,
                      std::string const &srdf_filename, Vector3 const &gravity,
                      std::vector<std::string> const &joint_names = {},
                      std::vector<std::string> const &link_names = {},
                      bool const &verbose = true, bool const &convex = false);

  PinocchioModelTpl<S> &getPinocchioModel() { return pinocchio_model_; }

  FCLModelTpl<S> &getFCLModel() { return fcl_model_; }

  void setMoveGroup(std::string const &end_effector);

  void setMoveGroup(std::vector<std::string> const &end_effectors);

  std::vector<size_t> getMoveGroupJointIndices(void) {
    return move_group_user_joints_;
  }

  std::vector<std::string> getMoveGroupJointName(void);

  std::vector<std::string> getUserJointNames(void) { return user_joint_names_; }

  std::vector<std::string> getUserLinkNames(void) { return user_link_names_; }

  std::vector<std::string> getMoveGroupEndEffectors(void) {
    return move_group_end_effectors_;
  }

  size_t getQposDim(void) { return qpos_dim_; }

  VectorX getQpos(void) { return current_qpos_; }

  void setQpos(VectorX const &qpos, bool const &full = false);

  void updateSRDF(std::string const &srdf) {
    fcl_model_.removeCollisionPairsFromSrdf(srdf);
  }
};

template <typename T>
using ArticulatedModelTplPtr = std::shared_ptr<ArticulatedModelTpl<T>>;

using ArticulatedModeld = ArticulatedModelTpl<double>;
using ArticulatedModelf = ArticulatedModelTpl<float>;
using ArticulatedModeldPtr = ArticulatedModelTplPtr<double>;
using ArticulatedModelfPtr = ArticulatedModelTplPtr<float>;

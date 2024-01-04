#pragma once

#include <vector>

#include <urdf_model/types.h>
#include <urdf_world/types.h>

#include "macros_utils.h"
#include "types.h"

namespace mplib::pinocchio {

// PinocchioModelTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(PinocchioModelTpl);

template <typename S>
class PinocchioModelTpl {
 public:
  PinocchioModelTpl(const urdf::ModelInterfaceSharedPtr &urdfTree,
                    const Vector3<S> &gravity, bool verbose = true);

  PinocchioModelTpl(const std::string &urdf_filename, const Vector3<S> &gravity,
                    bool verbose = true);

  const Model<S> &getModel(void) const { return model_; }

  const Data<S> &getData(void) const { return data_; }

  const std::vector<std::string> &getLeafLinks() const { return leaf_links_; }

  std::string getJointType(size_t index, bool user = true) const;

  std::vector<std::string> getJointTypes(bool user = true) const;

  MatrixX<S> getJointLimit(size_t index, bool user = true) const;

  std::vector<MatrixX<S>> getJointLimits(bool user = true) const;

  size_t getJointId(size_t index, bool user = true) const {
    return user ? vidx_[index] : model_.idx_vs[index];
  }

  VectorXi getJointIds(bool user = true) const;

  size_t getJointDim(size_t index, bool user = true) const {
    return user ? nvs_[index] : model_.nvs[index];
  }

  VectorXi getJointDims(bool user = true) const;

  size_t getParent(size_t index, bool user = true) const {
    return user ? parents_[index] : model_.parents[index];
  }

  VectorXi getParents(bool user = true) const;

  std::vector<std::string> getLinkNames(bool user = true) const;

  const std::vector<std::string> &getJointNames(bool user = true) const;

  std::vector<std::vector<size_t>> getSupports(bool user = true) const;

  std::vector<std::vector<size_t>> getSubtrees(bool user = true) const;

  // Frame is a Pinocchio internal data type which is not supported outside this
  // class.
  int getNFrames() const { return model_.nframes; }

  void printFrames() const;

  void setJointOrder(const std::vector<std::string> &names);

  void setLinkOrder(const std::vector<std::string> &names);

  std::vector<std::size_t> getChainJointIndex(const std::string &end_effector) const;

  std::vector<std::string> getChainJointName(const std::string &end_effector) const;

  VectorX<S> getRandomConfiguration() const;

  void computeForwardKinematics(const VectorX<S> &qpos);

  Vector7<S> getLinkPose(size_t index) const;

  Vector7<S> getJointPose(size_t index) const;  // TODO not same as sapien

  void computeFullJacobian(const VectorX<S> &qpos);

  Matrix6X<S> getLinkJacobian(size_t index, bool local = false) const;

  Matrix6X<S> computeSingleLinkLocalJacobian(const VectorX<S> &qpos, size_t index);

  std::tuple<VectorX<S>, bool, Vector6<S>> computeIKCLIK(
      size_t index, const Vector7<S> &pose, const VectorX<S> &q_init,
      const std::vector<bool> &mask, double eps = 1e-5, int maxIter = 1000,
      double dt = 1e-1, double damp = 1e-12);

  std::tuple<VectorX<S>, bool, Vector6<S>> computeIKCLIKJL(
      size_t index, const Vector7<S> &pose, const VectorX<S> &q_init,
      const VectorX<S> &qmin, const VectorX<S> &qmax, double eps = 1e-5,
      int maxIter = 1000, double dt = 1e-1, double damp = 1e-12);

 private:
  urdf::ModelInterfaceSharedPtr urdf_model_;
  Model<S> model_;
  Data<S> data_;

  VectorXi joint_index_user2pinocchio_, joint_index_pinocchio2user_;
  VectorXi v_index_user2pinocchio_;  // the joint index in model
  // map between user and pinocchio
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> v_map_user2pinocchio_;
  VectorXi link_index_user2pinocchio_;

  std::vector<std::string> user_link_names_;
  std::vector<std::string> user_joint_names_;
  std::vector<std::string> leaf_links_;
  VectorXi qidx_, vidx_, nqs_, nvs_, parents_;

  const std::string joint_prefix_ = "JointModel";
  bool verbose_;

  VectorX<S> qposUser2Pinocchio(const VectorX<S> &qpos) const;

  VectorX<S> qposPinocchio2User(const VectorX<S> &qpos) const;

  void dfs_parse_tree(const urdf::LinkConstSharedPtr &link,
                      UrdfVisitorBase<S> &visitor);

  void init(const urdf::ModelInterfaceSharedPtr &urdfTree, const Vector3<S> &gravity);
};

// Common Type Alias ==========================================================
using PinocchioModelf = PinocchioModelTpl<float>;
using PinocchioModeld = PinocchioModelTpl<double>;
using PinocchioModelfPtr = PinocchioModelTplPtr<float>;
using PinocchioModeldPtr = PinocchioModelTplPtr<double>;

// Explicit Template Instantiation Declaration ================================
#define DECLARE_TEMPLATE_PINOCCHIO_MODEL(S) extern template class PinocchioModelTpl<S>

DECLARE_TEMPLATE_PINOCCHIO_MODEL(float);
DECLARE_TEMPLATE_PINOCCHIO_MODEL(double);

}  // namespace mplib::pinocchio

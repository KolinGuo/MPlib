#pragma once

#include <urdf_model/types.h>
#include <urdf_world/types.h>

#include <vector>

#include "macros_utils.h"
#include "types.h"

namespace mplib::pinocchio {

// PinocchioModelTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(PinocchioModelTpl);

template <typename S>
class PinocchioModelTpl {
 private:
  urdf::ModelInterfaceSharedPtr urdf_model_;
  Model<S> model_;
  Data<S> data_;

  VectorXi joint_index_user2pinocchio_, joint_index_pinocchio2user_;
  VectorXi v_index_user2pinocchio_;  // the joint index in model
  // map between user and pinocchio
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>
      v_map_user2pinocchio_;
  VectorXi link_index_user2pinocchio_;

  std::vector<std::string> user_link_names_;
  std::vector<std::string> user_joint_names_;
  std::vector<std::string> leaf_links_;
  VectorXi qidx_, vidx_, nqs_, nvs_, parents_;

  const std::string joint_prefix_ = "JointModel";
  bool verbose_;

  VectorX<S> qposUser2Pinocchio(VectorX<S> const &qpos);

  VectorX<S> qposPinocchio2User(VectorX<S> const &qpos);

  void init(urdf::ModelInterfaceSharedPtr const &urdfTree,
            Vector3<S> const &gravity);

  void dfs_parse_tree(urdf::LinkConstSharedPtr link,
                      UrdfVisitorBase<S> &visitor);

 public:
  PinocchioModelTpl(urdf::ModelInterfaceSharedPtr const &urdfTree,
                    Vector3<S> const &gravity, bool const &verbose = true);

  PinocchioModelTpl(std::string const &urdf_filename, Vector3<S> const &gravity,
                    bool const &verbose = true);

  inline Model<S> &getModel(void) { return model_; }

  inline Data<S> &getData(void) { return data_; }

  std::vector<std::string> getLeafLinks() { return leaf_links_; }

  inline std::string getJointType(size_t const &index,
                                  bool const &user = true) {
    if (user)
      return model_.joints[joint_index_user2pinocchio_[index]].shortname();
    else
      return model_.joints[index].shortname();
  }

  inline std::vector<std::string> getJointTypes(bool const &user = true) {
    std::vector<std::string> ret;
    auto njoints = user ? user_joint_names_.size() : model_.joints.size();
    for (size_t i = 0; i < njoints; i++) ret.push_back(getJointType(i, user));
    return ret;
  }

  inline std::vector<MatrixX<S>> getJointLimits(bool const &user = true) {
    std::vector<MatrixX<S>> ret;
    auto njoints = user ? user_joint_names_.size() : model_.joints.size();
    for (size_t i = 0; i < njoints; i++) ret.push_back(getJointLimit(i, user));
    return ret;
  }

  inline MatrixX<S> getJointLimit(size_t const &index,
                                  bool const &user = true) {
    auto joint_type = getJointType(index, user);
    size_t pinocchio_idx = user ? joint_index_user2pinocchio_[index] : index;
    size_t start_idx = model_.idx_qs[pinocchio_idx],
           nq = model_.nqs[pinocchio_idx], dim_joint = getJointDim(index, user);
    MatrixX<S> ret;
    ASSERT(dim_joint == 1,
           "Only support simple joint the dim of joint is not 1!");
    // std::cout << joint_type << " " << joint_type[joint_prefix.size()] << " "
    // << joint_type[joint_prefix.size() + 1] << " " <<  nq << " " << dim_joint
    // << " " << std::endl;
    if (joint_type[joint_prefix_.size()] == 'P' ||
        (joint_type[joint_prefix_.size()] == 'R' &&
         joint_type[joint_prefix_.size() + 1] != 'U')) {
      ret = MatrixX<S>(nq, 2);
      for (size_t j = 0; j < nq; j++) {
        ret(j, 0) = model_.lowerPositionLimit[start_idx + j];
        ret(j, 1) = model_.upperPositionLimit[start_idx + j];
      }
    } else if (joint_type[joint_prefix_.size()] == 'R' &&
               joint_type[joint_prefix_.size() + 1] == 'U') {
      ret = MatrixX<S>(1, 2);
      ret(0, 0) = -3.14159265359, ret(0, 1) = 3.14159265359;
    }
    return ret;
  }

  inline size_t getJointId(size_t const &index, bool const &user = true) {
    return user ? vidx_[index] : model_.idx_vs[index];
  }

  inline VectorXi getJointIds(bool const &user = true) {
    if (user)
      return vidx_;
    else {
      auto ret = VectorXi(model_.idx_vs.size());
      for (size_t i = 0; i < model_.idx_vs.size(); i++)
        ret[i] = model_.idx_vs[i];
      return ret;
    }
  }

  inline size_t getJointDim(size_t const &index, bool const &user = true) {
    return user ? nvs_[index] : model_.nvs[index];
  }

  inline VectorXi getJointDims(bool const &user = true) {
    if (user)
      return nvs_;
    else {
      auto ret = VectorXi(model_.nvs.size());
      for (size_t i = 0; i < model_.nvs.size(); i++) ret[i] = model_.nvs[i];
      return ret;
    }
  }

  inline size_t getParent(size_t const &index, bool const &user = true) {
    return user ? parents_[index] : model_.parents[index];
  }

  inline VectorXi getParents(bool const &user = true) {
    if (user)
      return parents_;
    else {
      auto ret = VectorXi(model_.parents.size());
      for (size_t i = 0; i < model_.parents.size(); i++)
        ret[i] = model_.parents[i];
      return ret;
    }
  }

  inline std::vector<std::string> getLinkNames(bool const &user = true) {
    if (user)
      return user_link_names_;
    else {
      std::vector<std::string> link_names;
      for (size_t i = 0; i < model_.frames.size(); i++)
        if (model_.frames[i].type == ::pinocchio::BODY)
          link_names.push_back(model_.frames[i].name);
      return link_names;
    }
  }

  inline std::vector<std::string> getJointNames(bool const &user = true) {
    if (user)
      return user_joint_names_;
    else
      return model_.names;
  }

  std::vector<std::vector<size_t>> getSupports(bool const &user = true) {
    if (user) {
      std::vector<std::vector<size_t>> ret;
      return ret;
    } else
      return model_.supports;
  }

  std::vector<std::vector<size_t>> getSubtrees(bool const &user = true) {
    if (user) {
      std::vector<std::vector<size_t>> ret;
      return ret;
    } else
      return model_.subtrees;
  }

  // Frame is a Pinocchio internal data type which is not supported outside this
  // class.
  void printFrames(void);

  int getNFrames(void) { return model_.nframes; }

  /*
      // The original model for debug only
      inline size_t getDimQpos(void) { return model.nv; }

      int getNQ(void) { return model.nq; }

      int getNV(void) { return model.nv; }

      int getNJoints(void) { return model.njoints; }

      int getNBodies(void) { return model.nbodies; }

      std::vector<std::vector<size_t>> getSupports(void) { return
     model.supports; };

      std::vector<std::vector<size_t>> getSubtrees(void) { return
     model.subtrees; };

      int getNLinks(void);

      int getNFrames(void) { return model.nframes; }

      std::vector<int> getNQs(void) { return model.nqs; }

      std::vector<int> getNVs(void) { return model.nvs; }

      std::vector<int> getIDXVs(void) { return model.idx_vs; }

      std::vector<int> getIDXQs(void) { return model.idx_qs; }

      std::vector<std::string> getJointNames(void) { return model.names; }

      std::vector<std::string> getFrameNames(void);

      std::vector<std::string> getLinkNames(void);

      std::vector<size_t> getParents(void) { return model.parents; }

      void printFrames(void);*/
  //

  void setJointOrder(std::vector<std::string> const &names);

  void setLinkOrder(std::vector<std::string> const &names);

  std::vector<std::size_t> getChainJointIndex(std::string const &end_effector);

  std::vector<std::string> getChainJointName(std::string const &end_effector);

  VectorX<S> getRandomConfiguration();

  void computeForwardKinematics(VectorX<S> const &qpos);

  Vector7<S> getLinkPose(size_t const &index);

  Vector7<S> getJointPose(size_t const &index);  // TODO not same as sapien

  void computeFullJacobian(VectorX<S> const &qpos);

  Matrix6X<S> getLinkJacobian(size_t const &index, bool const &local = false);

  Matrix6X<S> computeSingleLinkLocalJacobian(VectorX<S> const &qpos,
                                             size_t const &index);

  std::tuple<VectorX<S>, bool, Vector6<S>> computeIKCLIK(
      size_t const &index, Vector7<S> const &pose, VectorX<S> const &q_init,
      std::vector<bool> const &mask, double const &eps = 1e-5,
      int const &maxIter = 1000, double const &dt = 1e-1,
      double const &damp = 1e-12);

  std::tuple<VectorX<S>, bool, Vector6<S>> computeIKCLIKJL(
      size_t const &index, Vector7<S> const &pose, VectorX<S> const &q_init,
      VectorX<S> const &qmin, VectorX<S> const &qmax, double const &eps = 1e-5,
      int const &maxIter = 1000, double const &dt = 1e-1,
      double const &damp = 1e-12);
};

// Common Type Alias ==========================================================
using PinocchioModelf = PinocchioModelTpl<float>;
using PinocchioModeld = PinocchioModelTpl<double>;
using PinocchioModelfPtr = PinocchioModelTplPtr<float>;
using PinocchioModeldPtr = PinocchioModelTplPtr<double>;

// Explicit Template Instantiation Declaration ================================
#define DECLARE_TEMPLATE_PINOCCHIO_MODEL(S) \
  extern template class PinocchioModelTpl<S>

DECLARE_TEMPLATE_PINOCCHIO_MODEL(float);
DECLARE_TEMPLATE_PINOCCHIO_MODEL(double);

}  // namespace mplib::pinocchio

#include "pinocchio_model.h"

#include <urdf_parser/urdf_parser.h>

#include "urdf_utils.h"
// #include <boost/property_tree/xml_parser.hpp>
// #include <boost/property_tree/ptree.hpp>

#define DEFINE_TEMPLATE_PM(DATATYPE) template class PinocchioModelTpl<DATATYPE>;

DEFINE_TEMPLATE_PM(double)

DEFINE_TEMPLATE_PM(float)

template <typename DATATYPE>
void PinocchioModelTpl<DATATYPE>::dfs_parse_tree(urdf::LinkConstSharedPtr link,
                                                 UrdfVisitorBase &visitor) {
  const urdf::JointConstSharedPtr joint =
      urdf::const_pointer_cast<urdf::Joint>(link->parent_joint);
  if (verbose_) std::cout << link->name << " " << joint.get() << std::endl;
  if (joint) {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(link->getParent() && link);

    const std::string &joint_name = joint->name;
    const std::string &link_name = link->name;
    const std::string &parent_link_name = link->getParent()->name;
    FrameIndex parent_frame_id = visitor.getBodyId(parent_link_name);

    // Transformation from the parent link to the joint origin
    const SE3 joint_placement =
        pose_to_se3<DATATYPE>(joint->parent_to_joint_origin_transform);
    const Inertia Y = convert_inertial<DATATYPE>(link->inertial);

    const DATATYPE infty = std::numeric_limits<DATATYPE>::infinity();
    VectorX max_effort(1), max_velocity(1), min_config(1), max_config(1);
    VectorX friction(VectorX::Constant(1, 0.)),
        damping(VectorX::Constant(1, 0.));
    Vector3 axis(joint->axis.x, joint->axis.y, joint->axis.z);

    std::ostringstream joint_info;

    switch (joint->type) {
      case urdf::Joint::FLOATING:
        joint_info << "joint FreeFlyer";

        max_effort = VectorX::Constant(6, infty);
        max_velocity = VectorX::Constant(6, infty);
        min_config = VectorX::Constant(7, -infty);
        max_config = VectorX::Constant(7, infty);
        min_config.tail(4).setConstant(-1.01);
        max_config.tail(4).setConstant(1.01);

        friction = VectorX::Constant(6, 0.);
        damping = VectorX::Constant(6, 0.);

        visitor.addJointAndBody(UrdfVisitorBase::FLOATING, axis,
                                parent_frame_id, joint_placement, joint->name,
                                Y, link->name, max_effort, max_velocity,
                                min_config, max_config, friction, damping);
        break;

      case ::urdf::Joint::REVOLUTE:
        joint_info << "joint REVOLUTE with axis";

        // TODO I think the URDF standard forbids REVOLUTE with no limits.
        ASSERT(joint->limits, "REVOLUTE without limits");
        if (joint->limits) {
          max_effort << joint->limits->effort;
          max_velocity << joint->limits->velocity;
          min_config << joint->limits->lower;
          max_config << joint->limits->upper;
        }

        if (joint->dynamics) {
          friction << joint->dynamics->friction;
          damping << joint->dynamics->damping;
        }

        visitor.addJointAndBody(UrdfVisitorBase::REVOLUTE, axis,
                                parent_frame_id, joint_placement, joint->name,
                                Y, link->name, max_effort, max_velocity,
                                min_config, max_config, friction, damping);
        break;

      case ::urdf::Joint::CONTINUOUS:  // Revolute joint with no joint limits
        joint_info << "joint CONTINUOUS with axis";

        min_config.resize(2);
        max_config.resize(2);
        min_config << -1.01, -1.01;
        max_config << 1.01, 1.01;

        if (joint->limits) {
          max_effort << joint->limits->effort;
          max_velocity << joint->limits->velocity;
        } else {
          max_effort << infty;
          max_velocity << infty;
        }

        if (joint->dynamics) {
          friction << joint->dynamics->friction;
          damping << joint->dynamics->damping;
        }

        visitor.addJointAndBody(UrdfVisitorBase::CONTINUOUS, axis,
                                parent_frame_id, joint_placement, joint->name,
                                Y, link->name, max_effort, max_velocity,
                                min_config, max_config, friction, damping);
        break;

      case ::urdf::Joint::PRISMATIC:
        joint_info << "joint PRISMATIC with axis";

        // TODO I think the URDF standard forbids REVOLUTE with no limits.
        ASSERT(joint->limits, "PRISMATIC without limits");
        if (joint->limits) {
          max_effort << joint->limits->effort;
          max_velocity << joint->limits->velocity;
          min_config << joint->limits->lower;
          max_config << joint->limits->upper;
        }

        if (joint->dynamics) {
          friction << joint->dynamics->friction;
          damping << joint->dynamics->damping;
        }

        visitor.addJointAndBody(UrdfVisitorBase::PRISMATIC, axis,
                                parent_frame_id, joint_placement, joint->name,
                                Y, link->name, max_effort, max_velocity,
                                min_config, max_config, friction, damping);
        break;

      case ::urdf::Joint::PLANAR:
        joint_info << "joint PLANAR with normal axis along Z";

        max_effort = VectorX::Constant(3, infty);
        max_velocity = VectorX::Constant(3, infty);
        min_config = VectorX::Constant(4, -infty);
        max_config = VectorX::Constant(4, infty);

        min_config.tail(2).setConstant(-1.01);
        max_config.tail(2).setConstant(1.01);

        friction = VectorX::Constant(3, 0.);
        damping = VectorX::Constant(3, 0.);

        visitor.addJointAndBody(UrdfVisitorBase::PLANAR, axis, parent_frame_id,
                                joint_placement, joint->name, Y, link->name,
                                max_effort, max_velocity, min_config,
                                max_config, friction, damping);
        break;

      case ::urdf::Joint::FIXED:
        joint_info << "fixed joint";
        visitor.addFixedJointAndBody(parent_frame_id, joint_placement,
                                     joint_name, Y, link_name);
        break;

      default:
        throw std::invalid_argument("The type of joint " + joint_name +
                                    " is not supported.");
    }

    FrameIndex frame_id = visitor.getBodyId(link->name);
    visitor << "Adding Body" << '\n'
            << '\"' << link_name << "\" connected to \"" << parent_link_name
            << "\" through joint \"" << joint_name << "\"\n"
            << "joint type: " << joint_info.str() << '\n'
            << "joint placement:\n"
            << joint_placement << '\n'
            << "body info: " << '\n'
            << "  mass: " << Y.mass() << '\n'
            << "  lever: " << Y.lever().transpose() << '\n'
            << "  inertia elements (Ixx,Iyx,Iyy,Izx,Izy,Izz): "
            << Y.inertia().data().transpose() << '\n'
            << '\n';
  } else if (link->getParent())
    throw std::invalid_argument(link->name + " - joint information missing.");
  for (auto child : link->child_links) dfs_parse_tree(child, visitor);
  if (link->child_links.empty()) leaf_links_.push_back(link->name);
}

template <typename DATATYPE>
void PinocchioModelTpl<DATATYPE>::init(
    urdf::ModelInterfaceSharedPtr const &urdfTree, Vector3 const &gravity) {
  urdf_model_ = urdfTree;

  pinocchio::urdf::details::UrdfVisitor<DATATYPE, 0,
                                        pinocchio::JointCollectionDefaultTpl>
      visitor(model_);
  if (verbose_) {
    std::cout << "Begin to parse URDF" << std::endl;
    visitor.log = &std::cout;
  }
  if (not urdf_model_)
    throw std::invalid_argument(
        "The XML stream does not contain a valid URDF model.");

  visitor.setName(urdf_model_->getName());
  urdf::LinkConstSharedPtr root_link = urdf_model_->getRoot();
  visitor.addRootJoint(convert_inertial<DATATYPE>(root_link->inertial),
                       root_link->name);
  for (auto child : root_link->child_links) dfs_parse_tree(child, visitor);

  model_.gravity = {gravity, Vector3{0, 0, 0}};
  data_ = Data(model_);
  if (verbose_)
    std::cout << "Begin to set joint order and link order" << std::endl;
  setJointOrder(getJointNames(false));
  setLinkOrder(getLinkNames(false));
}

template <typename DATATYPE>
PinocchioModelTpl<DATATYPE>::PinocchioModelTpl(std::string const &urdf_filename,
                                               Vector3 const &gravity,
                                               bool const &verbose)
    : verbose_(verbose) {
  // std::cout << "Verbose" << verbose << std::endl;
  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDFFile(urdf_filename);
  init(urdf, gravity);
}

template <typename DATATYPE>
PinocchioModelTpl<DATATYPE>::PinocchioModelTpl(
    urdf::ModelInterfaceSharedPtr const &urdfTree, Vector3 const &gravity,
    bool const &verbose)
    : verbose_(verbose) {
  init(urdfTree, gravity);
}

template <typename DATATYPE>
Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1>
PinocchioModelTpl<DATATYPE>::qposUser2Pinocchio(VectorX const &q_user) {
  VectorX q_pinocchio(model_.nq);
  size_t count = 0;
  for (size_t i = 0; i < joint_index_user2pinocchio_.size(); i++) {
    auto start_idx = model_.idx_qs[joint_index_user2pinocchio_[i]];
    switch (nqs_[i]) {
      case 0:  // FIX
        break;
      case 1:  // REVOLUTE OR PRISMATIC
        ASSERT(start_idx < model_.nq, "posS2P out of bound");
        q_pinocchio[start_idx] = q_user[count];
        break;
      case 2:  // CONTINUOUS
        ASSERT(start_idx + 1 < model_.nq, "posS2P out of bound");
        q_pinocchio[start_idx] = std::cos(q_user(count));
        q_pinocchio[start_idx + 1] = std::sin(q_user(count));
        break;
      default:
        throw std::runtime_error(
            "Unsupported joint in computation. Currently support: fixed, "
            "revolute, prismatic");
    }
    count += nvs_[i];
  }
  ASSERT(count == q_user.size(), "Qpos user2pinocchio failed");
  return q_pinocchio;
}

template <typename DATATYPE>
Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1>
PinocchioModelTpl<DATATYPE>::qposPinocchio2User(VectorX const &q_pinocchio) {
  VectorX q_user(model_.nv);
  uint32_t count = 0;
  for (size_t i = 0; i < joint_index_user2pinocchio_.size(); i++) {
    auto start_idx = model_.idx_qs[joint_index_user2pinocchio_[i]];
    // std::cout << "Joint " << i << " " << start_idx << " " <<
    // joint_tangent_dim[i] << " " << count << std::endl;
    switch (nqs_[i]) {
      case 0:  // FIX
        break;
      case 1:  // REVOLUTE OR PRISMATIC
        ASSERT(count < model_.nv, "posP2S out of bound");
        q_user[count] = q_pinocchio[start_idx];
        break;
      case 2:  // CONTINUOUS
        ASSERT(count < model_.nv, "posS2P out of bound");
        q_user[count] =
            std::atan2(q_pinocchio[start_idx + 1], q_pinocchio[start_idx]);
        break;
      default:
        throw std::runtime_error(
            "Unsupported joint in computation. Currently support: fixed, "
            "revolute, prismatic");
    }
    count += nvs_[i];
  }
  ASSERT(count == q_user.size(), "Qpos pinocchio2user failed");
  return q_user;
}

template <typename DATATYPE>
void PinocchioModelTpl<DATATYPE>::setJointOrder(
    std::vector<std::string> const &names) {
  user_joint_names_ = names;
  v_index_user2pinocchio_ = VectorXI(model_.nv);
  vidx_ = VectorXI(names.size());
  qidx_ = VectorXI(names.size());
  nvs_ = VectorXI(names.size());
  nqs_ = VectorXI(names.size());
  parents_ = VectorXI(names.size());
  joint_index_user2pinocchio_ = VectorXI(names.size());
  joint_index_pinocchio2user_ = VectorXI::Constant(model_.njoints, -1);

  // joint_start_index = Eigen::VectorXi(names.size());
  // joint_tangent_dim = Eigen::VectorXi(names.size());
  // joint_dim = Eigen::VectorXi(names.size());

  size_t len_v = 0, len_q = 0;
  for (size_t i = 0; i < names.size(); i++) {
    auto pinocchio_idx = model_.getJointId(names[i]);
    if (i == model_.njoints)
      throw std::invalid_argument(names[i] +
                                  " is a invalid name in setJointOrder");
    vidx_[i] = len_v;
    qidx_[i] = len_q;
    nvs_[i] = model_.nvs[pinocchio_idx];
    nqs_[i] = model_.nqs[pinocchio_idx];
    parents_[i] = model_.parents[pinocchio_idx];
    joint_index_user2pinocchio_[i] = pinocchio_idx;
    joint_index_pinocchio2user_[pinocchio_idx] = i;
    len_q += model_.nqs[pinocchio_idx];
    for (int j = 0; j < model_.nvs[pinocchio_idx]; j++)
      v_index_user2pinocchio_[len_v++] = model_.idx_vs[pinocchio_idx] + j;
  }
  ASSERT(len_v == model_.nv, "setJointOrder failed");
  v_map_user2pinocchio_ =
      Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>(
          v_index_user2pinocchio_);
}

template <typename DATATYPE>
void PinocchioModelTpl<DATATYPE>::setLinkOrder(
    std::vector<std::string> const &names) {
  user_link_names_ = names;
  link_index_user2pinocchio_ = VectorXI(names.size());
  for (size_t i = 0; i < names.size(); i++) {
    auto pinocchio_idx = model_.getFrameId(names[i], pinocchio::BODY);
    if (pinocchio_idx == model_.nframes)
      throw std::invalid_argument(names[i] +
                                  " is a invalid names in setLinkOrder");
    link_index_user2pinocchio_[i] = pinocchio_idx;
  }
}

template <typename DATATYPE>
void PinocchioModelTpl<DATATYPE>::printFrames(void) {
  std::cout << "Joint dim " << model_.joints.size() << " " << model_.nv << " "
            << model_.nvs.size() << " " << model_.idx_vs.size() << std::endl;
  std::cout << "Joint Tangent dim " << model_.nq << " " << model_.nqs.size()
            << " " << model_.idx_qs.size() << std::endl;
  std::cout << "Joint Limit " << model_.lowerPositionLimit.size() << " "
            << model_.upperPositionLimit.size() << std::endl;
  for (size_t i = 0; i < model_.frames.size(); i++) {
    std::string type_name = "NONE";
    auto frame = model_.frames[i];
    if (frame.type == pinocchio::OP_FRAME)
      type_name = "OP_FRAME";
    else if (frame.type == pinocchio::JOINT)
      type_name = "JOINT";
    else if (frame.type == pinocchio::FIXED_JOINT)
      type_name = "FIXED_JOINT";
    else if (frame.type == pinocchio::BODY)
      type_name = "BODY";
    else if (frame.type == pinocchio::SENSOR)
      type_name = "BODY";
    std::cout << "Frame " << i << " " << frame.name << " " << frame.parent
              << " " << type_name << " " << frame.previousFrame << std::endl;
  }
}

template <typename DATATYPE>
std::vector<std::size_t> PinocchioModelTpl<DATATYPE>::getChainJointIndex(
    const std::string &end_effector) {
  auto frame_id = model_.getBodyId(end_effector);
  std::vector<std::size_t> index_pinocchio =
      model_.supports[model_.frames[frame_id].parent];
  std::vector<std::size_t> ret;
  for (auto index : index_pinocchio)
    if (joint_index_pinocchio2user_[index] != -1)
      ret.push_back(joint_index_pinocchio2user_[index]);
  return ret;
}

template <typename DATATYPE>
std::vector<std::string> PinocchioModelTpl<DATATYPE>::getChainJointName(
    const std::string &end_effector) {
  auto frame_id = model_.getBodyId(end_effector);
  std::vector<std::size_t> index_pinocchio =
      model_.supports[model_.frames[frame_id].parent];
  std::vector<std::string> ret;
  for (auto index : index_pinocchio)
    if (joint_index_pinocchio2user_[index] != -1)
      ret.push_back(model_.names[index]);
  return ret;
}

template <typename DATATYPE>
Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1>
PinocchioModelTpl<DATATYPE>::getRandomConfiguration() {
  auto qpos = pinocchio::randomConfiguration(model_);
  return qposPinocchio2User(qpos);
}

template <typename DATATYPE>
void PinocchioModelTpl<DATATYPE>::computeForwardKinematics(
    VectorX const &qpos) {
  pinocchio::forwardKinematics(model_, data_, qposUser2Pinocchio(qpos));
}

template <typename DATATYPE>
Eigen::Matrix<DATATYPE, 7, 1> PinocchioModelTpl<DATATYPE>::getLinkPose(
    size_t const &index) {
  ASSERT(index < link_index_user2pinocchio_.size(),
         "The link index is out of bound!");
  auto frame = link_index_user2pinocchio_[index];
  auto parent_joint = model_.frames[frame].parent;
  auto link2joint = model_.frames[frame].placement;
  auto joint2world = data_.oMi[parent_joint];
  // std::cout << parent_joint << " " << link2joint << std::endl;
  // std::cout << joint2world << std::endl;

  /*
      auto parent_joint2 = model2.frames[frame].parent;
      auto link2joint2 = model2.frames[frame].placement;
      auto joint2world2 = data2.oMi[parent_joint];

      std::cout << "Pi link " << model.frames[frame].name << std::endl;
      std::cout << parent_joint << " " << parent_joint2 << std::endl;
      std::cout << link2joint << " " << link2joint2 << std::endl;
      std::cout << joint2world << " " << joint2world2 << std::endl;
  */
  auto link2world = joint2world * link2joint;
  auto p = link2world.translation();
  auto q = Quaternion(link2world.rotation());

  /*
  auto link2world2 = joint2world2 * link2joint2;
  auto p2 = link2world2.translation();
  auto q2 = Eigen::Quaternion<double>(link2world2.rotation());
  std::cout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() <<
  std::endl; std::cout << q2.w() << " " << q2.x() << " " << q2.y() << " " <<
  q2.z() << std::endl;
*/

  DATATYPE pose[] = {p.x(), p.y(), p.z(), q.w(), q.x(), q.y(), q.z()};
  return Vector7(pose);
}

template <typename DATATYPE>
Eigen::Matrix<DATATYPE, 7, 1> PinocchioModelTpl<DATATYPE>::getJointPose(
    size_t const &index) {
  ASSERT(index < joint_index_user2pinocchio_.size(),
         "The link index is out of bound!");
  auto frame = joint_index_user2pinocchio_[index];

  /* std::cout << index << " " << user_joint_names[index] << " "<< frame << " "
   << model.getJointId(user_joint_names[index]) << std::endl; std::cout <<
   data.oMi[0] << std::endl;

   std::cout << data.oMi[1] << std::endl;
*/
  // using Data = pinocchio::DataTpl<DATATYPE>

  auto joint2world = data_.oMi[frame];
  auto p = joint2world.translation();
  auto q = Quaternion(joint2world.rotation());
  DATATYPE pose[] = {p.x(), p.y(), p.z(), q.w(), q.x(), q.y(), q.z()};
  return Vector7(pose);
}

template <typename DATATYPE>
void PinocchioModelTpl<DATATYPE>::computeFullJacobian(const VectorX &qpos) {
  pinocchio::computeJointJacobians(model_, data_, qposUser2Pinocchio(qpos));
}

template <typename DATATYPE>
Eigen::Matrix<DATATYPE, 6, Eigen::Dynamic>
PinocchioModelTpl<DATATYPE>::getLinkJacobian(size_t const &index,
                                             bool const &local) {
  ASSERT(index < link_index_user2pinocchio_.size(), "link index out of bound");
  auto frameId = link_index_user2pinocchio_[index];
  auto jointId = model_.frames[frameId].parent;

  auto link2joint = model_.frames[frameId].placement;
  auto joint2world = data_.oMi[jointId];
  auto link2world = joint2world * link2joint;

  Matrix6x J(6, model_.nv);
  J.fill(0);

  pinocchio::getJointJacobian(model_, data_, jointId,
                              pinocchio::ReferenceFrame::WORLD, J);
  if (local) {
    J = link2world.toActionMatrixInverse() * J;
  }
  return J * v_map_user2pinocchio_;
}

template <typename DATATYPE>
Eigen::Matrix<DATATYPE, 6, Eigen::Dynamic>
PinocchioModelTpl<DATATYPE>::computeSingleLinkLocalJacobian(
    VectorX const &qpos, size_t const &index) {
  ASSERT(index < link_index_user2pinocchio_.size(), "link index out of bound");
  auto frameId = link_index_user2pinocchio_[index];
  auto jointId = model_.frames[frameId].parent;
  auto link2joint = model_.frames[frameId].placement;

  Eigen::Matrix<DATATYPE, 6, Eigen::Dynamic> J(6, model_.nv);
  J.fill(0);
  pinocchio::computeJointJacobian(model_, data_, qposUser2Pinocchio(qpos),
                                  jointId, J);
  return link2joint.toActionMatrixInverse() * J * v_map_user2pinocchio_;
}

template <typename DATATYPE>
std::tuple<Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1>, bool,
           Eigen::Matrix<DATATYPE, 6, 1>>
PinocchioModelTpl<DATATYPE>::computeIKCLIK(
    size_t const &index, Vector7 const &pose, VectorX const &q_init,
    std::vector<bool> const &mask, double const &eps, int const &maxIter,
    double const &dt, double const &damp) {
  ASSERT(index < link_index_user2pinocchio_.size(), "link index out of bound");
  auto frameId = link_index_user2pinocchio_[index];
  auto jointId = model_.frames[frameId].parent;

  SE3 link_pose;
  link_pose.translation({pose[0], pose[1], pose[2]});
  // w, x, y, z
  link_pose.rotation(
      Quaternion(pose[3], pose[4], pose[5], pose[6]).toRotationMatrix());
  auto link2joint = model_.frames[frameId].placement;

  SE3 joint_pose = link_pose * link2joint.inverse();
  VectorX q = qposUser2Pinocchio(q_init);  // pinocchio::neutral(model);
  Matrix6x J(6, model_.nv);
  J.setZero();

  bool success = false;
  Vector6 err;
  VectorX v(model_.nv);

  int mask_size = mask.size();
  for (int i = 0;; i++) {
    pinocchio::forwardKinematics(model_, data_, q);
    const SE3 dMi = joint_pose.actInv(data_.oMi[jointId]);
    err = pinocchio::log6(dMi).toVector();
    if (err.norm() < eps) {
      success = true;
      break;
    }
    if (i >= maxIter) {
      success = false;
      break;
    }
    pinocchio::computeJointJacobian(model_, data_, q, jointId, J);

    // mask out certain joints
    if (mask_size != 0) {
      for (int j = 0; j < mask_size; j++)
        if (mask[j]) {
          int u = joint_index_user2pinocchio_[j] - 1;
          for (int k = 0; k < 6; k++) J(k, u) = 0;
        }
    }
    Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model_, q, v * dt);
  }
  return {qposPinocchio2User(q), success, err};
}

template <typename DATATYPE>
std::tuple<Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1>, bool,
           Eigen::Matrix<DATATYPE, 6, 1>>
PinocchioModelTpl<DATATYPE>::computeIKCLIKJL(
    size_t const &index, Vector7 const &pose, VectorX const &q_init,
    VectorX const &q_min, VectorX const &q_max, double const &eps,
    int const &maxIter, double const &dt, double const &damp) {
  ASSERT(index < link_index_user2pinocchio_.size(), "link index out of bound");
  auto frameId = link_index_user2pinocchio_[index];
  auto jointId = model_.frames[frameId].parent;

  SE3 link_pose;
  link_pose.translation({pose[0], pose[1], pose[2]});
  // w, x, y, z
  link_pose.rotation(
      Quaternion(pose[3], pose[4], pose[5], pose[6]).toRotationMatrix());
  auto link2joint = model_.frames[frameId].placement;

  SE3 joint_pose = link_pose * link2joint.inverse();
  VectorX q = qposUser2Pinocchio(q_init);  // pinocchio::neutral(model);
  VectorX qmin = qposUser2Pinocchio(q_min);
  VectorX qmax = qposUser2Pinocchio(q_max);
  Matrix6x J(6, model_.nv);
  J.setZero();

  bool success = false;
  Vector6 err;
  VectorX v(model_.nv);

  for (int i = 0;; i++) {
    pinocchio::forwardKinematics(model_, data_, q);
    const SE3 dMi = joint_pose.actInv(data_.oMi[jointId]);
    err = pinocchio::log6(dMi).toVector();
    if (err.norm() < eps) {
      success = true;
      break;
    }
    if (i >= maxIter) {
      success = false;
      break;
    }
    pinocchio::computeJointJacobian(model_, data_, q, jointId, J);
    Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model_, q, v * dt);
    for (int j = 0; j < q.rows(); j++) {
      if (q[j] < qmin[j]) {
        if (abs(qmin[j] + 3.1415926) < 1e-3)
          q[j] = 3.1415926;
        else
          q[j] = qmin[j];
      }
      if (q[j] > qmax[j]) {
        if (abs(qmin[j] - 3.1415926) < 1e-3)
          q[j] = -3.1415926;
        else
          q[j] = qmax[j];
      }
    }
  }
  return {qposPinocchio2User(q), success, err};
}
/*


Eigen::MatrixXd PinocchioModelTpl::computeGeneralizedMassMatrix(const
Eigen::VectorXd &qpos) { pinocchio::crba(model, data, posS2P(qpos));
    data.M.triangularView<Eigen::StrictlyLower>() =
            data.M.transpose().triangularView<Eigen::StrictlyLower>();
    return indexS2P.transpose() * data.M * indexS2P;
}

Eigen::MatrixXd PinocchioModelTpl::computeCoriolisMatrix(const Eigen::VectorXd
&qpos, const Eigen::VectorXd &qvel) { return indexS2P.transpose() *
           pinocchio::computeCoriolisMatrix(model, data, posS2P(qpos), indexS2P
* qvel) * indexS2P;
}

Eigen::VectorXd PinocchioModelTpl::computeInverseDynamics(const Eigen::VectorXd
&qpos, const Eigen::VectorXd &qvel, const Eigen::VectorXd &qacc) { return
indexS2P.transpose() * pinocchio::rnea(model, data, posS2P(qpos), indexS2P *
qvel, indexS2P * qacc);
}

Eigen::VectorXd PinocchioModelTpl::computeForwardDynamics(const Eigen::VectorXd
&qpos, const Eigen::VectorXd &qvel, const Eigen::VectorXd &qf) { return
indexS2P.transpose() * pinocchio::aba(model, data, posS2P(qpos), indexS2P *
qvel, indexS2P * qf);
}

*/

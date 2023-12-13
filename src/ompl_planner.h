#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/RandomNumbers.h>

#include <pinocchio/fwd.hpp>

#include "planning_world.h"

namespace ob = ompl::base;
// namespace oc = ompl::control;
namespace og = ompl::geometric;

template <typename S>
std::vector<S> state2vector(const ob::State *state_raw,
                            ob::SpaceInformation *const &si_) {
  auto state = state_raw->as<ob::CompoundState>();
  std::vector<S> ret;
  auto si = si_->getStateSpace()->as<ob::CompoundStateSpace>();

  for (size_t i = 0; i < si->getSubspaceCount(); i++) {
    auto subspace(si->getSubspace(i));
    size_t n;
    switch (subspace->getType()) {
      case ob::STATE_SPACE_REAL_VECTOR:
        n = subspace->as<ob::RealVectorStateSpace>()->getDimension();
        for (size_t j = 0; j < n; j++)
          ret.push_back((S)(*state)[i]
                            ->as<ob::RealVectorStateSpace::StateType>()
                            ->values[j]);
        break;
      case ob::STATE_SPACE_SO2:
        ret.push_back(
            (S)(*state)[i]->as<ob::SO2StateSpace::StateType>()->value);
        break;
      default:
        throw std::invalid_argument("Unhandled subspace type.");
        break;
    }
  }
  return ret;
}

template <typename IN_TYPE, typename OUT_TYPE>
std::vector<OUT_TYPE> eigen2vector(
    Eigen::Matrix<IN_TYPE, Eigen::Dynamic, 1> const &x) {
  std::vector<OUT_TYPE> ret;
  for (size_t i = 0; i < x.rows(); i++) ret.push_back((OUT_TYPE)x[i]);
  return ret;
}

template <typename IN_TYPE, typename OUT_TYPE>
Eigen::Matrix<OUT_TYPE, Eigen::Dynamic, 1> vector2eigen(
    std::vector<IN_TYPE> const &x) {
  Eigen::Matrix<OUT_TYPE, Eigen::Dynamic, 1> ret(x.size());
  for (size_t i = 0; i < x.size(); i++) ret[i] = (OUT_TYPE)x[i];
  return ret;
}

template <typename S>
Eigen::Matrix<S, Eigen::Dynamic, 1> state2eigen(
    const ob::State *state_raw, ob::SpaceInformation *const &si_) {
  auto vec_ret = state2vector<S>(state_raw, si_);
  /*for (size_t i = 0; i < vec_ret.size(); i++)
      std::cout << vec_ret[i] << " ";
  std::cout << std::endl;
  */
  auto ret = vector2eigen<S, S>(vec_ret);
  /*std::cout << ret.rows() << " " << ret.cols() << std::endl;
  for (size_t i = 0; i < ret.rows(); i++)
      std::cout << ret[i] << " ";
  std::cout << std::endl;
  */
  return ret;
}

template <typename S>
class ValidityCheckerTpl : public ob::StateValidityChecker {
  using VectorX = Eigen::Matrix<S, Eigen::Dynamic, 1>;
  PlanningWorldTplPtr<S> world_;

 public:
  ValidityCheckerTpl(PlanningWorldTplPtr<S> world,
                     const ob::SpaceInformationPtr &si)
      : ob::StateValidityChecker(si), world_(world) {}

  bool isValid(const ob::State *state_raw) const {
    // std::cout << "Begin to check state" << std::endl;
    // std::cout << "check " << state2eigen<S>(state_raw, si_) <<
    // std::endl;
    world_->setQposAll(state2eigen<S>(state_raw, si_));
    return !world_->collide();
  }

  bool _isValid(VectorX state) const {
    world_->setQposAll(state);
    return !world_->collide();
  }
};

template <typename S>
using ValidityCheckerTplPtr = std::shared_ptr<ValidityCheckerTpl<S>>;

using ValidityCheckerdPtr = ValidityCheckerTplPtr<double>;
using ValidityCheckerfPtr = ValidityCheckerTplPtr<float>;
using ValidityCheckerd = ValidityCheckerTpl<double>;
using ValidityCheckerf = ValidityCheckerTpl<float>;

template <typename S>
class OMPLPlannerTpl {
  using CompoundStateSpacePtr = std::shared_ptr<ob::CompoundStateSpace>;
  using SpaceInformationPtr = std::shared_ptr<ob::SpaceInformation>;
  using ProblemDefinitionPtr = std::shared_ptr<ob::ProblemDefinition>;

  using CompoundStateSpace = ob::CompoundStateSpace;
  using SpaceInformation = ob::SpaceInformation;
  using ProblemDefinition = ob::ProblemDefinition;
  using ValidityChecker = ValidityCheckerTpl<S>;
  using ValidityCheckerPtr = ValidityCheckerTplPtr<S>;

  DEFINE_TEMPLATE_EIGEN(S)

  CompoundStateSpacePtr cs_;
  SpaceInformationPtr si_;
  ProblemDefinitionPtr pdef_;
  PlanningWorldTplPtr<S> world_;
  ValidityCheckerTplPtr<S> valid_checker_;
  size_t dim_;
  std::vector<S> lower_joint_limits_, upper_joint_limits_;
  std::vector<bool> is_revolute_;

 public:
  VectorX random_sample_nearby(VectorX const &start_state);

  OMPLPlannerTpl(PlanningWorldTplPtr<S> const &world);

  void build_state_space();

  PlanningWorldTplPtr<S> get_world() { return world_; }

  size_t get_dim() { return dim_; }

  std::pair<std::string, Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>> plan(
      VectorX const &start_state, std::vector<VectorX> const &goal_states,
      const std::string &planner_name = "RRTConnect", const double &time = 1.0,
      const double &range = 0.0, const bool &verbose = false);
};

template <typename S>
using OMPLPlannerTplPtr = std::shared_ptr<ValidityCheckerTpl<S>>;

using OMPLPlannerTpldPtr = OMPLPlannerTplPtr<double>;
using OMPLPlannerTplfPtr = OMPLPlannerTplPtr<float>;
using OMPLPlannerTpld = OMPLPlannerTpl<double>;
using OMPLPlannerTplf = OMPLPlannerTpl<float>;

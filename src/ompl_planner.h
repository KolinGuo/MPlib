#pragma once

#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <vector>

/* #include <ompl/base/goals/GoalStates.h> */
/* #include <ompl/base/objectives/MaximizeMinClearanceObjective.h> */
/* #include <ompl/base/objectives/PathLengthOptimizationObjective.h> */
/* #include <ompl/base/objectives/StateCostIntegralObjective.h> */
/* #include <ompl/base/samplers/ObstacleBasedValidStateSampler.h> */
/* #include <ompl/geometric/PathSimplifier.h> */
/* #include <ompl/geometric/SimpleSetup.h> */
/* #include <ompl/geometric/planners/rrt/RRTstar.h> */
/* #include <ompl/util/RandomNumbers.h> */

#include "macros_utils.h"
#include "planning_world.h"
#include "types.h"

namespace mplib::ompl {

template <typename S>
std::vector<S> state2vector(const ob::State *const &state_raw,
                            const SpaceInformation *const &si_);

template <typename IN_TYPE, typename OUT_TYPE>
std::vector<OUT_TYPE> eigen2vector(VectorX<IN_TYPE> const &x) {
  std::vector<OUT_TYPE> ret;
  for (size_t i = 0; i < static_cast<size_t>(x.rows()); i++)
    ret.push_back(static_cast<OUT_TYPE>(x[i]));
  return ret;
}

template <typename IN_TYPE, typename OUT_TYPE>
VectorX<OUT_TYPE> vector2eigen(std::vector<IN_TYPE> const &x) {
  VectorX<OUT_TYPE> ret(x.size());
  for (size_t i = 0; i < x.size(); i++) ret[i] = static_cast<OUT_TYPE>(x[i]);
  return ret;
}

template <typename S>
VectorX<S> state2eigen(const ob::State *const &state_raw,
                       const SpaceInformation *const &si_) {
  auto vec_ret = state2vector<S>(state_raw, si_);
  auto ret = vector2eigen<S, S>(vec_ret);
  return ret;
}

// ValidityCheckerTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(ValidityCheckerTpl);

template <typename S>
class ValidityCheckerTpl : public ob::StateValidityChecker {
 public:
  ValidityCheckerTpl(const PlanningWorldTplPtr<S> &world,
                     const SpaceInformationPtr &si)
      : ob::StateValidityChecker(si), world_(world) {}

  bool isValid(const ob::State *state_raw) const {
    // std::cout << "Begin to check state" << std::endl;
    // std::cout << "check " << state2eigen<S>(state_raw, si_) <<
    // std::endl;
    world_->setQposAll(state2eigen<S>(state_raw, si_));
    return !world_->collide();
  }

  bool _isValid(const VectorX<S> &state) const {
    world_->setQposAll(state);
    return !world_->collide();
  }

 private:
  PlanningWorldTplPtr<S> world_;
};

// Common Type Alias ==========================================================
using ValidityCheckerf = ValidityCheckerTpl<float>;
using ValidityCheckerd = ValidityCheckerTpl<double>;
using ValidityCheckerfPtr = ValidityCheckerTplPtr<float>;
using ValidityCheckerdPtr = ValidityCheckerTplPtr<double>;

// OMPLPlannerTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(OMPLPlannerTpl);

template <typename S>
class OMPLPlannerTpl {
 public:
  OMPLPlannerTpl(PlanningWorldTplPtr<S> const &world);

  const PlanningWorldTplPtr<S> &get_world() const { return world_; }

  size_t get_dim() const { return dim_; }

  VectorX<S> random_sample_nearby(VectorX<S> const &start_state) const;

  std::pair<std::string, MatrixX<S>> plan(
      VectorX<S> const &start_state, std::vector<VectorX<S>> const &goal_states,
      const std::string &planner_name = "RRTConnect", double time = 1.0,
      double range = 0.0, bool verbose = false) const;

 private:
  CompoundStateSpacePtr cs_;
  SpaceInformationPtr si_;
  ProblemDefinitionPtr pdef_;
  PlanningWorldTplPtr<S> world_;
  ValidityCheckerTplPtr<S> valid_checker_;
  size_t dim_;
  std::vector<S> lower_joint_limits_, upper_joint_limits_;
  std::vector<bool> is_revolute_;

  void build_state_space();
};

// Common Type Alias ==========================================================
using OMPLPlannerTplf = OMPLPlannerTpl<float>;
using OMPLPlannerTpld = OMPLPlannerTpl<double>;
using OMPLPlannerTplfPtr = OMPLPlannerTplPtr<float>;
using OMPLPlannerTpldPtr = OMPLPlannerTplPtr<double>;

// Explicit Template Instantiation Declaration ================================
#define DECLARE_TEMPLATE_OMPL_PLANNER(S)                                      \
  extern template std::vector<S> state2vector<S>(                             \
      const ob::State *const &state_raw, const SpaceInformation *const &si_); \
  extern template class ValidityCheckerTpl<S>;                                \
  extern template class OMPLPlannerTpl<S>

DECLARE_TEMPLATE_OMPL_PLANNER(float);
DECLARE_TEMPLATE_OMPL_PLANNER(double);

}  // namespace mplib::ompl

#include "ompl_planner.h"

#include <memory>

#include <ompl/base/Planner.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "macros_utils.h"

namespace mplib::ompl {

// Explicit Template Instantiation Definition =================================
#define DEFINE_TEMPLATE_OMPL_PLANNER(S)                                        \
  template std::vector<S> state2vector<S>(const ob::State *const &state_raw,   \
                                          const SpaceInformation *const &si_); \
  template class ValidityCheckerTpl<S>;                                        \
  template class OMPLPlannerTpl<S>

DEFINE_TEMPLATE_OMPL_PLANNER(float);
DEFINE_TEMPLATE_OMPL_PLANNER(double);

#define PI 3.14159265359

template <typename S>
std::vector<S> state2vector(const ob::State *const &state_raw,
                            const SpaceInformation *const &si_) {
  auto state = state_raw->as<ob::CompoundState>();
  std::vector<S> ret;
  auto si = si_->getStateSpace()->as<CompoundStateSpace>();

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

template <typename S>
OMPLPlannerTpl<S>::OMPLPlannerTpl(const PlanningWorldTplPtr<S> &world)
    : world_(world) {
  build_state_space();
  si_ = std::make_shared<SpaceInformation>(cs_);
  valid_checker_ = std::make_shared<ValidityCheckerTpl<S>>(world, si_);
  si_->setStateValidityChecker(valid_checker_);

  pdef_ = std::make_shared<ProblemDefinition>(si_);
}

template <typename S>
VectorX<S> OMPLPlannerTpl<S>::random_sample_nearby(
    const VectorX<S> &start_state) const {
  int cnt = 0;
  while (true) {
    S ratio = (S)(cnt + 1) / 1000;
    VectorX<S> new_state = start_state;
    for (size_t i = 0; i < dim_; i++) {
      S r = (S)rand() / RAND_MAX * 2 - 1;
      new_state[i] +=
          (upper_joint_limits_[i] - lower_joint_limits_[i]) * ratio * r;
      if (new_state[i] < lower_joint_limits_[i])
        new_state[i] = lower_joint_limits_[i];
      else if (new_state[i] > upper_joint_limits_[i])
        new_state[i] = upper_joint_limits_[i];
    }
    if (valid_checker_->_isValid(new_state)) {
      std::cout << "successfully sampled a new state with a perturbation of "
                << ratio * 100 << "% joint limits." << std::endl;
      return new_state;
    }
    cnt += 1;
    if (cnt > 1000) return start_state;
  }
}

template <typename S>
std::pair<std::string, MatrixX<S>> OMPLPlannerTpl<S>::plan(
    const VectorX<S> &start_state, const std::vector<VectorX<S>> &goal_states,
    const std::string &planner_name, double time, double range,
    double goal_bias, double pathlen_obj_weight, bool pathlen_obj_only,
    bool verbose) const {
  ASSERT(start_state.rows() == goal_states[0].rows(),
         "Length of start state and goal state should be equal");
  ASSERT(static_cast<size_t>(start_state.rows()) == dim_,
         "Length of start state and problem dimension should be equal");
  if (verbose == false) ::ompl::msg::noOutputHandler();

  ob::ScopedState<> start(cs_);
  start = eigen2vector<S, double>(start_state);

  bool invalid_start = !valid_checker_->_isValid(start_state);
  if (invalid_start) {
    std::cout << "invalid start state!! (collision)" << std::endl;
    VectorX<S> new_start_state = random_sample_nearby(start_state);
    start = eigen2vector<S, double>(new_start_state);
  }

  auto goals = std::make_shared<ob::GoalStates>(si_);

  int tot_enum_states = 1, tot_goal_state = 0;
  for (size_t i = 0; i < dim_; i++) tot_enum_states *= 3;

  for (size_t ii = 0; ii < goal_states.size(); ii++)
    for (int i = 0; i < tot_enum_states; i++) {
      std::vector<double> tmp_state;
      int tmp = i;
      bool flag = true;
      for (size_t j = 0; j < dim_; j++) {
        tmp_state.push_back(goal_states[ii](j));
        int dir = tmp % 3;
        tmp /= 3;
        if (dir != 0 && is_revolute_[j] == false) {
          flag = false;
          break;
        }
        if (dir == 1) {
          if (tmp_state[j] - 2 * PI > lower_joint_limits_[j])
            tmp_state[j] -= 2 * PI;
          else {
            flag = false;
            break;
          }
        } else if (dir == 2) {
          if (tmp_state[j] + 2 * PI < upper_joint_limits_[j])
            tmp_state[j] += 2 * PI;
          else {
            flag = false;
            break;
          }
        }
      }
      if (flag) {
        ob::ScopedState<> goal(cs_);
        goal = tmp_state;
        goals->addState(goal);
        tot_goal_state += 1;
      }
    }
  if (verbose)
    std::cout << "number of goal state: " << tot_goal_state << std::endl;

  pdef_->clearStartStates();
  pdef_->clearGoal();
  pdef_->clearSolutionPaths();
  pdef_->clearSolutionNonExistenceProof();
  // pdef->setStartAndGoalStates(start, goal);
  pdef_->setGoal(goals);
  pdef_->addStartState(start);
  ob::PlannerPtr planner;
  if (planner_name == "RRTConnect") {
    auto rrt_connect = std::make_shared<og::RRTConnect>(si_);
    if (range > 1E-6) rrt_connect->setRange(range);
    planner = rrt_connect;
  } else if (planner_name == "RRT") {
    auto rrt = std::make_shared<og::RRT>(si_);
    if (range > 1E-6) rrt->setRange(range);
    rrt->setGoalBias(goal_bias);
    planner = rrt;
  } else {
    // Create optimization objective
    auto length_objective =
        std::make_shared<ob::PathLengthOptimizationObjective>(si_);
    auto clear_objective =
        std::make_shared<ob::MaximizeMinClearanceObjective>(si_);
    if (pathlen_obj_only)
      pdef_->setOptimizationObjective(length_objective);
    else
      pdef_->setOptimizationObjective(pathlen_obj_weight * length_objective +
                                      clear_objective);
    if (planner_name == "PRMstar")
      planner = std::make_shared<og::PRMstar>(si_);
    else if (planner_name == "LazyPRMstar") {
      auto lazy_prm_star = std::make_shared<og::LazyPRMstar>(si_);
      if (range > 1E-6) lazy_prm_star->setRange(range);
      planner = lazy_prm_star;
    } else if (planner_name == "RRTstar") {
      auto rrt_star = std::make_shared<og::RRTstar>(si_);
      if (range > 1E-6) rrt_star->setRange(range);
      rrt_star->setGoalBias(goal_bias);
      planner = rrt_star;
    } else if (planner_name == "RRTsharp") {
      auto rrt_sharp = std::make_shared<og::RRTsharp>(si_);
      if (range > 1E-6) rrt_sharp->setRange(range);
      rrt_sharp->setGoalBias(goal_bias);
      planner = rrt_sharp;
    } else if (planner_name == "RRTXstatic") {
      auto rrtx_static = std::make_shared<og::RRTXstatic>(si_);
      if (range > 1E-6) rrtx_static->setRange(range);
      rrtx_static->setGoalBias(goal_bias);
      planner = rrtx_static;
    } else if (planner_name == "InformedRRTstar") {
      auto informed_rrt_star = std::make_shared<og::InformedRRTstar>(si_);
      if (range > 1E-6) informed_rrt_star->setRange(range);
      informed_rrt_star->setGoalBias(goal_bias);
      planner = informed_rrt_star;
    } else
      throw std::runtime_error("Planner Not implemented");
  }

  planner->setProblemDefinition(pdef_);
  planner->setup();
  if (verbose) std::cout << "OMPL setup" << std::endl;
  ob::PlannerStatus solved = planner->ob::Planner::solve(time);
  if (solved) {
    if (verbose) std::cout << "Solved!" << std::endl;
    ob::PathPtr path = pdef_->getSolutionPath();
    auto geo_path = std::dynamic_pointer_cast<og::PathGeometric>(path);
    size_t len = geo_path->getStateCount();
    MatrixX<S> ret(len + invalid_start, dim_);
    if (verbose) std::cout << "Result size " << len << " " << dim_ << std::endl;
    if (invalid_start) {
      for (size_t j = 0; j < dim_; j++) ret(0, j) = start_state(j);
    }
    for (size_t i = 0; i < len; i++) {
      auto res_i = state2eigen<S>(geo_path->getState(i), si_.get());
      // std::cout << "Size_i " << res_i.rows() << std::endl;
      ASSERT(static_cast<size_t>(res_i.rows()) == dim_,
             "Result dimension is not correct!");
      for (size_t j = 0; j < dim_; j++) ret(invalid_start + i, j) = res_i[j];
    }
    return std::make_pair(solved.asString(), ret);
  } else {
    MatrixX<S> ret(0, dim_);
    return std::make_pair(solved.asString(), ret);
  }
}

template <typename S>
void OMPLPlannerTpl<S>::build_state_space() {
  cs_ = std::make_shared<CompoundStateSpace>();
  dim_ = 0;
  const std::string joint_prefix = "JointModel";
  for (const auto &robot : world_->getPlannedArticulations()) {
    size_t dim_i = 0;
    auto model = robot->getPinocchioModel();
    auto joint_types = model->getJointTypes();
    auto d =
        robot->getQposDim();  // TODO!!! only construct for move group joints
    auto indices = robot->getMoveGroupJointIndices();
    ASSERT(d == indices.size(), "QposDim != size of the movegroup joints");
    for (size_t i = 0; i < d; i++) {
      auto id = indices[i];
      auto joint_type = joint_types[id];
      if (joint_type[joint_prefix.size()] == 'P' ||
          (joint_type[joint_prefix.size()] == 'R' &&
           joint_type[joint_prefix.size() + 1] !=
               'U'))  // PRISMATIC and REVOLUTE
      {
        auto bound = model->getJointLimit(id);
        auto subspcae =
            std::make_shared<ob::RealVectorStateSpace>(bound.rows());
        auto ob_bounds = ob::RealVectorBounds(bound.rows());
        dim_i += bound.rows();
        for (size_t j = 0; j < static_cast<size_t>(bound.rows()); j++) {
          lower_joint_limits_.push_back(bound(j, 0));
          upper_joint_limits_.push_back(bound(j, 1));
          ob_bounds.setLow(j, bound(j, 0)), ob_bounds.setHigh(j, bound(j, 1));
        }
        subspcae->setBounds(ob_bounds);
        cs_->addSubspace(subspcae, 1.0);
      } else if (joint_type[joint_prefix.size()] == 'R' &&
                 joint_type[joint_prefix.size() + 1] == 'U') {
        cs_->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);
        lower_joint_limits_.push_back(-PI);
        upper_joint_limits_.push_back(PI);
        dim_i += 1;
      }
      if (joint_type[joint_prefix.size()] == 'R' ||
          joint_type[joint_prefix.size()] == 'P') {
        if (joint_type[joint_prefix.size()] == 'R' &&
            joint_type[joint_prefix.size() + 1] != 'U')
          is_revolute_.push_back(true);
        else
          is_revolute_.push_back(false);
      }
    }
    ASSERT(dim_i == robot->getQposDim(),
           "Dim of bound is different from dim of qpos " +
               std::to_string(dim_i) + " " +
               std::to_string(robot->getQposDim()));
    dim_ += dim_i;
  }
}

}  // namespace mplib::ompl

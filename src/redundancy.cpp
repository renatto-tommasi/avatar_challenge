// Copyright 2026 Renatto Tommasi

#include "avatar_challenge/redundancy.hpp"

#include <moveit/collision_detection/collision_common.h>

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace avatar_challenge
{
namespace
{
constexpr double kEps = 1e-9;
}  // namespace

RedundancyResolver::RedundancyResolver(
  moveit::core::RobotModelConstPtr robot_model, const std::string & group_name,
  const std::string & tip_link, RedundancyOptions options)
: robot_model_(std::move(robot_model)),
  tip_link_(tip_link),
  options_(std::move(options))
{
  group_ = robot_model_->getJointModelGroup(group_name);
  if (group_ == nullptr) {
    throw std::runtime_error("no joint model group named '" + group_name + "'");
  }
  if (robot_model_->getLinkModel(tip_link_) == nullptr) {
    throw std::runtime_error("no link named '" + tip_link_ + "'");
  }

  // An empty scene over the robot model still carries the SRDF's allowed
  // collision matrix, which is all that is needed for self-collision checks.
  scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  scratch_ = std::make_shared<moveit::core::RobotState>(robot_model_);
  scratch_->setToDefaultValues();

  const std::vector<std::string> & names = group_->getVariableNames();
  const moveit::core::JointBoundsVector & bounds = group_->getActiveJointModelsBounds();
  lower_.reserve(names.size());
  upper_.reserve(names.size());
  for (const auto * joint_bounds : bounds) {
    // Every active joint in this group is a single-variable revolute joint.
    const moveit::core::VariableBounds & b = joint_bounds->front();
    lower_.push_back(b.position_bounded_ ? b.min_position_ : -M_PI);
    upper_.push_back(b.position_bounded_ ? b.max_position_ : M_PI);
  }

  if (options_.joint_weights.size() != lower_.size()) {
    options_.joint_weights.assign(lower_.size(), 1.0);
  }
}

double RedundancyResolver::weightedDistance(
  const std::vector<double> & a, const std::vector<double> & b) const
{
  double sum = 0.0;
  const std::size_t n = std::min(a.size(), b.size());
  for (std::size_t i = 0; i < n; ++i) {
    const double w = options_.joint_weights[i];
    const double d = a[i] - b[i];
    sum += w * d * d;
  }
  return std::sqrt(sum);
}

bool RedundancyResolver::solveIk(
  const Eigen::Isometry3d & pose, const std::vector<double> & seed,
  std::vector<double> & solution) const
{
  scratch_->setJointGroupPositions(group_, seed);
  scratch_->update();
  if (!scratch_->setFromIK(group_, pose, tip_link_, options_.ik_timeout)) {
    return false;
  }
  scratch_->update();
  scratch_->copyJointGroupPositions(group_, solution);
  return true;
}

bool RedundancyResolver::nullSpaceDirection(
  const std::vector<double> & joints, Eigen::VectorXd & direction) const
{
  scratch_->setJointGroupPositions(group_, joints);
  scratch_->update();

  Eigen::MatrixXd jacobian;
  if (!scratch_->getJacobian(
      group_, robot_model_->getLinkModel(tip_link_), Eigen::Vector3d::Zero(), jacobian, false))
  {
    return false;
  }

  // J is 6 x n with n = 7. Its null space is spanned by the right singular
  // vectors whose singular values vanish — for a non-singular 7-DoF arm that
  // is exactly one vector, the last column of V.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullV);
  const Eigen::VectorXd sigma = svd.singularValues();
  if (sigma.size() < 6 || sigma(5) < 1e-4) {
    return false;  // at/near a singularity the null space is not 1-dimensional
  }
  direction = svd.matrixV().col(jacobian.cols() - 1).normalized();
  return true;
}

double RedundancyResolver::manipulability(const std::vector<double> & joints) const
{
  scratch_->setJointGroupPositions(group_, joints);
  scratch_->update();

  Eigen::MatrixXd jacobian;
  if (!scratch_->getJacobian(
      group_, robot_model_->getLinkModel(tip_link_), Eigen::Vector3d::Zero(), jacobian, false))
  {
    return 0.0;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  double product = 1.0;
  for (int i = 0; i < std::min<int>(6, svd.singularValues().size()); ++i) {
    product *= svd.singularValues()(i);
  }
  return product;
}

double RedundancyResolver::limitPenalty(const std::vector<double> & joints) const
{
  double penalty = 0.0;
  for (std::size_t i = 0; i < joints.size() && i < lower_.size(); ++i) {
    const double mid = 0.5 * (lower_[i] + upper_[i]);
    const double half_range = 0.5 * (upper_[i] - lower_[i]);
    if (half_range < kEps) {
      continue;
    }
    const double normalised = (joints[i] - mid) / half_range;  // 0 mid-range, +-1 at a limit
    penalty += normalised * normalised;
  }
  return penalty / std::max<std::size_t>(1, joints.size());
}

bool RedundancyResolver::isUsable(const std::vector<double> & joints) const
{
  for (std::size_t i = 0; i < joints.size() && i < lower_.size(); ++i) {
    if (joints[i] < lower_[i] + options_.limit_margin ||
      joints[i] > upper_[i] - options_.limit_margin)
    {
      return false;
    }
  }

  scratch_->setJointGroupPositions(group_, joints);
  scratch_->update();
  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult result;
  request.contacts = false;
  scene_->checkSelfCollision(request, result, *scratch_);
  return !result.collision;
}

void RedundancyResolver::scoreCandidate(
  const std::vector<double> & current_joints, const std::vector<Eigen::Isometry3d> & waypoints,
  ConfigCandidate & candidate) const
{
  const int stride = std::max(1, options_.evaluation_stride);

  std::vector<double> previous = candidate.joints;
  std::vector<double> solution;
  double path_cost = 0.0;
  double limit_cost = limitPenalty(previous);
  double min_manip = manipulability(previous);
  std::size_t checked = 1;
  std::size_t reached = 1;
  bool broke = false;

  // Walk the trace exactly the way MoveIt's Cartesian interpolator will: solve
  // each waypoint seeded from the previous solution, so the configuration is
  // carried forward and any discontinuity shows up here rather than at
  // execution time.
  for (std::size_t i = stride; i < waypoints.size(); i += stride) {
    ++checked;
    if (!solveIk(waypoints[i], previous, solution) || !isUsable(solution)) {
      broke = true;
      break;
    }
    const double step = weightedDistance(previous, solution);
    // A large jump means IK hopped to a different branch of the solution set;
    // the Cartesian planner would reject that as a configuration-space jump.
    if (step > 0.6) {
      broke = true;
      break;
    }
    path_cost += step;
    limit_cost = std::max(limit_cost, limitPenalty(solution));
    min_manip = std::min(min_manip, manipulability(solution));
    previous = solution;
    ++reached;
  }

  const std::size_t total = 1 + (waypoints.size() > 1 ? (waypoints.size() - 1 + stride - 1) /
    stride : 0);
  candidate.reach_fraction =
    broke ? static_cast<double>(reached) / static_cast<double>(std::max<std::size_t>(1, total))
          : 1.0;
  (void)checked;

  candidate.approach_cost = weightedDistance(current_joints, candidate.joints);
  candidate.path_cost = path_cost;
  candidate.limit_cost = limit_cost;
  candidate.min_manipulability = min_manip;
  // Manipulability is a volume-like quantity; its reciprocal is the natural
  // "how close to degenerate" penalty. Normalised by a nominal 0.05 so the
  // weight stays comparable to the other, roughly unit-scale, terms.
  candidate.manip_cost = 0.05 / std::max(min_manip, 1e-4);

  candidate.feasible = candidate.reach_fraction >= options_.min_reach_fraction;
  candidate.total_cost = options_.w_approach * candidate.approach_cost +
    options_.w_path * candidate.path_cost +
    options_.w_limits * candidate.limit_cost +
    options_.w_manip * candidate.manip_cost;
  if (!candidate.feasible) {
    // Keep the finite score for reporting, but push it behind every feasible
    // candidate and rank the failures by how far they got.
    candidate.total_cost += 1e3 * (1.0 - candidate.reach_fraction) + 1e3;
  }
}

std::vector<ConfigCandidate> RedundancyResolver::evaluate(
  const std::vector<double> & current_joints, const Eigen::Isometry3d & entry_pose,
  const std::vector<Eigen::Isometry3d> & waypoints, double spin)
{
  std::vector<ConfigCandidate> candidates;

  // Step 1: land on the self-motion manifold, seeded from where the arm is.
  std::vector<double> anchor;
  if (!solveIk(entry_pose, current_joints, anchor)) {
    // The direct seed failed; fall back to random restarts before giving up.
    for (int attempt = 0; attempt < 20 && anchor.empty(); ++attempt) {
      scratch_->setToRandomPositions(group_);
      std::vector<double> seed;
      scratch_->copyJointGroupPositions(group_, seed);
      std::vector<double> solution;
      if (solveIk(entry_pose, seed, solution)) {
        anchor = solution;
      }
    }
    if (anchor.empty()) {
      return candidates;  // entry pose is genuinely unreachable
    }
  }

  std::vector<std::vector<double>> configs;
  std::vector<double> parameters;
  const auto add_unique = [&](const std::vector<double> & q, double parameter) {
      if (!isUsable(q)) {
        return false;
      }
      for (const std::vector<double> & seen : configs) {
        if (weightedDistance(seen, q) < options_.dedup_distance) {
          return false;
        }
      }
      configs.push_back(q);
      parameters.push_back(parameter);
      return true;
    };
  add_unique(anchor, 0.0);

  // Step 2: walk the manifold in both directions. Each step moves along the
  // Jacobian's null space (which leaves the tool pose unchanged to first order)
  // and then re-solves IK to cancel the second-order drift, so every sample is
  // an exact solution for the same entry pose but a different elbow posture.
  for (const int direction : {+1, -1}) {
    std::vector<double> walker = anchor;
    double parameter = 0.0;
    for (int step = 0; step < options_.samples_per_direction; ++step) {
      Eigen::VectorXd null_direction;
      if (!nullSpaceDirection(walker, null_direction)) {
        break;
      }
      std::vector<double> nudged(walker.size());
      for (std::size_t i = 0; i < walker.size(); ++i) {
        nudged[i] = walker[i] + direction * options_.null_step * null_direction(i);
      }
      std::vector<double> snapped;
      if (!solveIk(entry_pose, nudged, snapped)) {
        break;
      }
      // If IK snapped straight back to where we started, the manifold is not
      // being explored any more and further steps are wasted.
      if (weightedDistance(snapped, walker) < 1e-3) {
        break;
      }
      parameter += direction * options_.null_step;
      walker = snapped;
      add_unique(walker, parameter);
    }
  }

  // Step 3: score every distinct configuration against the whole shape.
  candidates.reserve(configs.size());
  for (std::size_t i = 0; i < configs.size(); ++i) {
    ConfigCandidate candidate;
    candidate.joints = configs[i];
    candidate.manifold_parameter = parameters[i];
    candidate.spin = spin;
    scoreCandidate(current_joints, waypoints, candidate);
    candidates.push_back(candidate);
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const ConfigCandidate & a, const ConfigCandidate & b) {
      if (a.feasible != b.feasible) {
        return a.feasible;
      }
      return a.total_cost < b.total_cost;
    });
  return candidates;
}

}  // namespace avatar_challenge

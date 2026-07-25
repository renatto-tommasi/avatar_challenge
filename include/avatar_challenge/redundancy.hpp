// Copyright 2026 Renatto Tommasi
//
// Redundancy resolution for a 7-DoF arm tracing a fully constrained 6-DoF path.
//
// For a 7-DoF arm and a 6-DoF task the inverse kinematics has, generically, a
// one-parameter family of solutions: the self-motion manifold. Every point on
// it puts the tool in exactly the same place with the elbow swung somewhere
// else. Which one you pick decides whether the whole shape stays inside the
// joint limits, how far the arm has to travel to get there from wherever it is
// now, and how close it drifts to a singularity halfway through the trace.
//
// A plain "solve IK at the first point and go" throws that choice away — it
// takes whatever the numeric solver happens to converge to from the current
// seed. This class instead:
//
//   1. lands on the manifold once, seeded from the robot's current state;
//   2. walks along it in both directions by integrating the Jacobian's null
//      space, re-snapping onto the exact entry pose after every step, to
//      enumerate genuinely distinct configurations for the same entry pose;
//   3. dry-runs the *entire* shape from each candidate, IK-chaining waypoint to
//      waypoint the way the Cartesian planner will;
//   4. scores each candidate on reachability, joint travel while drawing, how
//      far the arm must move to get there, joint-limit headroom and worst-case
//      manipulability, and returns them cheapest-first.
//
// The winner is then handed to the planner as a *joint-space* goal, so what
// gets executed is the configuration that was actually evaluated rather than
// whatever IK finds at execution time.

#ifndef AVATAR_CHALLENGE__REDUNDANCY_HPP_
#define AVATAR_CHALLENGE__REDUNDANCY_HPP_

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

namespace avatar_challenge
{

struct RedundancyOptions
{
  /// How many self-motion samples to take in each direction along the manifold.
  int samples_per_direction{10};

  /// Null-space integration step, radians of joint motion per sample.
  double null_step{0.20};

  /// Two candidates closer than this (weighted joint distance) are duplicates.
  double dedup_distance{0.12};

  /// Per-attempt IK timeout, seconds.
  double ik_timeout{0.01};

  /// Joints closer than this to a limit are treated as unusable.
  double limit_margin{0.05};

  /// Stride through the trace waypoints when dry-running a candidate. 1 checks
  /// every waypoint; larger values trade fidelity for search speed on shapes
  /// with thousands of samples.
  int evaluation_stride{4};

  /// Reject a candidate that cannot reach at least this fraction of the trace.
  double min_reach_fraction{0.999};

  /// Relative weight of each cost term.
  double w_approach{1.0};   ///< distance from the robot's current configuration
  double w_path{2.0};       ///< joint travel accumulated while drawing
  double w_limits{1.5};     ///< proximity to joint limits along the trace
  double w_manip{1.0};      ///< inverse of the worst manipulability on the trace

  /// Per-joint cost weights. Moving joint 1 sweeps the whole arm through space,
  /// moving joint 7 spins the flange, so they should not cost the same.
  std::vector<double> joint_weights{2.0, 1.8, 1.4, 1.2, 0.8, 0.6, 0.4};
};

/// One evaluated configuration for a whole shape.
struct ConfigCandidate
{
  /// Joint values at the trace's entry pose.
  std::vector<double> joints;

  /// Where along the self-motion manifold this came from, in accumulated
  /// null-space radians relative to the direct IK solution (signed).
  double manifold_parameter{0.0};

  /// Free tool spin about the plane normal this candidate was built with.
  double spin{0.0};

  double reach_fraction{0.0};
  double approach_cost{0.0};
  double path_cost{0.0};
  double limit_cost{0.0};
  double manip_cost{0.0};
  double min_manipulability{0.0};
  double total_cost{0.0};
  bool feasible{false};
};

class RedundancyResolver
{
public:
  RedundancyResolver(
    moveit::core::RobotModelConstPtr robot_model, const std::string & group_name,
    const std::string & tip_link, RedundancyOptions options);

  /// Enumerate and score configurations for tracing `waypoints`, entered at
  /// `entry_pose`, starting from `current_joints`. Returns candidates sorted by
  /// increasing total cost; feasible ones always precede infeasible ones.
  /// `spin` is recorded on each candidate but not applied — the caller bakes it
  /// into the poses it passes in.
  std::vector<ConfigCandidate> evaluate(
    const std::vector<double> & current_joints, const Eigen::Isometry3d & entry_pose,
    const std::vector<Eigen::Isometry3d> & waypoints, double spin);

  /// Weighted joint distance used by the cost function and by shape ordering.
  double weightedDistance(const std::vector<double> & a, const std::vector<double> & b) const;

  const moveit::core::JointModelGroup * group() const {return group_;}

private:
  /// Snap `seed` onto `pose` with the group's IK solver.
  bool solveIk(
    const Eigen::Isometry3d & pose, const std::vector<double> & seed,
    std::vector<double> & solution) const;

  /// Unit vector spanning the Jacobian's null space at `joints`, or false if
  /// the arm is at a singularity where the null space is not 1-dimensional.
  bool nullSpaceDirection(const std::vector<double> & joints, Eigen::VectorXd & direction) const;

  /// sqrt(det(J J^T)), i.e. the product of the Jacobian's singular values.
  double manipulability(const std::vector<double> & joints) const;

  /// Sum over joints of the squared normalised offset from the middle of the
  /// joint's range: 0 at mid-range, 1 at a limit.
  double limitPenalty(const std::vector<double> & joints) const;

  /// True if the configuration is within limits (by `limit_margin`) and free of
  /// self-collision.
  bool isUsable(const std::vector<double> & joints) const;

  /// Dry-run the trace from `start` and fill in the candidate's cost terms.
  void scoreCandidate(
    const std::vector<double> & current_joints,
    const std::vector<Eigen::Isometry3d> & waypoints, ConfigCandidate & candidate) const;

  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup * group_{nullptr};
  std::string tip_link_;
  RedundancyOptions options_;
  planning_scene::PlanningScenePtr scene_;

  /// Cached per-joint bounds for the group, in group variable order.
  std::vector<double> lower_;
  std::vector<double> upper_;

  /// Scratch state, reused to keep the search allocation-free in the hot loop.
  mutable moveit::core::RobotStatePtr scratch_;
};

}  // namespace avatar_challenge

#endif  // AVATAR_CHALLENGE__REDUNDANCY_HPP_

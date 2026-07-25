// Copyright 2026 Renatto Tommasi
//
// shape_tracer: read 2D shapes from YAML, trace them in the air with an xArm 7.
//
// Shapes come from two places: the YAML file named by `shapes_file`, read once
// at startup, and the ~/add_shapes topic, which carries the same schema as a
// message so another node can hand over shapes at runtime. Both feed one queue,
// and one batch is drawn to completion before the next starts.
//
// Pipeline per shape
// ------------------
//   YAML / ShapeArray msg
//         ->  ShapeSpec (2D geometry + pose of the shape frame in the robot's
//             base frame)
//         ->  dense 2D outline (lines / arcs / circles / B-splines, with
//             optional corner blending)
//         ->  Cartesian waypoints: p_base = T_base_shape * (x, y, standoff),
//             every one carrying the same orientation, which puts the tool's
//             approach axis normal to the shape plane
//         ->  redundancy search over the 7th DoF for the configuration that
//             traces the whole outline most cheaply from where the arm is now
//         ->  joint-space transit to that configuration, Cartesian trace,
//             time-optimal retiming, execution.

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "avatar_challenge/markers.hpp"
#include "avatar_challenge/msg/shape_array.hpp"
#include "avatar_challenge/path_sampler.hpp"
#include "avatar_challenge/redundancy.hpp"
#include "avatar_challenge/shape_msg_conversion.hpp"
#include "avatar_challenge/yaml_loader.hpp"

namespace avatar_challenge
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

std::string joinJoints(const std::vector<double> & q)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(3) << "[";
  for (std::size_t i = 0; i < q.size(); ++i) {
    os << (i ? ", " : "") << q[i];
  }
  os << "]";
  return os.str();
}

}  // namespace

class ShapeTracer
{
public:
  explicit ShapeTracer(const rclcpp::Node::SharedPtr & node)
  : node_(node), logger_(node->get_logger())
  {
    declareParameters();

    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/shape_markers", rclcpp::QoS(1).transient_local());
    traced_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/traced_path", rclcpp::QoS(1).transient_local());
    static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  }

  /// Trace the shapes from the YAML file, then keep tracing whatever arrives on
  /// ~/add_shapes until shutdown. Returns false if anything failed hard.
  bool run()
  {
    defaults_.reference_frame = base_frame_;
    defaults_.velocity_scaling = node_->get_parameter("velocity_scaling").as_double();
    defaults_.acceleration_scaling = node_->get_parameter("acceleration_scaling").as_double();
    defaults_.approach_distance = node_->get_parameter("approach_distance").as_double();

    // loadShapeProgram folds the file's `defaults:` block into defaults_, so this
    // has to happen before anything else reads it.
    ShapeProgram initial = loadInitialProgram();

    initPlanning();

    // Only subscribe once defaults_ has settled: shapes arriving on the topic
    // then inherit exactly what a YAML shape with the same keys missing would.
    // Transient-local means a producer that published before this node came up
    // is not lost.
    shapes_sub_ = node_->create_subscription<msg::ShapeArray>(
      "~/add_shapes", rclcpp::QoS(10).reliable().transient_local(),
      [this](const msg::ShapeArray::SharedPtr msg) {onShapeArray(msg);});
    RCLCPP_INFO(
      logger_, "Listening for more shapes on '%s'",
      shapes_sub_->get_topic_name());

    enqueue(std::move(initial), false);
    return consume();
  }

  /// Pop programs off the queue and trace them, one batch at a time. A batch
  /// that arrives mid-trace waits its turn rather than preempting, so the arm is
  /// never left with a half-drawn outline.
  bool consume()
  {
    const bool exit_when_done = node_->get_parameter("exit_when_done").as_bool();
    const bool stop_on_failure = node_->get_parameter("stop_on_failure").as_bool();
    bool all_ok = true;

    while (rclcpp::ok()) {
      PendingProgram pending;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        // Timed wait rather than a plain one: rclcpp::ok() flipping on Ctrl-C
        // does not notify the condition variable, and a missed wake-up here
        // would hang the process on shutdown.
        queue_cv_.wait_for(
          lock, std::chrono::milliseconds(200), [this] {return !queue_.empty();});
        if (queue_.empty()) {
          continue;
        }
        pending = std::move(queue_.front());
        queue_.pop_front();
      }

      if (pending.replace) {
        clearVisuals();
      }
      if (!pending.program.shapes.empty()) {
        if (!runProgram(std::move(pending.program)) ) {
          all_ok = false;
          if (stop_on_failure) {
            return false;
          }
        }
      }

      bool idle = false;
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        idle = queue_.empty();
      }
      if (idle) {
        park();
        if (!exit_when_done) {
          RCLCPP_INFO(
            logger_, "Idle. Publish on '%s' to draw more, or Ctrl-C to exit.",
            shapes_sub_->get_topic_name());
        }
      }
      if (exit_when_done) {
        // The initial program is all the (test) harness asked for.
        break;
      }
    }
    return all_ok;
  }

  /// Plan and (unless plan_only) execute one batch of shapes.
  bool runProgram(ShapeProgram program)
  {
    // Names become RViz labels and TF child frames, so two shapes called "square"
    // in different batches would fight over one frame. Taken by value so the
    // rename happens on our copy.
    for (ShapeSpec & shape : program.shapes) {
      shape.name = uniqueName(shape.name);
    }

    // Build every trace up front so ordering can reason about all of them.
    std::vector<CartesianTrace> traces;
    traces.reserve(program.shapes.size());
    for (const ShapeSpec & shape : program.shapes) {
      if (shape.reference_frame != base_frame_) {
        RCLCPP_WARN(
          logger_,
          "Shape '%s' declares frame '%s'; this node plans in '%s'. Provide the start pose in "
          "the planning frame, or add a static transform.",
          shape.name.c_str(), shape.reference_frame.c_str(), base_frame_.c_str());
      }
      try {
        traces.push_back(buildTrace(shape, defaults_.approach_distance, 0.0, false));
      } catch (const std::exception & e) {
        // One unbuildable shape must not sink the batch now that a producer can
        // send several at once.
        RCLCPP_ERROR(logger_, "Shape '%s': %s — skipping it", shape.name.c_str(), e.what());
        traces.push_back(CartesianTrace{});
        continue;
      }
      RCLCPP_INFO(
        logger_, "Shape '%s': %zu waypoints, outline length %.1f mm",
        shape.name.c_str(), traces.back().waypoints.size(), traces.back().length * 1000.0);
    }

    // Marker ids have to be unique across every batch this node has drawn, not
    // just within one program, or a later batch overwrites an earlier one in
    // RViz. Each shape reserves 16, matching shapeMarkers().
    std::vector<int> id_bases(program.shapes.size());
    std::vector<int> traced_ids(program.shapes.size());
    for (std::size_t i = 0; i < program.shapes.size(); ++i) {
      id_bases[i] = next_marker_id_;
      next_marker_id_ += 16;
      traced_ids[i] = next_traced_id_++;
    }

    publishShapeVisuals(program.shapes, traces, id_bases);

    const std::vector<std::size_t> order = shapeOrder(program.shapes, traces);
    if (order.size() > 1) {
      std::ostringstream os;
      for (std::size_t i = 0; i < order.size(); ++i) {
        os << (i ? " -> " : "") << program.shapes[order[i]].name;
      }
      RCLCPP_INFO(logger_, "Draw order: %s", os.str().c_str());
    }

    bool all_ok = true;
    for (const std::size_t index : order) {
      if (!rclcpp::ok()) {
        return false;
      }
      if (traces[index].waypoints.empty()) {
        all_ok = false;  // already reported above
        continue;
      }
      if (!traceShape(program.shapes[index], traces[index], traced_ids[index])) {
        RCLCPP_ERROR(logger_, "Shape '%s' failed", program.shapes[index].name.c_str());
        all_ok = false;
        if (node_->get_parameter("stop_on_failure").as_bool()) {
          return false;
        }
      }
    }
    return all_ok;
  }

  /// Move to a named SRDF state once everything is drawn, so the arm ends in a
  /// known posture instead of hovering over the last shape it traced.
  void park()
  {
    const std::string target = node_->get_parameter("park_state").as_string();
    if (target.empty() || node_->get_parameter("plan_only").as_bool() || !rclcpp::ok()) {
      return;
    }
    move_group_->setStartStateToCurrentState();
    if (!move_group_->setNamedTarget(target)) {
      RCLCPP_WARN(
        logger_, "park_state '%s' is not a named state of group '%s'; staying put",
        target.c_str(), group_name_.c_str());
      return;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
      move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger_, "Parked at '%s'", target.c_str());
    } else {
      RCLCPP_WARN(logger_, "Could not park at '%s'", target.c_str());
    }
  }

private:
  /// One queued batch: a program plus what to do with what is already on screen.
  struct PendingProgram
  {
    ShapeProgram program;
    bool replace{false};
  };

  /// Read the shapes file named by the `shapes_file` parameter. Neither an
  /// absent file nor a broken one is fatal any more: shapes can also arrive on
  /// ~/add_shapes, so the node comes up and waits instead of exiting.
  ShapeProgram loadInitialProgram()
  {
    if (shapes_file_.empty()) {
      RCLCPP_INFO(logger_, "No shapes_file given; starting with nothing to draw");
      return ShapeProgram{};
    }
    try {
      ShapeProgram program = loadShapeProgram(shapes_file_, defaults_);
      RCLCPP_INFO(
        logger_, "Loaded %zu shape(s) from %s", program.shapes.size(), shapes_file_.c_str());
      return program;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "Failed to load shapes from '%s': %s — continuing without them",
        shapes_file_.c_str(), e.what());
      return ShapeProgram{};
    }
  }

  /// Build the MoveIt handles and the redundancy resolver. Done once, not per
  /// batch: MoveGroupInterface brings up action clients and a state monitor.
  void initPlanning()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_, group_name_);
    move_group_->setEndEffectorLink(tip_link_);
    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setPlannerId(node_->get_parameter("planner_id").as_string());
    move_group_->setPlanningTime(node_->get_parameter("planning_time").as_double());
    move_group_->setNumPlanningAttempts(
      static_cast<unsigned>(node_->get_parameter("planning_attempts").as_int()));
    move_group_->setMaxVelocityScalingFactor(defaults_.velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(defaults_.acceleration_scaling);
    move_group_->startStateMonitor(5.0);

    robot_model_ = move_group_->getRobotModel();
    resolver_ = std::make_shared<RedundancyResolver>(
      robot_model_, group_name_, tip_link_, redundancyOptions());
  }

  /// Subscription callback. Runs on the executor thread while tracing blocks the
  /// main one, so it only converts and queues — never plans.
  void onShapeArray(const msg::ShapeArray::SharedPtr msg)
  {
    std::vector<std::string> errors;
    ShapeProgram program = fromMsg(*msg, defaults_, errors);
    for (const std::string & error : errors) {
      RCLCPP_ERROR(logger_, "Rejected a shape from ~/add_shapes: %s", error.c_str());
    }

    const bool replace = msg->mode == msg::ShapeArray::REPLACE;
    if (program.shapes.empty() && !replace) {
      RCLCPP_WARN(logger_, "ShapeArray had no usable shapes; nothing queued");
      return;
    }
    RCLCPP_INFO(
      logger_, "Queued %zu shape(s) from ~/add_shapes (%s)", program.shapes.size(),
      replace ? "replace" : "append");
    enqueue(std::move(program), replace);
  }

  void enqueue(ShapeProgram program, bool replace)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_.push_back(PendingProgram{std::move(program), replace});
    }
    queue_cv_.notify_one();
  }

  /// Shape names identify TF frames and RViz labels, so they have to stay unique
  /// across every batch, not just within one.
  std::string uniqueName(const std::string & requested)
  {
    std::string name = requested.empty() ? "shape" : requested;
    if (used_names_.insert(name).second) {
      return name;
    }
    for (int suffix = 2; ; ++suffix) {
      const std::string candidate = name + "_" + std::to_string(suffix);
      if (used_names_.insert(candidate).second) {
        RCLCPP_WARN(
          logger_, "Shape name '%s' is already in use; drawing it as '%s'",
          name.c_str(), candidate.c_str());
        return candidate;
      }
    }
  }

  /// Wipe every marker this node has published and start ids over. Used by
  /// ShapeArray::REPLACE. The static TF frames published per shape cannot be
  /// retracted — tf2 has no "forget this static transform" — so stale
  /// shape_<name> frames linger until the node restarts.
  void clearVisuals()
  {
    visualization_msgs::msg::MarkerArray clear;
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    clear.markers.push_back(marker);
    marker_pub_->publish(clear);
    traced_pub_->publish(clear);

    next_marker_id_ = 0;
    next_traced_id_ = 0;
    used_names_.clear();
    RCLCPP_INFO(logger_, "Cleared previously drawn shapes");
  }

  void declareParameters()
  {
    shapes_file_ = node_->declare_parameter<std::string>("shapes_file", "");
    group_name_ = node_->declare_parameter<std::string>("group", "xarm7");
    tip_link_ = node_->declare_parameter<std::string>("tip_link", "link_eef");
    base_frame_ = node_->declare_parameter<std::string>("base_frame", "link_base");

    node_->declare_parameter<std::string>("planner_id", "RRTConnect");
    node_->declare_parameter<double>("planning_time", 5.0);
    node_->declare_parameter<int>("planning_attempts", 10);
    node_->declare_parameter<double>("velocity_scaling", 0.3);
    node_->declare_parameter<double>("acceleration_scaling", 0.3);
    node_->declare_parameter<double>("approach_distance", 0.05);

    node_->declare_parameter<double>("cartesian_step", 0.004);
    node_->declare_parameter<double>("cartesian_jump_threshold", 4.0);
    node_->declare_parameter<double>("min_cartesian_fraction", 0.995);
    node_->declare_parameter<double>("totg_path_tolerance", 0.05);
    node_->declare_parameter<int>("max_config_attempts", 4);

    node_->declare_parameter<bool>("plan_only", false);
    node_->declare_parameter<bool>("stop_on_failure", false);
    node_->declare_parameter<bool>("optimize_shape_order", true);
    node_->declare_parameter<bool>("exit_when_done", false);
    node_->declare_parameter<std::string>("park_state", "");

    node_->declare_parameter<int>("redundancy.samples_per_direction", 10);
    node_->declare_parameter<double>("redundancy.null_step", 0.20);
    node_->declare_parameter<double>("redundancy.dedup_distance", 0.12);
    node_->declare_parameter<double>("redundancy.ik_timeout", 0.01);
    node_->declare_parameter<double>("redundancy.limit_margin", 0.05);
    node_->declare_parameter<int>("redundancy.evaluation_stride", 4);
    node_->declare_parameter<double>("redundancy.w_approach", 1.0);
    node_->declare_parameter<double>("redundancy.w_path", 2.0);
    node_->declare_parameter<double>("redundancy.w_limits", 1.5);
    node_->declare_parameter<double>("redundancy.w_manip", 1.0);
    node_->declare_parameter<std::vector<double>>(
      "redundancy.joint_weights", {2.0, 1.8, 1.4, 1.2, 0.8, 0.6, 0.4});
  }

  RedundancyOptions redundancyOptions() const
  {
    RedundancyOptions options;
    options.samples_per_direction =
      static_cast<int>(node_->get_parameter("redundancy.samples_per_direction").as_int());
    options.null_step = node_->get_parameter("redundancy.null_step").as_double();
    options.dedup_distance = node_->get_parameter("redundancy.dedup_distance").as_double();
    options.ik_timeout = node_->get_parameter("redundancy.ik_timeout").as_double();
    options.limit_margin = node_->get_parameter("redundancy.limit_margin").as_double();
    options.evaluation_stride =
      static_cast<int>(node_->get_parameter("redundancy.evaluation_stride").as_int());
    options.w_approach = node_->get_parameter("redundancy.w_approach").as_double();
    options.w_path = node_->get_parameter("redundancy.w_path").as_double();
    options.w_limits = node_->get_parameter("redundancy.w_limits").as_double();
    options.w_manip = node_->get_parameter("redundancy.w_manip").as_double();
    options.joint_weights = node_->get_parameter("redundancy.joint_weights").as_double_array();
    return options;
  }

  /// Where the arm is, as far as planning is concerned. In plan_only mode
  /// nothing actually moves, so the joint state would stay at the start pose
  /// and every shape after the first would be costed from the wrong place;
  /// the virtual cursor keeps the preview honest.
  std::vector<double> currentJoints() const
  {
    if (!virtual_joints_.empty()) {
      return virtual_joints_;
    }
    moveit::core::RobotStatePtr state = move_group_->getCurrentState(5.0);
    std::vector<double> joints;
    if (state) {
      state->copyJointGroupPositions(resolver_->group(), joints);
    }
    return joints;
  }

  /// Greedy nearest-entry ordering. At each step the arm is somewhere specific,
  /// so "closest" is measured as weighted joint distance from the current
  /// configuration to the cheapest IK solution at each remaining shape's entry
  /// pose — not as Cartesian distance, which would ignore how the arm has to
  /// reconfigure to get there.
  std::vector<std::size_t> shapeOrder(
    const std::vector<ShapeSpec> & shapes, const std::vector<CartesianTrace> & traces)
  {
    std::vector<std::size_t> order;
    const std::size_t n = shapes.size();
    if (!node_->get_parameter("optimize_shape_order").as_bool() || n < 3) {
      order.resize(n);
      for (std::size_t i = 0; i < n; ++i) {
        order[i] = i;
      }
      return order;
    }

    std::vector<bool> taken(n, false);
    std::vector<double> cursor = currentJoints();

    for (std::size_t step = 0; step < n; ++step) {
      std::size_t best = n;
      double best_cost = std::numeric_limits<double>::infinity();
      std::vector<double> best_exit;

      for (std::size_t i = 0; i < n; ++i) {
        if (taken[i] || traces[i].waypoints.empty()) {
          continue;  // shapes that failed to build carry an empty trace
        }
        // Cheap probe: one IK solve at the entry pose seeded from where the arm
        // currently is. The expensive full evaluation happens once the shape is
        // actually committed to.
        std::vector<Eigen::Isometry3d> probe = {traces[i].approach_pose, traces[i].retreat_pose};
        const auto candidates = resolver_->evaluate(cursor, traces[i].approach_pose, probe, 0.0);
        if (candidates.empty()) {
          continue;
        }
        const double cost = candidates.front().total_cost;
        if (cost < best_cost) {
          best_cost = cost;
          best = i;
          best_exit = candidates.front().joints;
        }
      }

      if (best == n) {
        // Nothing reachable was found; fall back to input order for the rest.
        for (std::size_t i = 0; i < n; ++i) {
          if (!taken[i]) {
            order.push_back(i);
            taken[i] = true;
          }
        }
        break;
      }
      order.push_back(best);
      taken[best] = true;
      cursor = best_exit;
    }
    return order;
  }

  void publishShapeVisuals(
    const std::vector<ShapeSpec> & shapes, const std::vector<CartesianTrace> & traces,
    const std::vector<int> & id_bases)
  {
    visualization_msgs::msg::MarkerArray array;
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    const rclcpp::Time stamp = node_->now();

    for (std::size_t i = 0; i < shapes.size(); ++i) {
      const auto markers = shapeMarkers(shapes[i], traces[i], base_frame_, id_bases[i], stamp);
      array.markers.insert(array.markers.end(), markers.markers.begin(), markers.markers.end());

      // Publishing the shape frame on TF makes the "author in 2D, execute in 3D"
      // mapping directly inspectable: select the frame in RViz and the shape's
      // own axes are right there on the plane.
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = base_frame_;
      tf.child_frame_id = "shape_" + shapes[i].name;
      tf.transform = tf2::eigenToTransform(shapes[i].start_pose).transform;
      transforms.push_back(tf);
    }
    marker_pub_->publish(array);
    static_tf_->sendTransform(transforms);
  }

  /// Retime a Cartesian result with TOTG. `computeCartesianPath` interpolates in
  /// configuration space and leaves a trajectory that is geometrically right but
  /// dynamically meaningless; TOTG puts a time law on it that respects the
  /// velocity and acceleration limits and rounds corners in joint space, which
  /// is what stops the arm dead-halting at every vertex.
  bool retime(
    moveit_msgs::msg::RobotTrajectory & trajectory, const moveit::core::RobotState & start,
    double velocity_scaling, double acceleration_scaling)
  {
    robot_trajectory::RobotTrajectory rt(robot_model_, group_name_);
    rt.setRobotTrajectoryMsg(start, trajectory);
    if (rt.getWayPointCount() < 2) {
      return false;
    }

    trajectory_processing::TimeOptimalTrajectoryGeneration totg(
      node_->get_parameter("totg_path_tolerance").as_double(), 0.01);
    if (!totg.computeTimeStamps(rt, velocity_scaling, acceleration_scaling)) {
      RCLCPP_WARN(logger_, "Time-optimal retiming failed; keeping the planner's timing");
      return false;
    }
    rt.getRobotTrajectoryMsg(trajectory);
    return true;
  }

  /// Sample the tool position along an executed trajectory, for the trace marker.
  void appendTracedPoints(
    const moveit_msgs::msg::RobotTrajectory & trajectory, const moveit::core::RobotState & start,
    std::vector<Eigen::Vector3d> & out) const
  {
    robot_trajectory::RobotTrajectory rt(robot_model_, group_name_);
    rt.setRobotTrajectoryMsg(start, trajectory);
    for (std::size_t i = 0; i < rt.getWayPointCount(); ++i) {
      moveit::core::RobotState state = rt.getWayPoint(i);
      state.update();
      out.push_back(state.getGlobalLinkTransform(tip_link_).translation());
    }
  }

  bool traceShape(const ShapeSpec & shape, const CartesianTrace & trace, int traced_marker_id)
  {
    const double velocity_scaling =
      shape.velocity_scaling > 0 ? shape.velocity_scaling : defaults_.velocity_scaling;
    const double acceleration_scaling =
      shape.acceleration_scaling > 0 ? shape.acceleration_scaling : defaults_.acceleration_scaling;

    // The full pose sequence the arm must hold a single configuration branch
    // through: stand off above the entry, draw, stand off again.
    std::vector<Eigen::Isometry3d> full_path;
    full_path.reserve(trace.waypoints.size() + 2);
    full_path.push_back(trace.approach_pose);
    full_path.insert(full_path.end(), trace.waypoints.begin(), trace.waypoints.end());
    full_path.push_back(trace.retreat_pose);

    const std::vector<double> start_joints = currentJoints();
    if (start_joints.empty()) {
      RCLCPP_ERROR(logger_, "Could not read the current joint state");
      return false;
    }

    // --- 7th-DoF search --------------------------------------------------
    // With free_spin the rotation of the tool about the plane normal is also
    // unconstrained by the task, so it becomes a second search dimension: for
    // each spin sample the whole self-motion manifold is re-enumerated and the
    // best configuration across all of them wins.
    std::vector<ConfigCandidate> candidates;
    std::vector<CartesianTrace> spin_traces;   // one entry per spin sample
    std::vector<std::size_t> candidate_trace;  // candidate -> spin_traces index

    const int spin_samples = shape.tool.free_spin ? std::max(1, shape.tool.spin_samples) : 1;
    for (int s = 0; s < spin_samples; ++s) {
      double spin = shape.tool.spin;
      std::vector<Eigen::Isometry3d> spin_path = full_path;
      if (shape.tool.free_spin) {
        spin = shape.tool.spin + 2.0 * kPi * static_cast<double>(s) / spin_samples;
        spin_traces.push_back(buildTrace(shape, defaults_.approach_distance, spin, true));
        const CartesianTrace & st = spin_traces.back();
        spin_path.clear();
        spin_path.reserve(st.waypoints.size() + 2);
        spin_path.push_back(st.approach_pose);
        spin_path.insert(spin_path.end(), st.waypoints.begin(), st.waypoints.end());
        spin_path.push_back(st.retreat_pose);
      } else {
        spin_traces.push_back(trace);
      }

      auto found = resolver_->evaluate(start_joints, spin_path.front(), spin_path, spin);
      for (ConfigCandidate & candidate : found) {
        candidate.spin = spin;
        candidate_trace.push_back(spin_traces.size() - 1);
        candidates.push_back(candidate);
      }
    }

    if (candidates.empty()) {
      RCLCPP_ERROR(
        logger_, "Shape '%s': no IK solution at the entry pose — is it inside the workspace?",
        shape.name.c_str());
      return false;
    }

    // Re-sort across spins, keeping traces aligned with their candidates.
    std::vector<std::size_t> ranking(candidates.size());
    for (std::size_t i = 0; i < ranking.size(); ++i) {
      ranking[i] = i;
    }
    std::stable_sort(
      ranking.begin(), ranking.end(), [&](std::size_t a, std::size_t b) {
        if (candidates[a].feasible != candidates[b].feasible) {
          return candidates[a].feasible;
        }
        return candidates[a].total_cost < candidates[b].total_cost;
      });

    logCandidates(shape.name, candidates, ranking);

    // --- try the ranked configurations in order --------------------------
    const int max_attempts =
      static_cast<int>(node_->get_parameter("max_config_attempts").as_int());
    const bool plan_only = node_->get_parameter("plan_only").as_bool();

    for (int attempt = 0; attempt < max_attempts && attempt < static_cast<int>(ranking.size());
      ++attempt)
    {
      const ConfigCandidate & candidate = candidates[ranking[attempt]];
      const CartesianTrace & chosen = spin_traces[candidate_trace[ranking[attempt]]];
      if (!candidate.feasible && attempt > 0) {
        break;  // ran out of configurations that can trace the whole outline
      }

      RCLCPP_INFO(
        logger_,
        "Shape '%s': attempt %d using config %s (manifold %+.2f rad, spin %+.2f rad, cost %.3f)",
        shape.name.c_str(), attempt + 1, joinJoints(candidate.joints).c_str(),
        candidate.manifold_parameter, candidate.spin, candidate.total_cost);

      // 1. Transit to the chosen configuration as a *joint-space* goal, so the
      //    arm arrives in the posture that was actually evaluated.
      move_group_->setStartStateToCurrentState();
      move_group_->setMaxVelocityScalingFactor(velocity_scaling);
      move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
      move_group_->setJointValueTarget(candidate.joints);

      moveit::planning_interface::MoveGroupInterface::Plan transit;
      if (move_group_->plan(transit) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(logger_, "Shape '%s': transit plan failed", shape.name.c_str());
        continue;
      }
      if (!plan_only && move_group_->execute(transit) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(logger_, "Shape '%s': transit execution failed", shape.name.c_str());
        continue;
      }

      // 2. Cartesian trace from there: descend to the first point, draw the
      //    outline, retreat. Held on one IK branch by the jump threshold.
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.reserve(chosen.waypoints.size() + 1);
      for (const Eigen::Isometry3d & pose : chosen.waypoints) {
        waypoints.push_back(tf2::toMsg(pose));
      }
      waypoints.push_back(tf2::toMsg(chosen.retreat_pose));

      moveit::core::RobotStatePtr cartesian_start = move_group_->getCurrentState(5.0);
      if (!cartesian_start) {
        RCLCPP_WARN(logger_, "Shape '%s': lost the joint state before tracing", shape.name.c_str());
        continue;
      }
      if (plan_only) {
        // Nothing was executed, so anchor the Cartesian path at the planned
        // transit endpoint rather than at the (unmoved) real robot.
        moveit::core::robotStateMsgToRobotState(transit.start_state_, *cartesian_start);
        cartesian_start->setJointGroupPositions(resolver_->group(), candidate.joints);
        cartesian_start->update();
      }
      move_group_->setStartState(*cartesian_start);

      moveit_msgs::msg::RobotTrajectory traced;
      const double fraction = move_group_->computeCartesianPath(
        waypoints, node_->get_parameter("cartesian_step").as_double(),
        node_->get_parameter("cartesian_jump_threshold").as_double(), traced, true);

      const double min_fraction = node_->get_parameter("min_cartesian_fraction").as_double();
      if (fraction < min_fraction) {
        RCLCPP_WARN(
          logger_, "Shape '%s': Cartesian path only %.1f%% complete, trying the next configuration",
          shape.name.c_str(), fraction * 100.0);
        continue;
      }

      // 3. Give the geometric path a dynamically feasible time law.
      retime(traced, *cartesian_start, velocity_scaling, acceleration_scaling);

      const double duration = traced.joint_trajectory.points.empty() ?
        0.0 :
        rclcpp::Duration(traced.joint_trajectory.points.back().time_from_start).seconds();
      RCLCPP_INFO(
        logger_, "Shape '%s': Cartesian path %.1f%% over %zu points, %.2f s",
        shape.name.c_str(), fraction * 100.0, traced.joint_trajectory.points.size(), duration);

      if (!plan_only) {
        if (move_group_->execute(traced) != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(logger_, "Shape '%s': trace execution failed", shape.name.c_str());
          continue;
        }
      }

      if (plan_only && !traced.joint_trajectory.points.empty()) {
        virtual_joints_ = traced.joint_trajectory.points.back().positions;
      }

      std::vector<Eigen::Vector3d> points;
      appendTracedPoints(traced, *cartesian_start, points);
      publishTracedPath(points, traced_marker_id);
      return true;
    }

    return false;
  }

  void logCandidates(
    const std::string & name, const std::vector<ConfigCandidate> & candidates,
    const std::vector<std::size_t> & ranking) const
  {
    std::ostringstream os;
    os << "Shape '" << name << "': evaluated " << candidates.size()
       << " configuration(s) on the self-motion manifold\n";
    os << std::fixed << std::setprecision(3);
    os << "      rank  psi[rad]  spin  reach  approach   path  limits  manip  "
       << "min|J|   total\n";
    const std::size_t shown = std::min<std::size_t>(ranking.size(), 8);
    for (std::size_t i = 0; i < shown; ++i) {
      const ConfigCandidate & c = candidates[ranking[i]];
      os << "      " << std::setw(4) << i << "  " << std::setw(8) << c.manifold_parameter << "  "
         << std::setw(4) << c.spin << "  " << std::setw(5) << c.reach_fraction << "  "
         << std::setw(8) << c.approach_cost << "  " << std::setw(5) << c.path_cost << "  "
         << std::setw(6) << c.limit_cost << "  " << std::setw(5) << c.manip_cost << "  "
         << std::setw(6) << c.min_manipulability << "  " << std::setw(6) << c.total_cost
         << (c.feasible ? "" : "  (incomplete)") << "\n";
    }
    RCLCPP_INFO(logger_, "%s", os.str().c_str());
  }

  void publishTracedPath(const std::vector<Eigen::Vector3d> & points, int traced_marker_id)
  {
    // "traced_path" is its own marker namespace (see markers.cpp), so this
    // counter is free to run independently of the shape-marker ids.
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(
      tracedPathMarker(points, base_frame_, traced_marker_id, node_->now()));
    traced_pub_->publish(array);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::string shapes_file_;
  std::string group_name_;
  std::string tip_link_;
  std::string base_frame_;

  moveit::core::RobotModelConstPtr robot_model_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<RedundancyResolver> resolver_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traced_pub_;
  rclcpp::Subscription<msg::ShapeArray>::SharedPtr shapes_sub_;

  /// Seeded from parameters, then folded with the shapes file's `defaults:`
  /// block. Written only during run() before the subscription exists, and
  /// read-only afterwards, which is what makes it safe to touch from the
  /// subscription callback.
  ProgramDefaults defaults_;

  /// Batches waiting to be drawn, newest last.
  std::deque<PendingProgram> queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  /// Marker ids and shape names accumulate across batches; see runProgram().
  int next_marker_id_{0};
  int next_traced_id_{0};
  std::set<std::string> used_names_;

  /// Non-empty only in plan_only mode; see currentJoints().
  std::vector<double> virtual_joints_;
};

}  // namespace avatar_challenge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(false);
  auto node = std::make_shared<rclcpp::Node>("shape_tracer", options);

  // MoveGroupInterface needs a spinning executor for its state monitor and
  // action clients while the (blocking) tracing work runs on this thread.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  int exit_code = 0;
  try {
    avatar_challenge::ShapeTracer tracer(node);
    // run() draws the shapes file and then serves ~/add_shapes until shutdown,
    // so it only returns on Ctrl-C or when exit_when_done cuts it short.
    if (!tracer.run()) {
      exit_code = 1;
    } else {
      RCLCPP_INFO(node->get_logger(), "All shapes traced.");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "shape_tracer failed: %s", e.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  spinner.join();
  return exit_code;
}

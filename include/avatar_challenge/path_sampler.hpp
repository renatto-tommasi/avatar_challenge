// Copyright 2026 Renatto Tommasi
//
// Turns a ShapeSpec's primitive list into a dense 2D polyline in the shape
// frame, then lifts that polyline into full 6-DoF Cartesian waypoints.

#ifndef AVATAR_CHALLENGE__PATH_SAMPLER_HPP_
#define AVATAR_CHALLENGE__PATH_SAMPLER_HPP_

#include <Eigen/Geometry>

#include <string>
#include <vector>

#include "avatar_challenge/shape_spec.hpp"

namespace avatar_challenge
{

/// A densely sampled outline in shape-frame 2D, plus the indices of the samples
/// that came from an authored corner (kept so markers can show them).
struct SampledOutline
{
  std::vector<Point2d> points;
  std::vector<std::size_t> corner_indices;
};

/// Sample one primitive, appending to `out` and *not* repeating `start`.
/// Exposed for unit testing.
///
/// `start` is taken by value on purpose: callers pass `out.back()`, and
/// appending to `out` can reallocate it out from under a reference.
void sampleSegment(
  const Segment & segment, Point2d start, const SamplingSpec & sampling,
  std::vector<Point2d> & out);

/// Round every tangent discontinuity in `points` with a quadratic Bezier that
/// starts and ends `blend_distance` of arc length away from the corner. The
/// Bezier is G1-continuous with both neighbours, which is what removes the
/// full stop the trajectory would otherwise take at each corner. Corners whose
/// neighbouring edges are too short to give up `blend_distance` are trimmed to
/// what is available rather than skipped.
std::vector<Point2d> blendCorners(
  const std::vector<Point2d> & points, bool closed, double blend_distance,
  double max_segment_length);

/// Full 2D pipeline: primitives -> dense polyline -> optional corner blending.
/// Throws std::runtime_error if the shape has no traceable geometry.
SampledOutline sampleOutline(const ShapeSpec & shape);

/// A traceable shape, expressed in the robot's reference frame.
struct CartesianTrace
{
  /// Pose of the shape frame S in the reference frame.
  Eigen::Isometry3d shape_frame{Eigen::Isometry3d::Identity()};

  /// Constant tool orientation for the whole trace: normal to the shape plane.
  Eigen::Quaterniond tool_orientation{Eigen::Quaterniond::Identity()};

  /// Retreated pose above the first point, used as the approach/entry target.
  Eigen::Isometry3d approach_pose{Eigen::Isometry3d::Identity()};
  /// Retreated pose above the last point.
  Eigen::Isometry3d retreat_pose{Eigen::Isometry3d::Identity()};

  /// The trace itself: one pose per outline sample, all sharing
  /// `tool_orientation`.
  std::vector<Eigen::Isometry3d> waypoints;

  /// The 2D outline the waypoints came from (for markers/debug).
  SampledOutline outline;

  /// Arc length of the outline in metres.
  double length{0.0};
};

/// Lift a sampled outline onto the shape plane and attach the tool orientation.
///
/// Position: p_ref = T_ref_S * (x, y, standoff).
/// Orientation: the tool's approach axis is aligned with `normal_sign * Z_S`,
/// which is exactly the "tool normal to the plane" requirement, and the
/// remaining rotation about that axis is the free `spin` parameter.
CartesianTrace buildTrace(
  const ShapeSpec & shape, double approach_distance, double spin_override,
  bool use_spin_override);

/// Orientation of the tool for a given shape frame, normal sign and spin.
Eigen::Quaterniond toolOrientation(
  const Eigen::Isometry3d & shape_frame, double normal_sign, double spin);

}  // namespace avatar_challenge

#endif  // AVATAR_CHALLENGE__PATH_SAMPLER_HPP_

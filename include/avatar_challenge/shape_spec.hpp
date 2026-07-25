// Copyright 2026 Renatto Tommasi
//
// Data model for the shape description that comes in from YAML.
//
// A shape is authored entirely in 2D, in its own right-handed "shape frame" S.
// The shape frame's origin sits on the shape's first vertex, which the challenge
// fixes at (0, 0); its XY plane *is* the drawing plane, so the plane normal is
// simply Z_S. That single convention is what lets every authored coordinate be
// mapped back into robot coordinates with one rigid transform, and it is what
// makes "tool normal to the plane" a statement about Z_S rather than something
// that has to be recomputed per segment.

#ifndef AVATAR_CHALLENGE__SHAPE_SPEC_HPP_
#define AVATAR_CHALLENGE__SHAPE_SPEC_HPP_

#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace avatar_challenge
{

/// A point in the 2D shape frame (metres, after unit conversion).
using Point2d = Eigen::Vector2d;

/// The primitive kinds a shape's outline can be built from.
enum class SegmentType
{
  kLine,      ///< straight line to `to`
  kArc,       ///< circular arc to `to`, about `center` (or of the given `radius`)
  kCircle,    ///< full circle about `center` — a closed primitive, ignores `to`
  kBspline,   ///< B-spline through/near `control_points`
};

/// One primitive in a shape outline. The start point is always the end point of
/// the previous primitive (or the origin, for the first one), so only the
/// terminating geometry is stored.
struct Segment
{
  SegmentType type{SegmentType::kLine};

  /// End point, in shape-frame 2D. Unused by kCircle and kBspline.
  Point2d to{Point2d::Zero()};

  // ---- arc / circle ----------------------------------------------------
  /// Arc centre. Optional for kArc (may be derived from `radius` instead),
  /// required for kCircle.
  bool has_center{false};
  Point2d center{Point2d::Zero()};

  /// Arc radius. Only consulted when `has_center` is false (or for kCircle).
  double radius{0.0};

  /// Sweep direction: true = counter-clockwise in the shape frame's XY plane.
  bool counter_clockwise{true};

  /// For radius-defined arcs, pick the sweep > pi. Ignored for centre-defined
  /// arcs, where centre + direction already determine the sweep uniquely.
  bool large_arc{false};

  // ---- b-spline --------------------------------------------------------
  std::vector<Point2d> control_points;
  int degree{3};
  /// Periodic (wrapped) B-spline rather than clamped.
  bool periodic{false};
};

/// How the tool is posed relative to the shape plane.
struct ToolSpec
{
  /// +1 => tool approach axis (+Z of the end-effector link) points along +Z_S;
  /// -1 => it points along -Z_S, i.e. "into" the page, like a pen. Default -1.
  double normal_sign{-1.0};

  /// Rotation of the tool about its own approach axis, radians. The task only
  /// constrains the tool to be *normal* to the plane, so this angle is free —
  /// see `free_spin` for letting the solver choose it.
  double spin{0.0};

  /// If true, the spin angle above is treated as a second redundancy parameter
  /// and swept by the configuration search instead of being held fixed.
  bool free_spin{false};

  /// Number of spin samples to try over [0, 2*pi) when `free_spin` is set.
  int spin_samples{8};

  /// Constant offset of the tool along +Z_S from the drawing plane (metres).
  /// Zero means the tool tip rides exactly on the plane.
  double standoff{0.0};
};

/// Discretisation and smoothing knobs for turning primitives into waypoints.
struct SamplingSpec
{
  /// Upper bound on the straight-line distance between consecutive samples (m).
  double max_segment_length{0.004};

  /// Upper bound on how far a sampled chord may deviate from the true curve (m).
  /// Drives the angular step on arcs and the sample count on splines.
  double chord_tolerance{0.0002};

  /// Corner rounding: how far back along each incoming/outgoing edge the corner
  /// is trimmed before a quadratic Bezier is inserted (m). 0 disables blending
  /// and keeps exact, sharp corners.
  double blend_distance{0.0};
};

/// Everything needed to trace one shape.
struct ShapeSpec
{
  std::string name{"shape"};

  /// Frame the start pose is expressed in. Defaults to the robot base link.
  std::string reference_frame{"link_base"};

  /// Pose of the shape frame S in `reference_frame`. Its origin is the shape's
  /// first vertex and its Z axis is the drawing-plane normal.
  Eigen::Isometry3d start_pose{Eigen::Isometry3d::Identity()};

  /// Outline primitives, in draw order.
  std::vector<Segment> segments;

  /// Close the outline by returning to (0, 0) after the last primitive.
  bool closed{true};

  ToolSpec tool;
  SamplingSpec sampling;

  /// Per-shape overrides of the motion profile; negative means "inherit".
  double velocity_scaling{-1.0};
  double acceleration_scaling{-1.0};
};

/// File-level result of parsing the shapes YAML.
struct ShapeProgram
{
  std::vector<ShapeSpec> shapes;
};

}  // namespace avatar_challenge

#endif  // AVATAR_CHALLENGE__SHAPE_SPEC_HPP_

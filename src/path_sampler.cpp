// Copyright 2026 Renatto Tommasi

#include "avatar_challenge/path_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace avatar_challenge
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;

/// Number of steps needed to cover `total` without exceeding `step`.
int stepsFor(double total, double step)
{
  if (step <= kEps) {
    return 1;
  }
  return std::max(1, static_cast<int>(std::ceil(std::abs(total) / step)));
}

/// Wrap an angle into (0, 2*pi]. Used so a full circle stays a full circle
/// instead of collapsing to zero sweep.
double wrapPositive(double angle)
{
  while (angle <= kEps) {
    angle += 2.0 * kPi;
  }
  while (angle > 2.0 * kPi + kEps) {
    angle -= 2.0 * kPi;
  }
  return angle;
}

/// Angular step that keeps both the chord length and the sagitta within budget.
double arcAngularStep(double radius, const SamplingSpec & sampling)
{
  if (radius <= kEps) {
    return kPi;
  }
  // Chord budget: |chord| = 2 r sin(dtheta/2) <= max_segment_length.
  double by_chord = kPi;
  const double s = sampling.max_segment_length / (2.0 * radius);
  if (s < 1.0) {
    by_chord = 2.0 * std::asin(s);
  }
  // Sagitta budget: r (1 - cos(dtheta/2)) <= chord_tolerance.
  double by_sagitta = kPi;
  const double c = 1.0 - sampling.chord_tolerance / radius;
  if (c > -1.0 && c < 1.0) {
    by_sagitta = 2.0 * std::acos(c);
  }
  return std::max(1e-3, std::min(by_chord, by_sagitta));
}

void sampleLine(
  const Point2d & start, const Point2d & end, const SamplingSpec & sampling,
  std::vector<Point2d> & out)
{
  const double length = (end - start).norm();
  const int steps = stepsFor(length, sampling.max_segment_length);
  for (int i = 1; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    out.push_back(start + t * (end - start));
  }
}

/// Resolve an arc's centre. Centre-defined arcs are used verbatim (with a
/// consistency check on the two radii); radius-defined arcs use the standard
/// SVG-style construction, picking between the two candidate centres with the
/// direction and large-arc flags.
Point2d resolveArcCenter(const Segment & seg, const Point2d & start)
{
  if (seg.has_center) {
    return seg.center;
  }

  const Point2d chord = seg.to - start;
  const double d = chord.norm();
  if (d < kEps) {
    throw std::runtime_error("arc: start and end coincide; use type 'circle' for a full circle");
  }
  double r = std::abs(seg.radius);
  if (r < d / 2.0) {
    // Radius too small to span the chord: grow it to the minimum feasible one.
    r = d / 2.0;
  }
  const Point2d mid = 0.5 * (start + seg.to);
  const double h = std::sqrt(std::max(0.0, r * r - 0.25 * d * d));
  // Left-hand normal of the chord.
  const Point2d n(-chord.y() / d, chord.x() / d);

  // For a CCW sweep the centre sits to the left of the chord on a minor arc and
  // to the right on a major arc; CW mirrors that.
  const bool center_left = (seg.counter_clockwise != seg.large_arc);
  return center_left ? Point2d(mid + h * n) : Point2d(mid - h * n);
}

void sampleArcAbout(
  const Point2d & center, const Point2d & start, double sweep, const SamplingSpec & sampling,
  std::vector<Point2d> & out)
{
  const Point2d v0 = start - center;
  const double radius = v0.norm();
  const double theta0 = std::atan2(v0.y(), v0.x());

  const double dtheta = arcAngularStep(radius, sampling);
  const int steps = stepsFor(sweep, dtheta);
  for (int i = 1; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const double a = theta0 + t * sweep;
    out.emplace_back(center.x() + radius * std::cos(a), center.y() + radius * std::sin(a));
  }
}

void sampleArc(
  const Segment & seg, const Point2d & start, const SamplingSpec & sampling,
  std::vector<Point2d> & out)
{
  const Point2d center = resolveArcCenter(seg, start);
  const Point2d v0 = start - center;
  const Point2d v1 = seg.to - center;
  if (v0.norm() < kEps) {
    throw std::runtime_error("arc: start point coincides with the centre");
  }
  if (std::abs(v0.norm() - v1.norm()) > 1e-4) {
    throw std::runtime_error(
            "arc: end point is not on the circle defined by the centre (radius mismatch of " +
            std::to_string(std::abs(v0.norm() - v1.norm())) + " m)");
  }

  const double a0 = std::atan2(v0.y(), v0.x());
  const double a1 = std::atan2(v1.y(), v1.x());
  double sweep = seg.counter_clockwise ? wrapPositive(a1 - a0) : -wrapPositive(a0 - a1);
  sampleArcAbout(center, start, sweep, sampling, out);
}

void sampleCircle(
  const Segment & seg, const Point2d & start, const SamplingSpec & sampling,
  std::vector<Point2d> & out)
{
  if (!seg.has_center) {
    throw std::runtime_error("circle: 'center' is required");
  }
  const double sweep = seg.counter_clockwise ? 2.0 * kPi : -2.0 * kPi;
  sampleArcAbout(seg.center, start, sweep, sampling, out);
}

/// Clamped or periodic knot vector for `n` control points of degree `p`.
std::vector<double> makeKnots(int n, int p, bool periodic)
{
  std::vector<double> knots;
  if (periodic) {
    // Uniform knots; the caller has already wrapped the control points.
    knots.resize(n + p + 1);
    for (int i = 0; i < static_cast<int>(knots.size()); ++i) {
      knots[i] = static_cast<double>(i - p);
    }
  } else {
    knots.assign(p + 1, 0.0);
    const int interior = n - p - 1;
    for (int i = 1; i <= interior; ++i) {
      knots.push_back(static_cast<double>(i) / (interior + 1));
    }
    knots.insert(knots.end(), p + 1, 1.0);
  }
  return knots;
}

/// de Boor evaluation of a B-spline at parameter u.
Point2d deBoor(
  const std::vector<Point2d> & ctrl, const std::vector<double> & knots, int p, double u)
{
  const int n = static_cast<int>(ctrl.size());
  // Find the knot span k with knots[k] <= u < knots[k+1], restricted to the
  // valid domain [knots[p], knots[n]].
  int k = p;
  for (int i = p; i < n; ++i) {
    if (u >= knots[i] && u < knots[i + 1]) {
      k = i;
      break;
    }
    k = n - 1;
  }

  std::vector<Point2d> d(p + 1);
  for (int j = 0; j <= p; ++j) {
    d[j] = ctrl[k - p + j];
  }
  for (int r = 1; r <= p; ++r) {
    for (int j = p; j >= r; --j) {
      const int idx = k - p + j;
      const double den = knots[idx + p - r + 1] - knots[idx];
      const double alpha = (den < kEps) ? 0.0 : (u - knots[idx]) / den;
      d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
    }
  }
  return d[p];
}

void sampleBspline(
  const Segment & seg, const Point2d & start, const SamplingSpec & sampling,
  std::vector<Point2d> & out)
{
  std::vector<Point2d> ctrl = seg.control_points;
  if (ctrl.empty()) {
    throw std::runtime_error("bspline: 'control_points' is required");
  }
  // The spline continues from wherever the previous primitive ended, so the
  // authored control points are treated as relative to nothing — they are
  // absolute shape-frame coordinates — but a clamped spline must start at the
  // current point to keep the outline connected. Prepend it if the author did
  // not already do so.
  if ((ctrl.front() - start).norm() > 1e-6) {
    ctrl.insert(ctrl.begin(), start);
  }

  int p = std::max(1, seg.degree);
  if (seg.periodic) {
    // Wrap the first p control points onto the end to close the curve.
    const int original = static_cast<int>(ctrl.size());
    for (int i = 0; i < p; ++i) {
      ctrl.push_back(ctrl[i % original]);
    }
  }
  p = std::min(p, static_cast<int>(ctrl.size()) - 1);
  if (p < 1) {
    throw std::runtime_error("bspline: need at least 2 control points");
  }

  const std::vector<double> knots = makeKnots(static_cast<int>(ctrl.size()), p, seg.periodic);
  const double u_lo = knots[p];
  const double u_hi = knots[ctrl.size()];

  // Estimate the sample count from the control polygon length: it bounds the
  // curve length from above, so this cannot under-sample.
  double polygon_length = 0.0;
  for (std::size_t i = 1; i < ctrl.size(); ++i) {
    polygon_length += (ctrl[i] - ctrl[i - 1]).norm();
  }
  const int steps = std::max(16, stepsFor(polygon_length, sampling.max_segment_length));

  for (int i = 1; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    // Nudge the last sample inside the domain so the span search stays valid.
    const double u = u_lo + t * (u_hi - u_lo) - (i == steps ? 1e-9 : 0.0);
    out.push_back(deBoor(ctrl, knots, p, u));
  }
}

/// Signed turn angle at `b` between edges (a->b) and (b->c), radians.
double turnAngle(const Point2d & a, const Point2d & b, const Point2d & c)
{
  const Point2d u = b - a;
  const Point2d v = c - b;
  if (u.norm() < kEps || v.norm() < kEps) {
    return 0.0;
  }
  const double cross = u.x() * v.y() - u.y() * v.x();
  const double dot = u.dot(v);
  return std::atan2(cross, dot);
}

/// Walk `distance` of arc length backwards from index `i`, returning the
/// interpolated point and the index of the last sample strictly before it.
Point2d walkBack(
  const std::vector<Point2d> & pts, std::size_t i, double distance, bool closed,
  std::size_t & reached_index, double & consumed)
{
  const std::size_t n = pts.size();
  consumed = 0.0;
  std::size_t cur = i;
  while (consumed < distance) {
    const std::size_t prev = (cur == 0) ? (closed ? n - 1 : 0) : cur - 1;
    if (prev == cur) {
      break;  // start of an open path
    }
    const double seg = (pts[cur] - pts[prev]).norm();
    if (consumed + seg >= distance) {
      const double t = (distance - consumed) / std::max(seg, kEps);
      consumed = distance;
      reached_index = prev;
      return pts[cur] + t * (pts[prev] - pts[cur]);
    }
    consumed += seg;
    cur = prev;
    if (cur == i) {
      break;  // wrapped all the way around
    }
  }
  reached_index = cur;
  return pts[cur];
}

/// Mirror of walkBack, going forwards.
Point2d walkForward(
  const std::vector<Point2d> & pts, std::size_t i, double distance, bool closed,
  std::size_t & reached_index, double & consumed)
{
  const std::size_t n = pts.size();
  consumed = 0.0;
  std::size_t cur = i;
  while (consumed < distance) {
    const std::size_t next = (cur + 1 >= n) ? (closed ? 0 : cur) : cur + 1;
    if (next == cur) {
      break;
    }
    const double seg = (pts[next] - pts[cur]).norm();
    if (consumed + seg >= distance) {
      const double t = (distance - consumed) / std::max(seg, kEps);
      consumed = distance;
      reached_index = next;
      return pts[cur] + t * (pts[next] - pts[cur]);
    }
    consumed += seg;
    cur = next;
    if (cur == i) {
      break;
    }
  }
  reached_index = cur;
  return pts[cur];
}

}  // namespace

void sampleSegment(
  const Segment & segment, Point2d start, const SamplingSpec & sampling,
  std::vector<Point2d> & out)
{
  switch (segment.type) {
    case SegmentType::kLine:
      sampleLine(start, segment.to, sampling, out);
      break;
    case SegmentType::kArc:
      sampleArc(segment, start, sampling, out);
      break;
    case SegmentType::kCircle:
      sampleCircle(segment, start, sampling, out);
      break;
    case SegmentType::kBspline:
      sampleBspline(segment, start, sampling, out);
      break;
  }
}

std::vector<Point2d> blendCorners(
  const std::vector<Point2d> & points, bool closed, double blend_distance,
  double max_segment_length)
{
  if (blend_distance <= kEps || points.size() < 3) {
    return points;
  }

  const std::size_t n = points.size();
  // A corner is a sample where the tangent turns sharply. Densely sampled
  // curves turn a little at every sample, so only genuine discontinuities --
  // the joins the author actually drew -- clear this threshold.
  constexpr double kCornerThreshold = 0.15;  // ~8.6 degrees between samples

  struct Corner
  {
    std::size_t index;
    Point2d enter;      // point blend_distance back along the path
    Point2d leave;      // point blend_distance forward along the path
    std::size_t enter_index;
    std::size_t leave_index;
  };

  std::vector<Corner> corners;
  const std::size_t first = closed ? 0 : 1;
  const std::size_t last = closed ? n : n - 1;
  for (std::size_t i = first; i < last; ++i) {
    const Point2d & a = points[(i == 0) ? n - 1 : i - 1];
    const Point2d & b = points[i % n];
    const Point2d & c = points[(i + 1) % n];
    if (std::abs(turnAngle(a, b, c)) < kCornerThreshold) {
      continue;
    }

    Corner corner;
    corner.index = i % n;
    double back = 0.0;
    double fwd = 0.0;
    corner.enter = walkBack(points, corner.index, blend_distance, closed, corner.enter_index, back);
    corner.leave =
      walkForward(points, corner.index, blend_distance, closed, corner.leave_index, fwd);
    if (back < 1e-6 || fwd < 1e-6) {
      continue;
    }
    corners.push_back(corner);
  }

  if (corners.empty()) {
    return points;
  }

  // Rebuild the polyline, replacing [enter_index+1 .. leave_index] around each
  // corner with a quadratic Bezier enter -> corner -> leave.
  std::vector<Point2d> out;
  out.reserve(points.size() + corners.size() * 16);

  std::size_t corner_cursor = 0;
  for (std::size_t i = 0; i < n; ++i) {
    // Is this index inside the region a corner is about to overwrite?
    bool skipped = false;
    for (const Corner & corner : corners) {
      const bool wraps = corner.enter_index > corner.leave_index;
      const bool inside = wraps
        ? (i > corner.enter_index || i <= corner.leave_index)
        : (i > corner.enter_index && i <= corner.leave_index);
      if (inside) {
        skipped = true;
        break;
      }
    }
    if (skipped) {
      continue;
    }
    out.push_back(points[i]);

    // Emit the blend right after the sample that precedes it.
    while (corner_cursor < corners.size() && corners[corner_cursor].enter_index == i) {
      const Corner & corner = corners[corner_cursor];
      const double span = (corner.enter - points[corner.index]).norm() +
        (points[corner.index] - corner.leave).norm();
      const int steps = std::max(6, stepsFor(span, max_segment_length));
      for (int s = 0; s <= steps; ++s) {
        const double t = static_cast<double>(s) / steps;
        const double w = 1.0 - t;
        out.push_back(
          w * w * corner.enter + 2.0 * w * t * points[corner.index] + t * t * corner.leave);
      }
      ++corner_cursor;
    }
  }

  // A corner that wrapped past the end of the buffer still needs emitting.
  while (corner_cursor < corners.size()) {
    const Corner & corner = corners[corner_cursor];
    const double span = (corner.enter - points[corner.index]).norm() +
      (points[corner.index] - corner.leave).norm();
    const int steps = std::max(6, stepsFor(span, max_segment_length));
    for (int s = 0; s <= steps; ++s) {
      const double t = static_cast<double>(s) / steps;
      const double w = 1.0 - t;
      out.push_back(
        w * w * corner.enter + 2.0 * w * t * points[corner.index] + t * t * corner.leave);
    }
    ++corner_cursor;
  }

  return out;
}

SampledOutline sampleOutline(const ShapeSpec & shape)
{
  if (shape.segments.empty()) {
    throw std::runtime_error("shape '" + shape.name + "' has no segments");
  }

  std::vector<Point2d> points;
  points.push_back(Point2d::Zero());  // the first vertex is always the origin

  SampledOutline outline;
  for (const Segment & segment : shape.segments) {
    outline.corner_indices.push_back(points.size() - 1);
    sampleSegment(segment, points.back(), shape.sampling, points);
  }

  if (shape.closed && (points.back() - points.front()).norm() > 1e-6) {
    outline.corner_indices.push_back(points.size() - 1);
    Segment closing;
    closing.type = SegmentType::kLine;
    closing.to = points.front();
    sampleSegment(closing, points.back(), shape.sampling, points);
  }

  if (points.size() < 2) {
    throw std::runtime_error("shape '" + shape.name + "' sampled to fewer than 2 points");
  }

  outline.points = blendCorners(
    points, shape.closed, shape.sampling.blend_distance, shape.sampling.max_segment_length);
  return outline;
}

Eigen::Quaterniond toolOrientation(
  const Eigen::Isometry3d & shape_frame, double normal_sign, double spin)
{
  // The tool's approach axis (+Z of the end-effector link) must be parallel to
  // normal_sign * Z_S. Rotating the shape frame by pi about its own X flips Z
  // (and Y) while keeping X, which gives the "pen pointing into the page" pose;
  // normal_sign = +1 skips that flip. The remaining freedom is a rotation about
  // the tool's own Z, which is the (task-irrelevant) spin.
  Eigen::Matrix3d r_s_tool = Eigen::Matrix3d::Identity();
  if (normal_sign < 0.0) {
    r_s_tool = Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  r_s_tool = r_s_tool * Eigen::AngleAxisd(spin, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return Eigen::Quaterniond(shape_frame.rotation() * r_s_tool).normalized();
}

CartesianTrace buildTrace(
  const ShapeSpec & shape, double approach_distance, double spin_override, bool use_spin_override)
{
  CartesianTrace trace;
  trace.shape_frame = shape.start_pose;
  trace.outline = sampleOutline(shape);

  const double spin = use_spin_override ? spin_override : shape.tool.spin;
  trace.tool_orientation = toolOrientation(shape.start_pose, shape.tool.normal_sign, spin);

  trace.waypoints.reserve(trace.outline.points.size());
  for (const Point2d & p : trace.outline.points) {
    // The single transform that takes authored 2D coordinates back to robot
    // coordinates. Everything downstream works in the reference frame.
    const Eigen::Vector3d p_shape(p.x(), p.y(), shape.tool.standoff);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = shape.start_pose * p_shape;
    pose.linear() = trace.tool_orientation.toRotationMatrix();
    trace.waypoints.push_back(pose);
  }

  for (std::size_t i = 1; i < trace.outline.points.size(); ++i) {
    trace.length += (trace.outline.points[i] - trace.outline.points[i - 1]).norm();
  }

  // Approach and retreat back off along the plane normal, so transits between
  // shapes never drag the tool across the drawing plane.
  const Eigen::Vector3d normal = shape.start_pose.rotation().col(2);
  trace.approach_pose = trace.waypoints.front();
  trace.approach_pose.translation() += approach_distance * normal;
  trace.retreat_pose = trace.waypoints.back();
  trace.retreat_pose.translation() += approach_distance * normal;

  return trace;
}

}  // namespace avatar_challenge

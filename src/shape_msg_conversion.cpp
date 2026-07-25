// Copyright 2026 Renatto Tommasi

#include "avatar_challenge/shape_msg_conversion.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace avatar_challenge
{
namespace
{

Point2d asPoint2d(const msg::Point2D & p)
{
  return Point2d(p.x, p.y);
}

Segment parseSegment(const msg::Segment & msg, const std::string & where)
{
  Segment seg;
  seg.to = asPoint2d(msg.to);
  seg.has_center = msg.has_center;
  seg.center = asPoint2d(msg.center);
  seg.radius = msg.radius;
  seg.counter_clockwise = msg.counter_clockwise;
  seg.large_arc = msg.large_arc;
  seg.periodic = msg.periodic;
  if (msg.degree > 0) {
    seg.degree = msg.degree;
  }
  for (const msg::Point2D & p : msg.control_points) {
    seg.control_points.push_back(asPoint2d(p));
  }

  switch (msg.type) {
    case msg::Segment::LINE:
      seg.type = SegmentType::kLine;
      break;
    case msg::Segment::ARC:
      seg.type = SegmentType::kArc;
      if (!seg.has_center && seg.radius == 0.0) {
        throw std::runtime_error(where + ": arc needs either 'center' or 'radius'");
      }
      break;
    case msg::Segment::CIRCLE:
      seg.type = SegmentType::kCircle;
      if (!seg.has_center) {
        throw std::runtime_error(where + ": circle needs 'center'");
      }
      break;
    case msg::Segment::BSPLINE:
      seg.type = SegmentType::kBspline;
      if (seg.control_points.empty()) {
        throw std::runtime_error(where + ": bspline needs 'control_points'");
      }
      break;
    default:
      throw std::runtime_error(
              where + ": unknown segment type " + std::to_string(msg.type) +
              " (0=line, 1=arc, 2=circle, 3=bspline)");
  }
  return seg;
}

}  // namespace

ShapeSpec fromMsg(const msg::Shape & msg, const ProgramDefaults & defaults)
{
  ShapeSpec shape;
  shape.name = msg.name;
  const std::string where = "shape[" + (msg.name.empty() ? "<unnamed>" : msg.name) + "]";

  shape.reference_frame = msg.frame.empty() ? defaults.reference_frame : msg.frame;
  shape.closed = msg.closed;

  // A default-constructed geometry_msgs/Quaternion is all zeros, which is not a
  // rotation at all. Catching it here beats letting Eigen normalise a zero
  // quaternion into NaNs that only surface as an unexplained IK failure.
  const auto & q = msg.start.orientation;
  if (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 1e-9) {
    throw std::runtime_error(
            where + ": start.orientation is a zero quaternion — set it to a real "
            "rotation (w=1 for identity)");
  }
  tf2::fromMsg(msg.start, shape.start_pose);

  shape.tool = defaults.tool;
  if (msg.has_tool) {
    shape.tool.normal_sign =
      msg.tool.face == msg::ToolSpec::FACE_ALONG_NORMAL ? 1.0 : -1.0;
    shape.tool.spin = msg.tool.spin;
    shape.tool.free_spin = msg.tool.free_spin;
    shape.tool.spin_samples =
      msg.tool.spin_samples > 0 ? msg.tool.spin_samples : defaults.tool.spin_samples;
    shape.tool.standoff = msg.tool.standoff;
  }

  shape.sampling = defaults.sampling;
  if (msg.has_sampling) {
    shape.sampling.max_segment_length = msg.sampling.max_segment_length;
    shape.sampling.chord_tolerance = msg.sampling.chord_tolerance;
    shape.sampling.blend_distance = msg.sampling.blend_distance;
    if (shape.sampling.max_segment_length <= 0.0 || shape.sampling.chord_tolerance <= 0.0) {
      throw std::runtime_error(
              where + ": sampling.max_segment_length and sampling.chord_tolerance must be "
              "positive when has_sampling is set");
    }
  }

  if (msg.velocity_scaling > 0.0) {
    shape.velocity_scaling = msg.velocity_scaling;
  }
  if (msg.acceleration_scaling > 0.0) {
    shape.acceleration_scaling = msg.acceleration_scaling;
  }

  const bool has_vertices = !msg.vertices.empty();
  const bool has_path = !msg.path.empty();
  if (has_vertices == has_path) {
    throw std::runtime_error(where + ": provide exactly one of 'vertices' or 'path'");
  }

  if (has_vertices) {
    // Same sugar as the YAML polygon form: a vertex list becomes a chain of
    // lines, and the first vertex is validated rather than consumed because it
    // defines the shape frame origin.
    if (msg.vertices.size() < 2) {
      throw std::runtime_error(where + ": 'vertices' needs at least 2 entries");
    }
    if (asPoint2d(msg.vertices[0]).norm() > 1e-9) {
      throw std::runtime_error(
              where + ": the first vertex must be (0, 0) — it defines the shape frame origin");
    }
    for (std::size_t v = 1; v < msg.vertices.size(); ++v) {
      Segment seg;
      seg.type = SegmentType::kLine;
      seg.to = asPoint2d(msg.vertices[v]);
      shape.segments.push_back(seg);
    }
  } else {
    for (std::size_t s = 0; s < msg.path.size(); ++s) {
      shape.segments.push_back(
        parseSegment(msg.path[s], where + ".path[" + std::to_string(s) + "]"));
    }
  }

  return shape;
}

ShapeProgram fromMsg(
  const msg::ShapeArray & msg, const ProgramDefaults & defaults,
  std::vector<std::string> & errors)
{
  ShapeProgram program;
  program.shapes.reserve(msg.shapes.size());
  for (const msg::Shape & shape : msg.shapes) {
    try {
      program.shapes.push_back(fromMsg(shape, defaults));
    } catch (const std::exception & e) {
      errors.push_back(e.what());
    }
  }
  return program;
}

}  // namespace avatar_challenge

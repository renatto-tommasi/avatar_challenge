// Copyright 2026 Renatto Tommasi

#include "avatar_challenge/glyph_shapes.hpp"

#include <string>

#include "avatar_challenge/msg/point2_d.hpp"
#include "avatar_challenge/msg/segment.hpp"
#include "avatar_challenge/msg/shape.hpp"

namespace avatar_challenge
{
namespace
{

msg::Point2D point2d(const Point2d & point)
{
  msg::Point2D out;
  out.x = point.x();
  out.y = point.y();
  return out;
}

std::uint8_t segmentType(SegmentType type)
{
  switch (type) {
    case SegmentType::kLine:
      return msg::Segment::LINE;
    case SegmentType::kArc:
      return msg::Segment::ARC;
    case SegmentType::kCircle:
      return msg::Segment::CIRCLE;
    case SegmentType::kBspline:
      break;
  }
  return msg::Segment::BSPLINE;
}

}  // namespace

Eigen::Vector3d WritingPlane::point(double u, double v) const
{
  return Eigen::Vector3d(distance, (mirror ? 1.0 : -1.0) * u, v);
}

Eigen::Quaterniond WritingPlane::orientation() const
{
  Eigen::Matrix3d rotation;
  rotation.col(0) = Eigen::Vector3d(0.0, mirror ? 1.0 : -1.0, 0.0);  // X_S: writing direction
  rotation.col(1) = Eigen::Vector3d(0.0, 0.0, 1.0);                  // Y_S: up
  rotation.col(2) = rotation.col(0).cross(rotation.col(1));          // Z_S: plane normal
  return Eigen::Quaterniond(rotation);
}

msg::ShapeArray glyphToShapes(
  const PlacedGlyph & placed, const WritingPlane & plane, const StrokeStyle & style,
  const std::string & name_prefix, uint8_t mode)
{
  msg::ShapeArray batch;
  batch.mode = mode;
  if (placed.glyph == nullptr) {
    return batch;
  }

  const Eigen::Quaterniond orientation = plane.orientation();

  for (std::size_t i = 0; i < placed.glyph->strokes.size(); ++i) {
    const GlyphStroke & stroke = placed.glyph->strokes[i];

    msg::Shape shape;
    shape.name = name_prefix + "_s" + std::to_string(i);
    shape.frame = plane.frame;

    // The shape frame's origin is the stroke's own first point — the invariant
    // the tracer enforces — so everything else in the message is measured from
    // there.
    const Eigen::Vector3d origin = plane.point(
      placed.u + stroke.start.x() * placed.scale, placed.v + stroke.start.y() * placed.scale);
    shape.start.position.x = origin.x();
    shape.start.position.y = origin.y();
    shape.start.position.z = origin.z();
    shape.start.orientation.x = orientation.x();
    shape.start.orientation.y = orientation.y();
    shape.start.orientation.z = orientation.z();
    shape.start.orientation.w = orientation.w();

    // Glyph coordinates are in cap heights and absolute within the glyph; the
    // message wants metres from the stroke's first point. X_S and Y_S are u and
    // v, so that is a scale and a shift, with no rotation in it.
    const auto local = [&](const Point2d & point) {
        return point2d((point - stroke.start) * placed.scale);
      };

    for (const Segment & segment : stroke.segments) {
      msg::Segment out;
      out.type = segmentType(segment.type);
      out.to = local(segment.to);
      out.has_center = segment.has_center;
      out.center = local(segment.center);
      out.radius = segment.radius * placed.scale;
      out.counter_clockwise = segment.counter_clockwise;
      out.large_arc = segment.large_arc;
      out.degree = segment.degree;
      out.periodic = segment.periodic;
      for (const Point2d & control : segment.control_points) {
        out.control_points.push_back(local(control));
      }
      shape.path.push_back(out);
    }

    shape.closed = stroke.closed;
    shape.has_tool = true;
    shape.tool = style.tool;
    shape.has_sampling = true;
    shape.sampling = style.sampling;
    shape.velocity_scaling = style.velocity_scaling;
    shape.acceleration_scaling = style.acceleration_scaling;
    batch.shapes.push_back(std::move(shape));
  }
  return batch;
}

}  // namespace avatar_challenge

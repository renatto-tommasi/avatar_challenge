// Copyright 2026 Renatto Tommasi
//
// Placed glyphs -> shape messages.
//
// This is where 2D type meets the robot: a glyph laid out in plane coordinates
// becomes one avatar_challenge/msg/ShapeArray, with one Shape per pen-down
// stroke. Keeping it out of the node means the tracer's own validator can be
// pointed at the result in a unit test — if fromMsg() accepts every letter of
// the font here, the tracer will accept them on the wire.

#ifndef AVATAR_CHALLENGE__GLYPH_SHAPES_HPP_
#define AVATAR_CHALLENGE__GLYPH_SHAPES_HPP_

#include <Eigen/Geometry>

#include <string>

#include "avatar_challenge/msg/shape_array.hpp"
#include "avatar_challenge/text_layout.hpp"

namespace avatar_challenge
{

/// A writing plane parallel to the robot's YZ plane, at x = `distance`.
///
/// Plane coordinates are u along the writing direction and v up. Reading left
/// to right for someone behind the robot looking out along +X puts u on -Y,
/// which is why point() negates it; `mirror` flips that for a reader on the
/// other side of the plane.
struct WritingPlane
{
  std::string frame{"link_base"};
  double distance{0.42};
  bool mirror{false};

  Eigen::Vector3d point(double u, double v) const;

  /// Orientation of every shape frame on this plane: X_S along the writing
  /// direction, Y_S up, and therefore Z_S normal to the plane. Constant for the
  /// whole text, which is what writing on a plane means.
  Eigen::Quaterniond orientation() const;
};

/// How the strokes are to be drawn — the parts of a Shape that are not its
/// geometry.
struct StrokeStyle
{
  msg::ToolSpec tool;
  msg::SamplingSpec sampling;
  /// Non-positive inherits the tracer's own motion defaults.
  double velocity_scaling{0.0};
  double acceleration_scaling{0.0};
};

/// One placed glyph as a batch of shapes: stroke i is named
/// "<name_prefix>_s<i>". `mode` is ShapeArray::APPEND or ::REPLACE.
msg::ShapeArray glyphToShapes(
  const PlacedGlyph & placed, const WritingPlane & plane, const StrokeStyle & style,
  const std::string & name_prefix, uint8_t mode);

}  // namespace avatar_challenge

#endif  // AVATAR_CHALLENGE__GLYPH_SHAPES_HPP_

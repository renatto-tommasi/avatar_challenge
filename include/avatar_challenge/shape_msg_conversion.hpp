// Copyright 2026 Renatto Tommasi
//
// avatar_challenge_msgs -> ShapeProgram.
//
// The topic interface is the second way shapes enter the node, so it has to
// enforce exactly the same invariants as the YAML loader — the geometry code
// downstream assumes them either way. See src/yaml_loader.cpp for the parser
// these checks mirror, and msg/Shape.msg for the schema.

#ifndef AVATAR_CHALLENGE__SHAPE_MSG_CONVERSION_HPP_
#define AVATAR_CHALLENGE__SHAPE_MSG_CONVERSION_HPP_

#include <string>
#include <vector>

#include "avatar_challenge/msg/shape.hpp"
#include "avatar_challenge/msg/shape_array.hpp"
#include "avatar_challenge/shape_spec.hpp"
#include "avatar_challenge/yaml_loader.hpp"

namespace avatar_challenge
{

/// Convert one shape message. `defaults` supplies whatever the message asks to
/// inherit (has_tool / has_sampling false, non-positive scalings).
/// Throws std::runtime_error naming the offending shape/segment, in the same
/// style as loadShapeProgram.
ShapeSpec fromMsg(const msg::Shape & msg, const ProgramDefaults & defaults);

/// Convert a batch. One bad shape does not sink the rest: it is skipped and its
/// error appended to `errors`, so a producer emitting ten shapes does not lose
/// nine of them to one typo.
ShapeProgram fromMsg(
  const msg::ShapeArray & msg, const ProgramDefaults & defaults,
  std::vector<std::string> & errors);

}  // namespace avatar_challenge

#endif  // AVATAR_CHALLENGE__SHAPE_MSG_CONVERSION_HPP_

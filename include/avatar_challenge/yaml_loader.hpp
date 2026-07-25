// Copyright 2026 Renatto Tommasi
//
// YAML -> ShapeProgram. See config/shapes.yaml for the documented schema.

#ifndef AVATAR_CHALLENGE__YAML_LOADER_HPP_
#define AVATAR_CHALLENGE__YAML_LOADER_HPP_

#include <string>

#include "avatar_challenge/shape_spec.hpp"

namespace avatar_challenge
{

/// Defaults applied to every shape unless the shape overrides them.
struct ProgramDefaults
{
  std::string reference_frame{"link_base"};
  ToolSpec tool;
  SamplingSpec sampling;
  double velocity_scaling{0.3};
  double acceleration_scaling{0.3};
  double approach_distance{0.05};
};

/// Parse a shapes file. Throws std::runtime_error with a message that names the
/// offending shape/segment on any schema problem.
ShapeProgram loadShapeProgram(const std::string & path, ProgramDefaults & defaults);

}  // namespace avatar_challenge

#endif  // AVATAR_CHALLENGE__YAML_LOADER_HPP_

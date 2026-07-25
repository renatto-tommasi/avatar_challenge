// Copyright 2026 Renatto Tommasi
//
// RViz markers: the shape frame, the drawing plane, the target outline and the
// path the tool actually swept.

#ifndef AVATAR_CHALLENGE__MARKERS_HPP_
#define AVATAR_CHALLENGE__MARKERS_HPP_

#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

#include "avatar_challenge/path_sampler.hpp"
#include "avatar_challenge/shape_spec.hpp"

namespace avatar_challenge
{

/// Markers describing one shape: a translucent plane patch, the outline itself,
/// the shape-frame axes and a name label. `id_base` reserves 16 marker ids.
visualization_msgs::msg::MarkerArray shapeMarkers(
  const ShapeSpec & shape, const CartesianTrace & trace, const std::string & frame_id,
  int id_base, const rclcpp::Time & stamp);

/// A polyline of tool positions, published as the trace is executed.
visualization_msgs::msg::Marker tracedPathMarker(
  const std::vector<Eigen::Vector3d> & points, const std::string & frame_id, int id,
  const rclcpp::Time & stamp);

}  // namespace avatar_challenge

#endif  // AVATAR_CHALLENGE__MARKERS_HPP_

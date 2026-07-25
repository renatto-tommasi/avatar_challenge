// Copyright 2026 Renatto Tommasi

#include "avatar_challenge/markers.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <array>
#include <string>
#include <vector>

namespace avatar_challenge
{
namespace
{

std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

visualization_msgs::msg::Marker baseMarker(
  const std::string & frame_id, const rclcpp::Time & stamp, const std::string & ns, int id,
  int32_t type)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

}  // namespace

visualization_msgs::msg::MarkerArray shapeMarkers(
  const ShapeSpec & shape, const CartesianTrace & trace, const std::string & frame_id,
  int id_base, const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray array;

  // --- the outline, drawn on the plane -----------------------------------
  auto outline = baseMarker(
    frame_id, stamp, "shape_outline", id_base,
    visualization_msgs::msg::Marker::LINE_STRIP);
  outline.scale.x = 0.003;
  outline.color = rgba(0.15f, 0.85f, 0.35f, 1.0f);
  for (const Eigen::Isometry3d & pose : trace.waypoints) {
    outline.points.push_back(tf2::toMsg(Eigen::Vector3d(pose.translation())));
  }
  array.markers.push_back(outline);

  // --- translucent patch showing the plane the shape lives on ------------
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  for (const Point2d & p : trace.outline.points) {
    min_x = std::min(min_x, p.x());
    max_x = std::max(max_x, p.x());
    min_y = std::min(min_y, p.y());
    max_y = std::max(max_y, p.y());
  }
  const double pad = 0.03;
  Eigen::Isometry3d patch_pose = shape.start_pose;
  patch_pose.translation() =
    shape.start_pose * Eigen::Vector3d(0.5 * (min_x + max_x), 0.5 * (min_y + max_y), 0.0);

  auto plane = baseMarker(
    frame_id, stamp, "shape_plane", id_base + 1, visualization_msgs::msg::Marker::CUBE);
  plane.pose = tf2::toMsg(patch_pose);
  plane.scale.x = std::max(0.02, max_x - min_x + 2 * pad);
  plane.scale.y = std::max(0.02, max_y - min_y + 2 * pad);
  plane.scale.z = 0.001;
  plane.color = rgba(0.30f, 0.55f, 0.95f, 0.18f);
  array.markers.push_back(plane);

  // --- shape frame axes: X red, Y green, Z (the plane normal) blue --------
  const std::array<Eigen::Vector3d, 3> axes = {
    Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()};
  const std::array<std_msgs::msg::ColorRGBA, 3> axis_colors = {
    rgba(0.9f, 0.2f, 0.2f, 0.95f), rgba(0.2f, 0.9f, 0.2f, 0.95f), rgba(0.25f, 0.45f, 1.0f, 0.95f)};
  for (int i = 0; i < 3; ++i) {
    auto axis = baseMarker(
      frame_id, stamp, "shape_frame", id_base + 2 + i, visualization_msgs::msg::Marker::ARROW);
    axis.scale.x = 0.004;   // shaft diameter
    axis.scale.y = 0.010;   // head diameter
    axis.scale.z = 0.012;   // head length
    axis.color = axis_colors[i];
    const Eigen::Vector3d origin = shape.start_pose.translation();
    axis.points.push_back(tf2::toMsg(origin));
    axis.points.push_back(
      tf2::toMsg(Eigen::Vector3d(origin + 0.06 * (shape.start_pose.rotation() * axes[i]))));
    array.markers.push_back(axis);
  }

  // --- the entry point and the approach standoff above it -----------------
  auto entry = baseMarker(
    frame_id, stamp, "shape_entry", id_base + 5, visualization_msgs::msg::Marker::SPHERE);
  entry.pose.position = tf2::toMsg(Eigen::Vector3d(trace.waypoints.front().translation()));
  entry.scale.x = entry.scale.y = entry.scale.z = 0.012;
  entry.color = rgba(1.0f, 0.75f, 0.1f, 1.0f);
  array.markers.push_back(entry);

  auto approach = baseMarker(
    frame_id, stamp, "shape_approach", id_base + 6, visualization_msgs::msg::Marker::LINE_LIST);
  approach.scale.x = 0.002;
  approach.color = rgba(1.0f, 0.75f, 0.1f, 0.6f);
  approach.points.push_back(tf2::toMsg(Eigen::Vector3d(trace.approach_pose.translation())));
  approach.points.push_back(tf2::toMsg(Eigen::Vector3d(trace.waypoints.front().translation())));
  array.markers.push_back(approach);

  // --- name label ---------------------------------------------------------
  auto label = baseMarker(
    frame_id, stamp, "shape_label", id_base + 7,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  label.pose.position = tf2::toMsg(
    Eigen::Vector3d(shape.start_pose.translation() + Eigen::Vector3d(0, 0, 0.04)));
  label.scale.z = 0.025;
  label.color = rgba(1.0f, 1.0f, 1.0f, 0.9f);
  label.text = shape.name;
  array.markers.push_back(label);

  return array;
}

visualization_msgs::msg::Marker tracedPathMarker(
  const std::vector<Eigen::Vector3d> & points, const std::string & frame_id, int id,
  const rclcpp::Time & stamp)
{
  auto marker = baseMarker(
    frame_id, stamp, "traced_path", id, visualization_msgs::msg::Marker::LINE_STRIP);
  marker.scale.x = 0.005;
  marker.color = rgba(1.0f, 0.35f, 0.85f, 1.0f);
  marker.points.reserve(points.size());
  for (const Eigen::Vector3d & p : points) {
    marker.points.push_back(tf2::toMsg(p));
  }
  return marker;
}

}  // namespace avatar_challenge

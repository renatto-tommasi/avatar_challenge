// Copyright 2026 Renatto Tommasi

#include "avatar_challenge/yaml_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace avatar_challenge
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

/// Scale factors that turn authored numbers into metres and radians.
struct Units
{
  double length{1.0};
  double angle{1.0};
};

Units parseUnits(const YAML::Node & node, Units inherited)
{
  if (!node || !node.IsMap()) {
    return inherited;
  }
  Units units = inherited;
  if (node["length"]) {
    const std::string u = node["length"].as<std::string>();
    if (u == "m" || u == "meters" || u == "metres") {
      units.length = 1.0;
    } else if (u == "mm" || u == "millimeters" || u == "millimetres") {
      units.length = 0.001;
    } else if (u == "cm") {
      units.length = 0.01;
    } else {
      throw std::runtime_error("units.length must be one of m|cm|mm, got '" + u + "'");
    }
  }
  if (node["angle"]) {
    const std::string u = node["angle"].as<std::string>();
    if (u == "rad" || u == "radians") {
      units.angle = 1.0;
    } else if (u == "deg" || u == "degrees") {
      units.angle = kPi / 180.0;
    } else {
      throw std::runtime_error("units.angle must be one of rad|deg, got '" + u + "'");
    }
  }
  return units;
}

std::vector<double> asDoubles(const YAML::Node & node, const std::string & what, std::size_t expect)
{
  if (!node || !node.IsSequence() || node.size() != expect) {
    throw std::runtime_error(
            what + ": expected a sequence of " + std::to_string(expect) + " numbers");
  }
  std::vector<double> out;
  out.reserve(expect);
  for (const YAML::Node & item : node) {
    out.push_back(item.as<double>());
  }
  return out;
}

Point2d asPoint2d(const YAML::Node & node, const std::string & what, double length_scale)
{
  const std::vector<double> v = asDoubles(node, what, 2);
  return Point2d(v[0] * length_scale, v[1] * length_scale);
}

Eigen::Isometry3d parseStartPose(const YAML::Node & node, const Units & units)
{
  if (!node || !node.IsMap()) {
    throw std::runtime_error("'start' must be a map with 'position' and an orientation");
  }

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  const std::vector<double> p = asDoubles(node["position"], "start.position", 3);
  pose.translation() = Eigen::Vector3d(p[0], p[1], p[2]) * units.length;

  if (node["orientation_rpy"]) {
    const std::vector<double> rpy = asDoubles(node["orientation_rpy"], "start.orientation_rpy", 3);
    // Fixed-axis roll-pitch-yaw, i.e. R = Rz(yaw) * Ry(pitch) * Rx(roll). This
    // is the same convention ROS uses everywhere else (tf, urdf origins), so a
    // yaw of pi/4 here matches "angled 45 degrees about Z" in the brief.
    const Eigen::AngleAxisd roll(rpy[0] * units.angle, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(rpy[1] * units.angle, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(rpy[2] * units.angle, Eigen::Vector3d::UnitZ());
    pose.linear() = (yaw * pitch * roll).toRotationMatrix();
  } else if (node["orientation_quat_xyzw"]) {
    const std::vector<double> q =
      asDoubles(node["orientation_quat_xyzw"], "start.orientation_quat_xyzw", 4);
    pose.linear() = Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized().toRotationMatrix();
  } else {
    throw std::runtime_error("'start' needs orientation_rpy or orientation_quat_xyzw");
  }
  return pose;
}

ToolSpec parseTool(const YAML::Node & node, ToolSpec inherited, const Units & units)
{
  if (!node || !node.IsMap()) {
    return inherited;
  }
  ToolSpec tool = inherited;
  if (node["face"]) {
    const std::string face = node["face"].as<std::string>();
    if (face == "into_plane") {
      tool.normal_sign = -1.0;
    } else if (face == "along_normal") {
      tool.normal_sign = 1.0;
    } else {
      throw std::runtime_error("tool.face must be into_plane|along_normal, got '" + face + "'");
    }
  }
  if (node["spin"]) {
    tool.spin = node["spin"].as<double>() * units.angle;
  }
  if (node["free_spin"]) {
    tool.free_spin = node["free_spin"].as<bool>();
  }
  if (node["spin_samples"]) {
    tool.spin_samples = std::max(1, node["spin_samples"].as<int>());
  }
  if (node["standoff"]) {
    tool.standoff = node["standoff"].as<double>() * units.length;
  }
  return tool;
}

SamplingSpec parseSampling(const YAML::Node & node, SamplingSpec inherited, const Units & units)
{
  if (!node || !node.IsMap()) {
    return inherited;
  }
  SamplingSpec sampling = inherited;
  if (node["max_segment_length"]) {
    sampling.max_segment_length = node["max_segment_length"].as<double>() * units.length;
  }
  if (node["chord_tolerance"]) {
    sampling.chord_tolerance = node["chord_tolerance"].as<double>() * units.length;
  }
  if (node["blend_distance"]) {
    sampling.blend_distance = node["blend_distance"].as<double>() * units.length;
  }
  return sampling;
}

Segment parseSegment(const YAML::Node & node, const Units & units, const std::string & where)
{
  if (!node.IsMap() || !node["type"]) {
    throw std::runtime_error(where + ": each path entry needs a 'type'");
  }
  const std::string type = node["type"].as<std::string>();

  Segment seg;
  if (node["direction"]) {
    const std::string dir = node["direction"].as<std::string>();
    if (dir == "ccw" || dir == "counter_clockwise") {
      seg.counter_clockwise = true;
    } else if (dir == "cw" || dir == "clockwise") {
      seg.counter_clockwise = false;
    } else {
      throw std::runtime_error(where + ": direction must be cw|ccw, got '" + dir + "'");
    }
  }
  if (node["center"]) {
    seg.has_center = true;
    seg.center = asPoint2d(node["center"], where + ".center", units.length);
  }
  if (node["radius"]) {
    seg.radius = node["radius"].as<double>() * units.length;
  }
  if (node["large_arc"]) {
    seg.large_arc = node["large_arc"].as<bool>();
  }
  if (node["to"]) {
    seg.to = asPoint2d(node["to"], where + ".to", units.length);
  }

  if (type == "line") {
    seg.type = SegmentType::kLine;
    if (!node["to"]) {
      throw std::runtime_error(where + ": line needs 'to'");
    }
  } else if (type == "arc") {
    seg.type = SegmentType::kArc;
    if (!node["to"]) {
      throw std::runtime_error(where + ": arc needs 'to'");
    }
    // An absent key yields an undefined node, and IsNull() is false for those,
    // so test the node itself rather than its nullness.
    if (!seg.has_center && !node["radius"]) {
      throw std::runtime_error(where + ": arc needs either 'center' or 'radius'");
    }
  } else if (type == "circle") {
    seg.type = SegmentType::kCircle;
    if (!seg.has_center) {
      throw std::runtime_error(where + ": circle needs 'center'");
    }
  } else if (type == "bspline") {
    seg.type = SegmentType::kBspline;
    if (!node["control_points"] || !node["control_points"].IsSequence()) {
      throw std::runtime_error(where + ": bspline needs 'control_points'");
    }
    for (std::size_t i = 0; i < node["control_points"].size(); ++i) {
      seg.control_points.push_back(
        asPoint2d(
          node["control_points"][i], where + ".control_points[" + std::to_string(i) + "]",
          units.length));
    }
    if (node["degree"]) {
      seg.degree = node["degree"].as<int>();
    }
    if (node["periodic"]) {
      seg.periodic = node["periodic"].as<bool>();
    }
  } else {
    throw std::runtime_error(
            where + ": unknown segment type '" + type + "' (line|arc|circle|bspline)");
  }
  return seg;
}

}  // namespace

ShapeProgram loadShapeProgram(const std::string & path, ProgramDefaults & defaults)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("could not parse '" + path + "': " + e.what());
  }

  Units file_units;
  if (root["defaults"]) {
    const YAML::Node & d = root["defaults"];
    file_units = parseUnits(d["units"], file_units);
    if (d["reference_frame"]) {
      defaults.reference_frame = d["reference_frame"].as<std::string>();
    }
    defaults.tool = parseTool(d["tool"], defaults.tool, file_units);
    defaults.sampling = parseSampling(d["sampling"], defaults.sampling, file_units);
    if (d["motion"]) {
      const YAML::Node & m = d["motion"];
      if (m["velocity_scaling"]) {
        defaults.velocity_scaling = m["velocity_scaling"].as<double>();
      }
      if (m["acceleration_scaling"]) {
        defaults.acceleration_scaling = m["acceleration_scaling"].as<double>();
      }
      if (m["approach_distance"]) {
        defaults.approach_distance = m["approach_distance"].as<double>() * file_units.length;
      }
    }
  }

  // A file with no shapes is legitimate: it can still carry a `defaults:` block
  // for shapes that arrive later on the tracer's ~/add_shapes topic. A `shapes:`
  // key that is present but not a sequence is still a mistake worth reporting.
  if (root["shapes"] && !root["shapes"].IsSequence()) {
    throw std::runtime_error("'" + path + "': 'shapes' must be a sequence");
  }
  if (!root["shapes"]) {
    return ShapeProgram{};
  }

  ShapeProgram program;
  for (std::size_t i = 0; i < root["shapes"].size(); ++i) {
    const YAML::Node & node = root["shapes"][i];
    ShapeSpec shape;
    shape.name = node["name"] ? node["name"].as<std::string>() : ("shape_" + std::to_string(i));
    const std::string where = "shapes[" + shape.name + "]";

    const Units units = parseUnits(node["units"], file_units);
    shape.reference_frame =
      node["frame"] ? node["frame"].as<std::string>() : defaults.reference_frame;
    shape.start_pose = parseStartPose(node["start"], units);
    shape.tool = parseTool(node["tool"], defaults.tool, units);
    shape.sampling = parseSampling(node["sampling"], defaults.sampling, units);
    shape.closed = node["closed"] ? node["closed"].as<bool>() : true;
    if (node["velocity_scaling"]) {
      shape.velocity_scaling = node["velocity_scaling"].as<double>();
    }
    if (node["acceleration_scaling"]) {
      shape.acceleration_scaling = node["acceleration_scaling"].as<double>();
    }

    const bool has_vertices = node["vertices"] && node["vertices"].IsSequence();
    const bool has_path = node["path"] && node["path"].IsSequence();
    if (has_vertices == has_path) {
      throw std::runtime_error(where + ": provide exactly one of 'vertices' or 'path'");
    }

    if (has_vertices) {
      // Sugar for a polygon: a vertex list becomes a chain of line segments.
      // The challenge fixes the first vertex at the origin of the shape frame,
      // so it is validated rather than consumed.
      const YAML::Node & vertices = node["vertices"];
      if (vertices.size() < 2) {
        throw std::runtime_error(where + ": 'vertices' needs at least 2 entries");
      }
      const Point2d first =
        asPoint2d(vertices[0], where + ".vertices[0]", units.length);
      if (first.norm() > 1e-9) {
        throw std::runtime_error(
                where + ": the first vertex must be (0, 0) — it defines the shape frame origin");
      }
      for (std::size_t v = 1; v < vertices.size(); ++v) {
        Segment seg;
        seg.type = SegmentType::kLine;
        seg.to =
          asPoint2d(vertices[v], where + ".vertices[" + std::to_string(v) + "]", units.length);
        shape.segments.push_back(seg);
      }
    } else {
      for (std::size_t s = 0; s < node["path"].size(); ++s) {
        shape.segments.push_back(
          parseSegment(node["path"][s], units, where + ".path[" + std::to_string(s) + "]"));
      }
    }

    program.shapes.push_back(shape);
  }

  return program;
}

}  // namespace avatar_challenge

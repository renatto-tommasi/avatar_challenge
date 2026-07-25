// Copyright 2026 Renatto Tommasi

#include "avatar_challenge/alphabet.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "avatar_challenge/path_sampler.hpp"

namespace avatar_challenge
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

/// How far a segment's implied start may sit from where the pen actually is
/// before the font is called broken, in design units. Tight on purpose: the
/// point of the angle-defined arc syntax is that endpoints land exactly.
constexpr double kJoinTolerance = 1e-3;

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

Point2d asPoint2d(const YAML::Node & node, const std::string & what)
{
  const std::vector<double> v = asDoubles(node, what, 2);
  return Point2d(v[0], v[1]);
}

double optionalDouble(const YAML::Node & node, const char * key, double fallback)
{
  return node[key] ? node[key].as<double>() : fallback;
}

bool parseCounterClockwise(const YAML::Node & node, const std::string & what)
{
  if (!node["direction"]) {
    return true;
  }
  const std::string direction = node["direction"].as<std::string>();
  if (direction == "ccw" || direction == "counter_clockwise") {
    return true;
  }
  if (direction == "cw" || direction == "clockwise") {
    return false;
  }
  throw std::runtime_error(what + ": direction must be cw or ccw, got '" + direction + "'");
}

Point2d onCircle(const Point2d & center, double radius, double degrees)
{
  const double a = degrees * kPi / 180.0;
  return Point2d(center.x() + radius * std::cos(a), center.y() + radius * std::sin(a));
}

/// Where a stroke that did not declare a `start:` begins. Only primitives that
/// carry their own geometry can answer this — an angle-defined arc, a circle
/// with a radius, or a B-spline — which is why lines and `to`-defined arcs
/// oblige the author to write the start point out.
Point2d impliedStart(const YAML::Node & node, const std::string & what)
{
  if (!node || !node.IsMap() || !node["type"]) {
    throw std::runtime_error(what + ": expected a map with a 'type'");
  }
  const std::string type = node["type"].as<std::string>();
  if (type == "arc" && node["center"] && node["radius"] && node["start_angle"]) {
    return onCircle(
      asPoint2d(node["center"], what + ".center"), node["radius"].as<double>(),
      node["start_angle"].as<double>());
  }
  if (type == "circle" && node["center"] && node["radius"]) {
    // A circle sweeps from wherever the pen is, so opening a stroke with one
    // means starting at angle 0 on it.
    return onCircle(
      asPoint2d(node["center"], what + ".center"), node["radius"].as<double>(), 0.0);
  }
  if (type == "bspline" && node["control_points"] && node["control_points"].size() > 0) {
    return asPoint2d(node["control_points"][0], what + ".control_points[0]");
  }
  throw std::runtime_error(
          what + ": this primitive cannot imply where the stroke starts; give the stroke a "
          "'start'");
}

/// Parse one primitive and advance `cursor` to where it leaves the pen.
Segment parseSegment(const YAML::Node & node, const std::string & what, Point2d & cursor)
{
  if (!node || !node.IsMap() || !node["type"]) {
    throw std::runtime_error(what + ": expected a map with a 'type'");
  }
  const std::string type = node["type"].as<std::string>();

  Segment segment;
  segment.counter_clockwise = parseCounterClockwise(node, what);

  if (type == "line") {
    segment.type = SegmentType::kLine;
    segment.to = asPoint2d(node["to"], what + ".to");
    cursor = segment.to;
    return segment;
  }

  if (type == "arc") {
    segment.type = SegmentType::kArc;
    if (node["start_angle"] || node["end_angle"]) {
      // Angle form: both endpoints are computed from the circle, so they land
      // on it exactly and the sampler's radius check cannot trip on a typo.
      if (!node["center"] || !node["radius"]) {
        throw std::runtime_error(what + ": an angle-defined arc needs 'center' and 'radius'");
      }
      if (!node["start_angle"] || !node["end_angle"]) {
        throw std::runtime_error(what + ": give both 'start_angle' and 'end_angle'");
      }
      const Point2d center = asPoint2d(node["center"], what + ".center");
      const double radius = node["radius"].as<double>();
      const Point2d begin = onCircle(center, radius, node["start_angle"].as<double>());
      if ((begin - cursor).norm() > kJoinTolerance) {
        throw std::runtime_error(
                what + ": start_angle puts the arc at (" + std::to_string(begin.x()) + ", " +
                std::to_string(begin.y()) + ") but the pen is at (" + std::to_string(cursor.x()) +
                ", " + std::to_string(cursor.y()) + ")");
      }
      segment.has_center = true;
      segment.center = center;
      segment.to = onCircle(center, radius, node["end_angle"].as<double>());
      cursor = segment.to;
      return segment;
    }

    // The shapes.yaml form: an explicit end point plus a centre or a radius.
    segment.to = asPoint2d(node["to"], what + ".to");
    if (node["center"]) {
      segment.has_center = true;
      segment.center = asPoint2d(node["center"], what + ".center");
    } else if (node["radius"]) {
      segment.radius = node["radius"].as<double>();
    } else {
      throw std::runtime_error(what + ": an arc needs 'center' or 'radius'");
    }
    segment.large_arc = node["large_arc"] && node["large_arc"].as<bool>();
    cursor = segment.to;
    return segment;
  }

  if (type == "circle") {
    segment.type = SegmentType::kCircle;
    if (!node["center"]) {
      throw std::runtime_error(what + ": a circle needs 'center'");
    }
    segment.has_center = true;
    segment.center = asPoint2d(node["center"], what + ".center");
    if (node["radius"] &&
      std::abs((cursor - segment.center).norm() - node["radius"].as<double>()) > kJoinTolerance)
    {
      throw std::runtime_error(what + ": the pen is not on the circle this segment describes");
    }
    return segment;  // a circle ends where it started, so the cursor stays put
  }

  if (type == "bspline") {
    segment.type = SegmentType::kBspline;
    if (!node["control_points"] || !node["control_points"].IsSequence() ||
      node["control_points"].size() < 2)
    {
      throw std::runtime_error(what + ": a bspline needs at least two 'control_points'");
    }
    for (std::size_t i = 0; i < node["control_points"].size(); ++i) {
      segment.control_points.push_back(
        asPoint2d(node["control_points"][i], what + ".control_points"));
    }
    segment.degree = static_cast<int>(optionalDouble(node, "degree", 3.0));
    segment.periodic = node["periodic"] && node["periodic"].as<bool>();
    if (!segment.periodic) {
      cursor = segment.control_points.back();
    }
    return segment;
  }

  throw std::runtime_error(what + ": unknown segment type '" + type + "'");
}

GlyphStroke parseStroke(const YAML::Node & node, const std::string & what)
{
  if (!node || !node.IsMap()) {
    throw std::runtime_error(what + ": expected a map with 'polyline' or 'segments'");
  }

  GlyphStroke stroke;
  stroke.closed = node["closed"] && node["closed"].as<bool>();

  if (node["polyline"]) {
    const YAML::Node & points = node["polyline"];
    if (!points.IsSequence() || points.size() < 2) {
      throw std::runtime_error(what + ".polyline: needs at least two points");
    }
    stroke.start = asPoint2d(points[0], what + ".polyline[0]");
    for (std::size_t i = 1; i < points.size(); ++i) {
      Segment segment;
      segment.type = SegmentType::kLine;
      segment.to = asPoint2d(points[i], what + ".polyline");
      stroke.segments.push_back(segment);
    }
    return stroke;
  }

  const YAML::Node & segments = node["segments"];
  if (!segments || !segments.IsSequence() || segments.size() == 0) {
    throw std::runtime_error(what + ": needs a 'polyline' or a non-empty 'segments'");
  }

  stroke.start = node["start"] ?
    asPoint2d(node["start"], what + ".start") :
    impliedStart(segments[0], what + ".segments[0]");

  Point2d cursor = stroke.start;
  for (std::size_t i = 0; i < segments.size(); ++i) {
    stroke.segments.push_back(
      parseSegment(segments[i], what + ".segments[" + std::to_string(i) + "]", cursor));
  }
  return stroke;
}

/// Ink extent of a stroke, measured by sampling it with the very code the
/// tracer uses. Doing it this way both gets curve bulges right and turns a
/// malformed segment into a load-time error naming the glyph.
void accumulateInk(
  const GlyphStroke & stroke, const SamplingSpec & sampling, Point2d & lo, Point2d & hi)
{
  std::vector<Point2d> points{stroke.start};
  for (const Segment & segment : stroke.segments) {
    sampleSegment(segment, points.back(), sampling, points);
  }
  for (const Point2d & point : points) {
    lo = lo.cwiseMin(point);
    hi = hi.cwiseMax(point);
  }
}

}  // namespace

const Glyph * Alphabet::find(char character) const
{
  char key = character;
  if (metrics.case_fold_upper) {
    key = static_cast<char>(std::toupper(static_cast<unsigned char>(key)));
  }
  const auto it = glyphs.find(key);
  return it == glyphs.end() ? nullptr : &it->second;
}

Alphabet loadAlphabet(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    throw std::runtime_error("cannot read alphabet file '" + path + "': " + e.what());
  }

  const YAML::Node font = root["font"];
  if (!font || !font.IsMap()) {
    throw std::runtime_error(path + ": missing the 'font' block");
  }
  const double cap_height = optionalDouble(font, "cap_height", 100.0);
  if (cap_height <= 0.0) {
    throw std::runtime_error(path + ": font.cap_height must be positive");
  }
  // Everything is authored in design units and stored in cap heights, so the
  // node can scale the whole font with one number.
  const double s = 1.0 / cap_height;
  const double default_advance = optionalDouble(font, "default_advance", 70.0);

  Alphabet alphabet;
  alphabet.name = font["name"] ? font["name"].as<std::string>() : "alphabet";
  alphabet.metrics.space_advance = optionalDouble(font, "space_advance", 46.0) * s;
  alphabet.metrics.default_advance = default_advance * s;
  alphabet.metrics.letter_spacing = optionalDouble(font, "letter_spacing", 16.0) * s;
  alphabet.metrics.line_spacing = optionalDouble(font, "line_spacing", 165.0) * s;
  alphabet.metrics.descender = optionalDouble(font, "descender", -16.0) * s;
  alphabet.metrics.case_fold_upper =
    !font["case_fold_upper"] || font["case_fold_upper"].as<bool>();

  const YAML::Node glyphs = root["glyphs"];
  if (!glyphs || !glyphs.IsMap() || glyphs.size() == 0) {
    throw std::runtime_error(path + ": 'glyphs' must be a non-empty map");
  }

  // Sampling used only to measure ink: fine enough that a bowl's widest point
  // is not missed, coarse enough to stay free.
  SamplingSpec measure;
  measure.max_segment_length = 0.02 * cap_height;
  measure.chord_tolerance = 0.001 * cap_height;
  measure.blend_distance = 0.0;

  for (const auto & entry : glyphs) {
    const std::string key = entry.first.as<std::string>();
    if (key.size() != 1) {
      throw std::runtime_error(path + ": glyph key '" + key + "' must be exactly one character");
    }
    const YAML::Node & node = entry.second;
    const std::string what = "glyph '" + key + "'";
    if (!node.IsMap() || !node["strokes"] || !node["strokes"].IsSequence()) {
      throw std::runtime_error(what + ": needs a 'strokes' sequence");
    }

    Glyph glyph;
    glyph.id = node["id"] ? node["id"].as<std::string>() : key;
    glyph.advance = optionalDouble(node, "advance", default_advance) * s;

    Point2d lo = Point2d::Constant(std::numeric_limits<double>::max());
    Point2d hi = Point2d::Constant(std::numeric_limits<double>::lowest());
    for (std::size_t i = 0; i < node["strokes"].size(); ++i) {
      const std::string stroke_what = what + " stroke " + std::to_string(i);
      GlyphStroke stroke = parseStroke(node["strokes"][i], stroke_what);
      try {
        accumulateInk(stroke, measure, lo, hi);
      } catch (const std::exception & e) {
        throw std::runtime_error(stroke_what + ": " + e.what());
      }
      // Scale to cap heights only after measuring, so the tolerances above stay
      // in the units the file was authored in.
      stroke.start *= s;
      for (Segment & segment : stroke.segments) {
        segment.to *= s;
        segment.center *= s;
        segment.radius *= s;
        for (Point2d & control : segment.control_points) {
          control *= s;
        }
      }
      glyph.strokes.push_back(std::move(stroke));
    }
    if (glyph.strokes.empty()) {
      throw std::runtime_error(what + ": has no strokes");
    }

    glyph.ink_min = lo * s;
    glyph.ink_max = hi * s;
    alphabet.glyphs.emplace(key[0], std::move(glyph));
  }

  if (alphabet.metrics.case_fold_upper) {
    for (char c = 'A'; c <= 'Z'; ++c) {
      if (alphabet.glyphs.find(c) == alphabet.glyphs.end()) {
        throw std::runtime_error(
                path + ": case folding is on but the font has no glyph for '" +
                std::string(1, c) + "'");
      }
    }
  }
  return alphabet;
}

}  // namespace avatar_challenge

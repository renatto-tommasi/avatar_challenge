// Copyright 2026 Renatto Tommasi
//
// Unit tests for the ShapeArray -> ShapeProgram conversion. The topic is the
// second way shapes enter the node, so the invariants the geometry code relies
// on have to hold for a message exactly as they do for YAML; several of these
// tests assert that by building the same shape both ways and comparing.

#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "avatar_challenge/shape_msg_conversion.hpp"
#include "avatar_challenge/yaml_loader.hpp"

using avatar_challenge::ProgramDefaults;
using avatar_challenge::SegmentType;
using avatar_challenge::ShapeProgram;
using avatar_challenge::ShapeSpec;
namespace msg = avatar_challenge::msg;

namespace
{

constexpr double kPi = 3.14159265358979323846;

ProgramDefaults testDefaults()
{
  ProgramDefaults defaults;
  defaults.reference_frame = "link_base";
  defaults.tool.standoff = 0.011;
  defaults.tool.spin_samples = 8;
  defaults.sampling.max_segment_length = 0.004;
  defaults.sampling.chord_tolerance = 0.0002;
  defaults.sampling.blend_distance = 0.008;
  defaults.velocity_scaling = 0.35;
  defaults.acceleration_scaling = 0.35;
  return defaults;
}

msg::Point2D point(double x, double y)
{
  msg::Point2D p;
  p.x = x;
  p.y = y;
  return p;
}

/// A 100 mm square, the message equivalent of the `square` shape in
/// config/shapes.yaml.
msg::Shape squareMsg()
{
  msg::Shape shape;
  shape.name = "square";
  shape.start.position.x = 0.35;
  shape.start.position.y = -0.12;
  shape.start.position.z = 0.35;
  // Rz(45 deg) * Ry(90 deg), as a quaternion.
  const double half_pitch = kPi / 4.0;
  const double half_yaw = kPi / 8.0;
  shape.start.orientation.x = -std::sin(half_pitch) * std::sin(half_yaw);
  shape.start.orientation.y = std::sin(half_pitch) * std::cos(half_yaw);
  shape.start.orientation.z = std::cos(half_pitch) * std::sin(half_yaw);
  shape.start.orientation.w = std::cos(half_pitch) * std::cos(half_yaw);
  shape.vertices = {point(0.0, 0.0), point(0.0, 0.1), point(0.1, 0.1), point(0.1, 0.0)};
  shape.closed = true;
  return shape;
}

std::string writeTempYaml(const std::string & contents)
{
  char path[] = "/tmp/avatar_challenge_msg_test_XXXXXX";
  const int fd = mkstemp(path);
  if (fd < 0) {
    ADD_FAILURE() << "could not create a temporary file";
    return {};
  }
  const ssize_t written = write(fd, contents.data(), contents.size());
  EXPECT_EQ(written, static_cast<ssize_t>(contents.size()));
  close(fd);
  return std::string(path);
}

}  // namespace

// A message and the YAML it mirrors must produce the same ShapeSpec. This is
// the property that keeps the two entry points from drifting apart.
TEST(ShapeMsgConversion, MatchesTheEquivalentYaml)
{
  const std::string yaml =
    "defaults:\n"
    "  units: {length: mm, angle: deg}\n"
    "shapes:\n"
    "  - name: square\n"
    "    start:\n"
    "      position: [350.0, -120.0, 350.0]\n"
    "      orientation_rpy: [0.0, 90.0, 45.0]\n"
    "    vertices: [[0.0, 0.0], [0.0, 100.0], [100.0, 100.0], [100.0, 0.0]]\n"
    "    closed: true\n";
  const std::string path = writeTempYaml(yaml);
  ASSERT_FALSE(path.empty());

  ProgramDefaults yaml_defaults = testDefaults();
  const ShapeProgram from_yaml = avatar_challenge::loadShapeProgram(path, yaml_defaults);
  std::remove(path.c_str());
  ASSERT_EQ(from_yaml.shapes.size(), 1u);

  const ProgramDefaults defaults = testDefaults();
  const ShapeSpec from_message = avatar_challenge::fromMsg(squareMsg(), defaults);

  const ShapeSpec & expected = from_yaml.shapes[0];
  EXPECT_EQ(from_message.name, expected.name);
  EXPECT_EQ(from_message.closed, expected.closed);
  EXPECT_EQ(from_message.segments.size(), expected.segments.size());
  EXPECT_TRUE(from_message.start_pose.translation().isApprox(expected.start_pose.translation()));
  EXPECT_TRUE(
    from_message.start_pose.linear().isApprox(expected.start_pose.linear(), 1e-9))
    << "message quaternion and YAML rpy disagree";
  for (std::size_t i = 0; i < expected.segments.size(); ++i) {
    EXPECT_TRUE(from_message.segments[i].to.isApprox(expected.segments[i].to)) << "segment " << i;
  }
}

TEST(ShapeMsgConversion, InheritsDefaultsWhenHasFlagsAreClear)
{
  const ProgramDefaults defaults = testDefaults();
  const ShapeSpec shape = avatar_challenge::fromMsg(squareMsg(), defaults);

  EXPECT_DOUBLE_EQ(shape.tool.standoff, defaults.tool.standoff);
  EXPECT_EQ(shape.tool.spin_samples, defaults.tool.spin_samples);
  EXPECT_DOUBLE_EQ(shape.sampling.blend_distance, defaults.sampling.blend_distance);
  EXPECT_EQ(shape.reference_frame, defaults.reference_frame);
  // Negative means "inherit" downstream; see ShapeSpec.
  EXPECT_LT(shape.velocity_scaling, 0.0);
}

// The reason has_tool / has_sampling exist rather than a sentinel: 0.0 is a
// meaningful value for both of these fields.
TEST(ShapeMsgConversion, ExplicitZeroOverridesTheDefault)
{
  msg::Shape message = squareMsg();
  message.has_tool = true;
  message.tool.standoff = 0.0;
  message.tool.face = msg::ToolSpec::FACE_INTO_PLANE;
  message.has_sampling = true;
  message.sampling.max_segment_length = 0.004;
  message.sampling.chord_tolerance = 0.0002;
  message.sampling.blend_distance = 0.0;

  const ShapeSpec shape = avatar_challenge::fromMsg(message, testDefaults());
  EXPECT_DOUBLE_EQ(shape.tool.standoff, 0.0);
  EXPECT_DOUBLE_EQ(shape.sampling.blend_distance, 0.0);
  EXPECT_DOUBLE_EQ(shape.tool.normal_sign, -1.0);
}

TEST(ShapeMsgConversion, FaceAlongNormalFlipsTheApproachAxis)
{
  msg::Shape message = squareMsg();
  message.has_tool = true;
  message.tool.face = msg::ToolSpec::FACE_ALONG_NORMAL;

  const ShapeSpec shape = avatar_challenge::fromMsg(message, testDefaults());
  EXPECT_DOUBLE_EQ(shape.tool.normal_sign, 1.0);
}

TEST(ShapeMsgConversion, ConvertsEveryPrimitive)
{
  msg::Shape message = squareMsg();
  message.vertices.clear();

  msg::Segment line;
  line.type = msg::Segment::LINE;
  line.to = point(0.09, 0.0);

  msg::Segment arc;
  arc.type = msg::Segment::ARC;
  arc.to = point(0.09, 0.07);
  arc.has_center = true;
  arc.center = point(0.09, 0.035);
  arc.counter_clockwise = true;

  msg::Segment circle;
  circle.type = msg::Segment::CIRCLE;
  circle.has_center = true;
  circle.center = point(0.0, 0.05);

  msg::Segment spline;
  spline.type = msg::Segment::BSPLINE;
  spline.degree = 3;
  spline.periodic = true;
  spline.control_points = {point(0.0, 0.0), point(0.09, -0.025), point(0.14, 0.055)};

  message.path = {line, arc, circle, spline};

  const ShapeSpec shape = avatar_challenge::fromMsg(message, testDefaults());
  ASSERT_EQ(shape.segments.size(), 4u);
  EXPECT_EQ(shape.segments[0].type, SegmentType::kLine);
  EXPECT_EQ(shape.segments[1].type, SegmentType::kArc);
  EXPECT_TRUE(shape.segments[1].has_center);
  EXPECT_EQ(shape.segments[2].type, SegmentType::kCircle);
  EXPECT_EQ(shape.segments[3].type, SegmentType::kBspline);
  EXPECT_EQ(shape.segments[3].control_points.size(), 3u);
  EXPECT_TRUE(shape.segments[3].periodic);
}

// degree 0 is what a default-constructed message carries; it must not reach the
// spline sampler as a literal zero.
TEST(ShapeMsgConversion, UnsetSplineDegreeFallsBackToCubic)
{
  msg::Shape message = squareMsg();
  message.vertices.clear();
  msg::Segment spline;
  spline.type = msg::Segment::BSPLINE;
  spline.control_points = {point(0.0, 0.0), point(0.05, 0.02), point(0.1, 0.0)};
  message.path = {spline};

  const ShapeSpec shape = avatar_challenge::fromMsg(message, testDefaults());
  ASSERT_EQ(shape.segments.size(), 1u);
  EXPECT_EQ(shape.segments[0].degree, 3);
}

TEST(ShapeMsgConversion, RejectsBothGeometryForms)
{
  msg::Shape message = squareMsg();
  msg::Segment line;
  line.type = msg::Segment::LINE;
  line.to = point(0.1, 0.0);
  message.path = {line};
  EXPECT_THROW(avatar_challenge::fromMsg(message, testDefaults()), std::runtime_error);
}

TEST(ShapeMsgConversion, RejectsNeitherGeometryForm)
{
  msg::Shape message = squareMsg();
  message.vertices.clear();
  EXPECT_THROW(avatar_challenge::fromMsg(message, testDefaults()), std::runtime_error);
}

TEST(ShapeMsgConversion, RejectsAFirstVertexAwayFromTheOrigin)
{
  msg::Shape message = squareMsg();
  message.vertices[0] = point(0.01, 0.0);
  EXPECT_THROW(avatar_challenge::fromMsg(message, testDefaults()), std::runtime_error);
}

TEST(ShapeMsgConversion, RejectsASingleVertex)
{
  msg::Shape message = squareMsg();
  message.vertices = {point(0.0, 0.0)};
  EXPECT_THROW(avatar_challenge::fromMsg(message, testDefaults()), std::runtime_error);
}

TEST(ShapeMsgConversion, RejectsAnArcWithNeitherCentreNorRadius)
{
  msg::Shape message = squareMsg();
  message.vertices.clear();
  msg::Segment arc;
  arc.type = msg::Segment::ARC;
  arc.to = point(0.05, 0.05);
  message.path = {arc};
  EXPECT_THROW(avatar_challenge::fromMsg(message, testDefaults()), std::runtime_error);
}

TEST(ShapeMsgConversion, RejectsACircleWithNoCentre)
{
  msg::Shape message = squareMsg();
  message.vertices.clear();
  msg::Segment circle;
  circle.type = msg::Segment::CIRCLE;
  message.path = {circle};
  EXPECT_THROW(avatar_challenge::fromMsg(message, testDefaults()), std::runtime_error);
}

// geometry_msgs/Quaternion defaults to w = 1, so a default-constructed Pose is
// fine. An all-zero quaternion still reaches us from producers that fill a Pose
// from a raw array, and it normalises to NaN — better a named error here than an
// unexplained IK failure later.
TEST(ShapeMsgConversion, RejectsAZeroQuaternion)
{
  msg::Shape message = squareMsg();
  message.start.orientation.x = 0.0;
  message.start.orientation.y = 0.0;
  message.start.orientation.z = 0.0;
  message.start.orientation.w = 0.0;
  EXPECT_THROW(avatar_challenge::fromMsg(message, testDefaults()), std::runtime_error);
}

TEST(ShapeMsgConversion, AcceptsADefaultConstructedOrientation)
{
  msg::Shape message = squareMsg();
  message.start.orientation = geometry_msgs::msg::Quaternion();
  const ShapeSpec shape = avatar_challenge::fromMsg(message, testDefaults());
  EXPECT_TRUE(shape.start_pose.linear().isApprox(Eigen::Matrix3d::Identity()));
}

// One malformed shape must not cost a producer the rest of its batch.
TEST(ShapeMsgConversion, BadShapesAreSkippedNotFatal)
{
  msg::Shape bad = squareMsg();
  bad.name = "bad";
  bad.vertices.clear();  // neither vertices nor path

  msg::ShapeArray array;
  array.mode = msg::ShapeArray::APPEND;
  array.shapes = {squareMsg(), bad, squareMsg()};

  std::vector<std::string> errors;
  const ShapeProgram program = avatar_challenge::fromMsg(array, testDefaults(), errors);
  EXPECT_EQ(program.shapes.size(), 2u);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors[0].find("bad"), std::string::npos)
    << "the error should name the shape it rejected: " << errors[0];
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

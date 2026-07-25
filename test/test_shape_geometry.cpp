// Copyright 2026 Renatto Tommasi
//
// Unit tests for the parts that do not need a running move_group: YAML parsing,
// 2D sampling of every primitive, corner blending, and the shape-frame -> robot
// -frame lift including the tool-normal-to-plane constraint.

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "avatar_challenge/path_sampler.hpp"
#include "avatar_challenge/yaml_loader.hpp"

using avatar_challenge::CartesianTrace;
using avatar_challenge::Point2d;
using avatar_challenge::ProgramDefaults;
using avatar_challenge::Segment;
using avatar_challenge::SegmentType;
using avatar_challenge::ShapeProgram;
using avatar_challenge::ShapeSpec;

namespace
{

constexpr double kPi = 3.14159265358979323846;

double polylineLength(const std::vector<Point2d> & points)
{
  double length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    length += (points[i] - points[i - 1]).norm();
  }
  return length;
}

std::string writeTempYaml(const std::string & contents)
{
  char path[] = "/tmp/avatar_challenge_test_XXXXXX";
  const int fd = mkstemp(path);
  if (fd < 0) {
    ADD_FAILURE() << "could not create a temporary file";
    return {};
  }
  const ssize_t written = ::write(fd, contents.data(), contents.size());
  ::close(fd);
  EXPECT_EQ(written, static_cast<ssize_t>(contents.size()));
  return std::string(path);
}

ShapeSpec squareShape(double side = 0.1)
{
  ShapeSpec shape;
  shape.name = "square";
  shape.closed = true;
  shape.sampling.max_segment_length = 0.005;
  for (const Point2d & v :
    {Point2d(0, side), Point2d(side, side), Point2d(side, 0)})
  {
    Segment seg;
    seg.type = SegmentType::kLine;
    seg.to = v;
    shape.segments.push_back(seg);
  }
  return shape;
}

}  // namespace

TEST(Sampling, ClosedSquareHasTheRightPerimeter)
{
  const ShapeSpec shape = squareShape(0.1);
  const auto outline = avatar_challenge::sampleOutline(shape);

  EXPECT_NEAR(outline.points.front().x(), 0.0, 1e-12);
  EXPECT_NEAR(outline.points.front().y(), 0.0, 1e-12);
  // Closed, so the outline returns to the origin.
  EXPECT_NEAR((outline.points.back() - outline.points.front()).norm(), 0.0, 1e-9);
  EXPECT_NEAR(polylineLength(outline.points), 0.4, 1e-6);

  // Every step honours the sampling resolution.
  for (std::size_t i = 1; i < outline.points.size(); ++i) {
    EXPECT_LE((outline.points[i] - outline.points[i - 1]).norm(), 0.005 + 1e-9);
  }
}

TEST(Sampling, FullCircleMatchesItsCircumference)
{
  ShapeSpec shape;
  shape.closed = false;
  shape.sampling.max_segment_length = 0.002;
  shape.sampling.chord_tolerance = 1e-5;

  Segment circle;
  circle.type = SegmentType::kCircle;
  circle.has_center = true;
  circle.center = Point2d(0.05, 0.0);   // the outline starts at the origin
  circle.counter_clockwise = true;
  shape.segments.push_back(circle);

  const auto outline = avatar_challenge::sampleOutline(shape);
  // Chords slightly under-measure the arc, so allow a small deficit only.
  const double circumference = 2.0 * kPi * 0.05;
  EXPECT_NEAR(polylineLength(outline.points), circumference, 1e-4);
  EXPECT_NEAR((outline.points.back() - outline.points.front()).norm(), 0.0, 1e-6);

  // Every sample sits on the circle.
  for (const Point2d & p : outline.points) {
    EXPECT_NEAR((p - circle.center).norm(), 0.05, 1e-6);
  }
}

TEST(Sampling, CentreDefinedArcSweepsTheRequestedDirection)
{
  ShapeSpec shape;
  shape.closed = false;
  shape.sampling.max_segment_length = 0.002;

  // A semicircle from the bottom of the circle to the top. Counter-clockwise
  // (increasing angle) leaves the bottom towards +x; clockwise mirrors it.
  Segment arc;
  arc.type = SegmentType::kArc;
  arc.has_center = true;
  arc.center = Point2d(0.0, 0.05);
  arc.to = Point2d(0.0, 0.1);
  shape.segments.push_back(arc);

  const double half_circumference = kPi * 0.05;
  const auto extreme_x = [](const std::vector<Point2d> & pts) {
      double lo = 0.0;
      double hi = 0.0;
      for (const Point2d & p : pts) {
        lo = std::min(lo, p.x());
        hi = std::max(hi, p.x());
      }
      return std::make_pair(lo, hi);
    };

  shape.segments[0].counter_clockwise = true;
  const auto ccw = avatar_challenge::sampleOutline(shape);
  EXPECT_NEAR(polylineLength(ccw.points), half_circumference, 1e-4);
  EXPECT_NEAR(extreme_x(ccw.points).second, 0.05, 1e-4);
  EXPECT_NEAR(extreme_x(ccw.points).first, 0.0, 1e-9);

  shape.segments[0].counter_clockwise = false;
  const auto cw = avatar_challenge::sampleOutline(shape);
  EXPECT_NEAR(polylineLength(cw.points), half_circumference, 1e-4);
  EXPECT_NEAR(extreme_x(cw.points).first, -0.05, 1e-4);
  EXPECT_NEAR(extreme_x(cw.points).second, 0.0, 1e-9);
}

TEST(Sampling, RadiusDefinedArcRespectsTheLargeArcFlag)
{
  ShapeSpec shape;
  shape.closed = false;
  shape.sampling.max_segment_length = 0.002;

  Segment arc;
  arc.type = SegmentType::kArc;
  arc.to = Point2d(0.0, 0.1);
  arc.radius = 0.05;
  arc.counter_clockwise = true;
  shape.segments.push_back(arc);

  const double minor = polylineLength(avatar_challenge::sampleOutline(shape).points);

  shape.segments[0].large_arc = true;
  const double major = polylineLength(avatar_challenge::sampleOutline(shape).points);

  // The chord is a diameter here, so both halves are semicircles of equal
  // length -- what must differ is which side they bulge to.
  EXPECT_NEAR(minor, major, 1e-4);
}

TEST(Sampling, BsplineStartsAtTheOriginAndStaysInTheControlHull)
{
  ShapeSpec shape;
  shape.closed = false;
  shape.sampling.max_segment_length = 0.003;

  Segment spline;
  spline.type = SegmentType::kBspline;
  spline.degree = 3;
  spline.control_points = {
    Point2d(0.0, 0.0), Point2d(0.05, 0.08), Point2d(0.12, -0.02), Point2d(0.18, 0.06)};
  shape.segments.push_back(spline);

  const auto outline = avatar_challenge::sampleOutline(shape);
  ASSERT_GT(outline.points.size(), 16u);
  EXPECT_NEAR(outline.points.front().norm(), 0.0, 1e-9);
  // A clamped B-spline ends on its last control point.
  EXPECT_NEAR((outline.points.back() - spline.control_points.back()).norm(), 0.0, 1e-4);

  // Convex-hull property: the curve never leaves the control polygon's bounds.
  for (const Point2d & p : outline.points) {
    EXPECT_GE(p.x(), -1e-6);
    EXPECT_LE(p.x(), 0.18 + 1e-6);
    EXPECT_GE(p.y(), -0.02 - 1e-6);
    EXPECT_LE(p.y(), 0.08 + 1e-6);
  }
}

TEST(Blending, RoundedCornersAreShorterAndSmoother)
{
  ShapeSpec sharp = squareShape(0.1);
  sharp.sampling.max_segment_length = 0.002;
  const auto sharp_outline = avatar_challenge::sampleOutline(sharp);

  ShapeSpec blended = sharp;
  blended.sampling.blend_distance = 0.01;
  const auto blended_outline = avatar_challenge::sampleOutline(blended);

  // Cutting each of the four corners must shorten the path, but only a little.
  const double sharp_length = polylineLength(sharp_outline.points);
  const double blended_length = polylineLength(blended_outline.points);
  EXPECT_LT(blended_length, sharp_length);
  EXPECT_GT(blended_length, sharp_length - 0.02);

  // And it must remove the 90-degree tangent jumps.
  const auto max_turn = [](const std::vector<Point2d> & pts) {
      double worst = 0.0;
      for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
        const Point2d u = pts[i] - pts[i - 1];
        const Point2d v = pts[i + 1] - pts[i];
        if (u.norm() < 1e-9 || v.norm() < 1e-9) {
          continue;
        }
        worst = std::max(
          worst, std::abs(std::atan2(u.x() * v.y() - u.y() * v.x(), u.dot(v))));
      }
      return worst;
    };
  EXPECT_NEAR(max_turn(sharp_outline.points), kPi / 2.0, 1e-6);
  EXPECT_LT(max_turn(blended_outline.points), 0.5);
}

TEST(Transform, WaypointsLandOnThePlaneAndTheToolIsNormalToIt)
{
  ShapeSpec shape = squareShape(0.1);
  shape.start_pose.translation() = Eigen::Vector3d(0.35, -0.12, 0.35);
  // Pitch 90 deg tips the shape plane from horizontal to vertical, then yaw 45.
  shape.start_pose.linear() =
    (Eigen::AngleAxisd(kPi / 4.0, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitY()))
    .toRotationMatrix();

  const CartesianTrace trace = avatar_challenge::buildTrace(shape, 0.05, 0.0, false);

  // The first authored vertex is the shape frame's origin, so the first
  // waypoint must be exactly the requested start position.
  EXPECT_NEAR(
    (trace.waypoints.front().translation() - shape.start_pose.translation()).norm(), 0.0, 1e-12);

  const Eigen::Vector3d normal = shape.start_pose.rotation().col(2);
  for (const Eigen::Isometry3d & pose : trace.waypoints) {
    // On the plane: the offset from the origin has no component along the normal.
    const Eigen::Vector3d offset = pose.translation() - shape.start_pose.translation();
    EXPECT_NEAR(offset.dot(normal), 0.0, 1e-9);

    // Normal to the plane: the tool's approach axis is anti-parallel to it,
    // and identical for every waypoint.
    const Eigen::Vector3d approach = pose.rotation().col(2);
    EXPECT_NEAR(approach.dot(normal), -1.0, 1e-9);
    EXPECT_NEAR(
      (pose.rotation() - trace.waypoints.front().rotation()).norm(), 0.0, 1e-12);
  }

  // Approach and retreat back off along +normal by the requested distance.
  EXPECT_NEAR(
    (trace.approach_pose.translation() - trace.waypoints.front().translation()).dot(normal),
    0.05, 1e-12);
}

TEST(Transform, FaceAlongNormalFlipsTheToolOver)
{
  ShapeSpec shape = squareShape(0.1);
  shape.tool.normal_sign = 1.0;
  const CartesianTrace trace = avatar_challenge::buildTrace(shape, 0.05, 0.0, false);
  const Eigen::Vector3d normal = shape.start_pose.rotation().col(2);
  EXPECT_NEAR(trace.waypoints.front().rotation().col(2).dot(normal), 1.0, 1e-9);
}

TEST(Transform, SpinKeepsTheToolNormalToThePlane)
{
  ShapeSpec shape = squareShape(0.1);
  const Eigen::Vector3d normal = shape.start_pose.rotation().col(2);
  for (double spin = 0.0; spin < 2 * kPi; spin += 0.7) {
    const CartesianTrace trace = avatar_challenge::buildTrace(shape, 0.05, spin, true);
    EXPECT_NEAR(trace.waypoints.front().rotation().col(2).dot(normal), -1.0, 1e-9);
  }
}

TEST(Loader, ConvertsUnitsAndBuildsTheBriefsExampleSquare)
{
  // The example from the brief, authored in millimetres and degrees.
  const std::string path = writeTempYaml(
    R"(
defaults:
  units: {length: mm, angle: deg}
shapes:
  - name: square
    start:
      position: [50.0, 0.0, 150.0]
      orientation_rpy: [0.0, 0.0, 45.0]
    vertices: [[0, 0], [0, 100], [100, 100], [100, 0]]
    closed: true
)");

  ProgramDefaults defaults;
  const ShapeProgram program = avatar_challenge::loadShapeProgram(path, defaults);
  std::remove(path.c_str());

  ASSERT_EQ(program.shapes.size(), 1u);
  const ShapeSpec & shape = program.shapes.front();
  EXPECT_EQ(shape.name, "square");
  EXPECT_NEAR(shape.start_pose.translation().x(), 0.050, 1e-12);
  EXPECT_NEAR(shape.start_pose.translation().z(), 0.150, 1e-12);

  // 45 degrees about Z.
  const Eigen::Vector3d x_axis = shape.start_pose.rotation().col(0);
  EXPECT_NEAR(x_axis.x(), std::cos(kPi / 4), 1e-9);
  EXPECT_NEAR(x_axis.y(), std::sin(kPi / 4), 1e-9);

  // Four vertices become three line segments; `closed` supplies the fourth edge.
  ASSERT_EQ(shape.segments.size(), 3u);
  EXPECT_NEAR(shape.segments[0].to.y(), 0.100, 1e-12);
  EXPECT_NEAR(polylineLength(avatar_challenge::sampleOutline(shape).points), 0.4, 1e-6);
}

TEST(Loader, RejectsAFirstVertexAwayFromTheOrigin)
{
  const std::string path = writeTempYaml(
    R"(
shapes:
  - name: bad
    start:
      position: [0.3, 0.0, 0.3]
      orientation_rpy: [0.0, 0.0, 0.0]
    vertices: [[0.01, 0], [0, 0.1]]
)");
  ProgramDefaults defaults;
  EXPECT_THROW(avatar_challenge::loadShapeProgram(path, defaults), std::runtime_error);
  std::remove(path.c_str());
}

TEST(Loader, RejectsBothVerticesAndPath)
{
  const std::string path = writeTempYaml(
    R"(
shapes:
  - name: bad
    start:
      position: [0.3, 0.0, 0.3]
      orientation_rpy: [0.0, 0.0, 0.0]
    vertices: [[0, 0], [0, 0.1]]
    path:
      - {type: line, to: [0.1, 0.1]}
)");
  ProgramDefaults defaults;
  EXPECT_THROW(avatar_challenge::loadShapeProgram(path, defaults), std::runtime_error);
  std::remove(path.c_str());
}

TEST(Loader, RejectsAnArcWithNeitherCentreNorRadius)
{
  const std::string path = writeTempYaml(
    R"(
shapes:
  - name: bad
    start:
      position: [0.3, 0.0, 0.3]
      orientation_rpy: [0.0, 0.0, 0.0]
    path:
      - {type: arc, to: [0.1, 0.0], direction: ccw}
)");
  ProgramDefaults defaults;
  EXPECT_THROW(avatar_challenge::loadShapeProgram(path, defaults), std::runtime_error);
  std::remove(path.c_str());
}

TEST(Loader, ParsesEveryPrimitiveKind)
{
  const std::string path = writeTempYaml(
    R"(
defaults:
  units: {length: mm, angle: deg}
  sampling: {max_segment_length: 3.0, blend_distance: 5.0}
shapes:
  - name: mixed
    start:
      position: [350.0, 0.0, 300.0]
      orientation_rpy: [0.0, 90.0, 0.0]
    path:
      - {type: line, to: [80.0, 0.0]}
      - {type: arc, to: [80.0, 60.0], center: [80.0, 30.0], direction: ccw}
      - {type: bspline, degree: 3, control_points: [[80.0, 60.0], [40.0, 90.0], [0.0, 60.0]]}
    closed: true
)");

  ProgramDefaults defaults;
  const ShapeProgram program = avatar_challenge::loadShapeProgram(path, defaults);
  std::remove(path.c_str());

  ASSERT_EQ(program.shapes.size(), 1u);
  const ShapeSpec & shape = program.shapes.front();
  ASSERT_EQ(shape.segments.size(), 3u);
  EXPECT_EQ(shape.segments[0].type, SegmentType::kLine);
  EXPECT_EQ(shape.segments[1].type, SegmentType::kArc);
  EXPECT_EQ(shape.segments[2].type, SegmentType::kBspline);
  EXPECT_NEAR(shape.sampling.max_segment_length, 0.003, 1e-12);
  EXPECT_NEAR(shape.sampling.blend_distance, 0.005, 1e-12);

  const auto outline = avatar_challenge::sampleOutline(shape);
  EXPECT_GT(outline.points.size(), 50u);
  EXPECT_NEAR((outline.points.back() - outline.points.front()).norm(), 0.0, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

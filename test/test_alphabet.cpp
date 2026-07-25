// Copyright 2026 Renatto Tommasi
//
// Unit tests for the font and the text layout behind the word_writer node.
//
// These run against the shipped config/alphabet.yaml rather than a fixture: a
// font whose arcs do not close, whose glyphs spill out of their advance or
// whose strokes are unsamplable is a bug worth catching here, not in RViz.

#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "avatar_challenge/alphabet.hpp"
#include "avatar_challenge/glyph_shapes.hpp"
#include "avatar_challenge/path_sampler.hpp"
#include "avatar_challenge/shape_msg_conversion.hpp"
#include "avatar_challenge/text_layout.hpp"

using avatar_challenge::Alphabet;
using avatar_challenge::Glyph;
using avatar_challenge::GlyphStroke;
using avatar_challenge::LayoutOptions;
using avatar_challenge::Point2d;
using avatar_challenge::SamplingSpec;
using avatar_challenge::Segment;
using avatar_challenge::TextLayout;
using avatar_challenge::WritingArea;
using avatar_challenge::glyphToShapes;
using avatar_challenge::layoutText;
using avatar_challenge::loadAlphabet;
using avatar_challenge::sampleSegment;

namespace
{

std::string writeTempYaml(const std::string & contents)
{
  char path[] = "/tmp/avatar_challenge_font_XXXXXX";
  const int fd = mkstemp(path);
  if (fd < 0) {
    throw std::runtime_error("cannot create a temporary file");
  }
  const ssize_t written = write(fd, contents.data(), contents.size());
  close(fd);
  if (written != static_cast<ssize_t>(contents.size())) {
    throw std::runtime_error("short write to the temporary file");
  }
  return std::string(path);
}

const Alphabet & font()
{
  static const Alphabet alphabet = loadAlphabet(ALPHABET_YAML);
  return alphabet;
}

std::vector<Point2d> samplePoints(const GlyphStroke & stroke)
{
  SamplingSpec sampling;
  sampling.max_segment_length = 0.01;
  sampling.chord_tolerance = 0.0005;
  sampling.blend_distance = 0.0;

  std::vector<Point2d> points{stroke.start};
  for (const Segment & segment : stroke.segments) {
    sampleSegment(segment, points.back(), sampling, points);
  }
  return points;
}

LayoutOptions options(double letter_height, const WritingArea & area)
{
  LayoutOptions out;
  out.letter_height = letter_height;
  out.area = area;
  return out;
}

}  // namespace

TEST(Alphabet, HasEveryLetterOfTheAlphabet)
{
  for (char c = 'A'; c <= 'Z'; ++c) {
    const Glyph * glyph = font().find(c);
    ASSERT_NE(glyph, nullptr) << "no glyph for '" << c << "'";
    EXPECT_FALSE(glyph->strokes.empty()) << "'" << c << "' has no strokes";
    EXPECT_GT(glyph->advance, 0.0) << "'" << c << "' has no width";
  }
}

TEST(Alphabet, FoldsCaseAndRejectsUnknownCharacters)
{
  EXPECT_EQ(font().find('h'), font().find('H'));
  EXPECT_EQ(font().find('#'), nullptr);
  // Space is spacing, not a glyph.
  EXPECT_EQ(font().find(' '), nullptr);
  EXPECT_NE(font().find('7'), nullptr);
}

TEST(Alphabet, GlyphsSitInTheirDesignBox)
{
  for (const auto & entry : font().glyphs) {
    const std::string name(1, entry.first);
    const Glyph & glyph = entry.second;
    // Cap height is 1 by construction, and the loader normalises to it.
    EXPECT_GE(glyph.ink_min.x(), -1e-6) << name << " starts left of its origin";
    EXPECT_LE(glyph.ink_max.y(), 1.0 + 1e-6) << name << " pokes above the cap line";
    EXPECT_GE(glyph.ink_min.y(), font().metrics.descender - 1e-6)
      << name << " drops below the font's descender";
    // A glyph may kern slightly past its advance, but not by much, or the
    // spacing the layout computes stops meaning anything.
    EXPECT_LE(glyph.ink_max.x(), glyph.advance + 0.06) << name << " overflows its advance";
  }
}

TEST(Alphabet, CapitalsReachTheCapLineAndTheBaseline)
{
  for (char c = 'A'; c <= 'Z'; ++c) {
    const Glyph * glyph = font().find(c);
    const std::string name(1, c);
    EXPECT_NEAR(glyph->ink_max.y(), 1.0, 0.06) << name << " does not reach the cap line";
    // J and Q are the only ones allowed to hang, and neither does here.
    EXPECT_NEAR(glyph->ink_min.y(), 0.0, 0.06) << name << " does not sit on the baseline";
  }
}

TEST(Alphabet, EveryStrokeIsSamplable)
{
  for (const auto & entry : font().glyphs) {
    for (const GlyphStroke & stroke : entry.second.strokes) {
      const std::vector<Point2d> points = samplePoints(stroke);
      EXPECT_GE(points.size(), 2u) << "glyph '" << entry.first << "' has a stroke with no length";
      for (const Point2d & point : points) {
        EXPECT_TRUE(std::isfinite(point.x()) && std::isfinite(point.y()));
      }
    }
  }
}

TEST(Alphabet, RejectsAStrokeWhoseArcDoesNotStartWhereThePenIs)
{
  // The whole point of the angle-defined arc syntax is that a stroke cannot
  // silently jump: an arc that starts somewhere else is a broken font, not a
  // pen move.
  const std::string path = writeTempYaml(
    R"(font: {cap_height: 100.0}
glyphs:
  "A":
    advance: 50.0
    strokes:
      - start: [0.0, 0.0]
        segments:
          - {type: line, to: [10.0, 0.0]}
          - {type: arc, center: [50.0, 50.0], radius: 50.0,
             start_angle: 180.0, end_angle: 90.0, direction: ccw}
)");
  EXPECT_THROW(loadAlphabet(path), std::runtime_error);
  std::remove(path.c_str());
}

TEST(Alphabet, RejectsAFontMissingALetter)
{
  const std::string path = writeTempYaml(
    R"(font: {cap_height: 100.0, case_fold_upper: true}
glyphs:
  "A":
    advance: 50.0
    strokes:
      - polyline: [[0.0, 0.0], [50.0, 100.0]]
)");
  EXPECT_THROW(loadAlphabet(path), std::runtime_error);
  std::remove(path.c_str());
}

TEST(Layout, PutsLettersInReadingOrderOnOneLine)
{
  WritingArea area;
  area.u_min = -0.3;
  area.u_max = 0.3;
  area.v_min = 0.1;
  area.v_max = 0.5;

  const TextLayout layout = layoutText("HI", font(), options(0.06, area));
  ASSERT_EQ(layout.glyphs.size(), 2u);
  EXPECT_EQ(layout.glyphs[0].character, 'H');
  EXPECT_EQ(layout.glyphs[1].character, 'I');
  EXPECT_LT(layout.glyphs[0].u, layout.glyphs[1].u);
  EXPECT_DOUBLE_EQ(layout.glyphs[0].v, layout.glyphs[1].v);
  EXPECT_EQ(layout.lines, 1u);
  // The first baseline leaves exactly one cap height of headroom.
  EXPECT_NEAR(layout.glyphs[0].v, area.v_max - 0.06, 1e-9);
  EXPECT_NEAR(layout.glyphs[0].u, area.u_min, 1e-9);
}

TEST(Layout, WrapsWholeWordsToTheNextLine)
{
  WritingArea area;
  area.u_min = 0.0;
  area.u_max = 0.30;
  area.v_min = 0.0;
  area.v_max = 0.60;

  const TextLayout layout = layoutText("HELLO WORLD", font(), options(0.05, area));
  ASSERT_FALSE(layout.glyphs.empty());
  EXPECT_EQ(layout.lines, 2u);
  EXPECT_EQ(layout.dropped, 0u);
  EXPECT_FALSE(layout.truncated);

  // The word moved intact: every letter of HELLO on one line, WORLD on the next.
  for (std::size_t i = 0; i < layout.glyphs.size(); ++i) {
    EXPECT_EQ(layout.glyphs[i].line, i < 5 ? 0u : 1u) << "letter " << i;
  }
  // Each line starts back at the left margin and drops by the line spacing.
  EXPECT_NEAR(layout.glyphs[5].u, area.u_min, 1e-9);
  EXPECT_NEAR(layout.glyphs[0].v - layout.glyphs[5].v, layout.line_spacing, 1e-9);
}

TEST(Layout, KeepsEveryLetterInsideTheArea)
{
  WritingArea area;
  area.u_min = -0.25;
  area.u_max = 0.25;
  area.v_min = 0.10;
  area.v_max = 0.45;

  const TextLayout layout =
    layoutText("THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG", font(), options(0.04, area));
  ASSERT_FALSE(layout.glyphs.empty());
  for (const auto & placed : layout.glyphs) {
    const double right = placed.u + placed.glyph->ink_max.x() * placed.scale;
    const double bottom = placed.v + placed.glyph->ink_min.y() * placed.scale;
    const double top = placed.v + placed.glyph->ink_max.y() * placed.scale;
    EXPECT_GE(placed.u, area.u_min - 1e-9) << placed.character;
    EXPECT_LE(right, area.u_max + 1e-9) << placed.character;
    EXPECT_GE(bottom, area.v_min - 1e-9) << placed.character;
    EXPECT_LE(top, area.v_max + 1e-9) << placed.character;
  }
}

TEST(Layout, BreaksAWordTooLongForOneLine)
{
  WritingArea area;
  area.u_min = 0.0;
  area.u_max = 0.12;
  area.v_min = 0.0;
  area.v_max = 0.60;

  const TextLayout layout = layoutText("ABCDEFGH", font(), options(0.05, area));
  EXPECT_EQ(layout.glyphs.size(), 8u);
  EXPECT_GT(layout.lines, 1u);
  EXPECT_EQ(layout.dropped, 0u);
}

TEST(Layout, HonoursExplicitLineBreaksAndDropsWhatDoesNotFit)
{
  WritingArea area;
  area.u_min = 0.0;
  area.u_max = 0.40;
  area.v_min = 0.30;
  area.v_max = 0.40;   // one cap height of room, so one line only

  const TextLayout layout = layoutText("AB\nCD", font(), options(0.06, area));
  EXPECT_EQ(layout.lines, 1u);
  EXPECT_EQ(layout.glyphs.size(), 2u);
  EXPECT_EQ(layout.dropped, 2u);
  EXPECT_TRUE(layout.truncated);
}

TEST(GlyphShapes, TheTracerAcceptsEveryGlyphOfTheFont)
{
  // The tracer validates what arrives on its topic with fromMsg(); running the
  // whole font through it here is what says "these letters will be drawn"
  // rather than "these letters were published".
  WritingArea area;
  area.u_min = -0.30;
  area.u_max = 0.30;
  area.v_min = 0.10;
  area.v_max = 0.50;

  std::string every;
  for (const auto & entry : font().glyphs) {
    every.push_back(entry.first);
  }
  const TextLayout layout = layoutText(every, font(), options(0.03, area));
  ASSERT_EQ(layout.glyphs.size(), font().glyphs.size());

  avatar_challenge::WritingPlane plane;
  plane.distance = 0.42;
  avatar_challenge::StrokeStyle style;
  style.tool.face = avatar_challenge::msg::ToolSpec::FACE_INTO_PLANE;
  style.sampling.max_segment_length = 0.002;
  style.sampling.chord_tolerance = 0.0001;
  style.sampling.blend_distance = 0.0;

  avatar_challenge::ProgramDefaults defaults;
  std::size_t shapes = 0;
  for (const auto & placed : layout.glyphs) {
    const auto batch = glyphToShapes(
      placed, plane, style, std::string("g_") + placed.glyph->id,
      avatar_challenge::msg::ShapeArray::APPEND);
    ASSERT_FALSE(batch.shapes.empty()) << placed.character;

    std::vector<std::string> errors;
    const auto program = avatar_challenge::fromMsg(batch, defaults, errors);
    EXPECT_TRUE(errors.empty()) << placed.character << ": " << (errors.empty() ? "" : errors[0]);
    ASSERT_EQ(program.shapes.size(), batch.shapes.size()) << placed.character;
    shapes += program.shapes.size();

    for (const auto & shape : program.shapes) {
      const auto trace = avatar_challenge::buildTrace(shape, 0.05, 0.0, false);
      ASSERT_FALSE(trace.waypoints.empty()) << placed.character;
      for (const auto & waypoint : trace.waypoints) {
        // The requirement, checked where it ends up: every point of every
        // letter sits on one plane parallel to the base YZ plane.
        EXPECT_NEAR(waypoint.translation().x(), plane.distance, 1e-9) << placed.character;
      }
      // "Into the plane" means the tool's approach axis points away from the
      // robot, along +X, for the whole letter.
      const Eigen::Vector3d approach =
        trace.tool_orientation * Eigen::Vector3d::UnitZ();
      EXPECT_NEAR(approach.x(), 1.0, 1e-9) << placed.character;
    }
  }
  EXPECT_GT(shapes, font().glyphs.size());
}

TEST(GlyphShapes, PlacesTheLetterWhereTheLayoutPutIt)
{
  WritingArea area;
  area.u_min = 0.0;
  area.u_max = 0.5;
  area.v_min = 0.1;
  area.v_max = 0.5;
  const TextLayout layout = layoutText("L", font(), options(0.06, area));
  ASSERT_EQ(layout.glyphs.size(), 1u);

  avatar_challenge::WritingPlane plane;
  plane.distance = 0.42;
  const avatar_challenge::StrokeStyle style;
  const auto batch = glyphToShapes(
    layout.glyphs[0], plane, style, "L", avatar_challenge::msg::ShapeArray::APPEND);
  ASSERT_EQ(batch.shapes.size(), 1u);

  // L starts at the top left of its box: u = area.u_min maps to y = -u, and the
  // cap line sits one cap height above the baseline.
  const auto & start = batch.shapes[0].start.position;
  EXPECT_NEAR(start.x, 0.42, 1e-12);
  EXPECT_NEAR(start.y, -area.u_min, 1e-12);
  EXPECT_NEAR(start.z, area.v_max, 1e-12);
  EXPECT_EQ(batch.shapes[0].name, "L_s0");
  EXPECT_TRUE(batch.shapes[0].vertices.empty());
  EXPECT_FALSE(batch.shapes[0].path.empty());
}

TEST(Layout, ReportsUnknownCharactersInsteadOfDrawingThem)
{
  WritingArea area;
  const TextLayout layout = layoutText("A#B", font(), options(0.05, area));
  EXPECT_EQ(layout.glyphs.size(), 2u);
  EXPECT_EQ(layout.unknown, "#");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

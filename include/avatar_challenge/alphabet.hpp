// Copyright 2026 Renatto Tommasi
//
// YAML -> Alphabet. See config/alphabet.yaml for the documented schema.
//
// A glyph is a small set of pen-down strokes drawn in a design box whose
// baseline is y = 0 and whose cap line is y = 1 after normalisation, with x
// growing to the right. Nothing here knows about the robot, the writing plane
// or metres: a glyph is pure 2D, exactly like a ShapeSpec is, which is what
// lets one stroke become one shape with no geometry work in between.

#ifndef AVATAR_CHALLENGE__ALPHABET_HPP_
#define AVATAR_CHALLENGE__ALPHABET_HPP_

#include <map>
#include <string>
#include <vector>

#include "avatar_challenge/shape_spec.hpp"

namespace avatar_challenge
{

/// One pen-down outline of a glyph. Coordinates are absolute in the glyph's own
/// design box (cap heights); the shape frame origin a Shape message needs is
/// `start`, and the node subtracts it when it builds the message.
struct GlyphStroke
{
  Point2d start{Point2d::Zero()};
  std::vector<Segment> segments;
  bool closed{false};
};

/// One character's worth of strokes plus the metrics the layout needs.
struct Glyph
{
  /// Safe identifier used in published shape names — "A", or "period" for ".".
  std::string id;

  /// Width of the glyph in cap heights. The gap to the next glyph is the font's
  /// letter_spacing, so a glyph carries no side bearings of its own.
  double advance{0.7};

  std::vector<GlyphStroke> strokes;

  /// Ink bounding box, measured by sampling the strokes at load time rather
  /// than guessed from the control points, so a bowl's bulge is included.
  Point2d ink_min{Point2d::Zero()};
  Point2d ink_max{Point2d::Zero()};
};

/// Font-wide metrics, all in cap heights.
struct FontMetrics
{
  double space_advance{0.46};
  double default_advance{0.70};
  double letter_spacing{0.16};
  double line_spacing{1.65};
  /// Lowest ink in the font (negative). Used to decide whether a line still
  /// clears the bottom of the writing area.
  double descender{-0.16};
  /// Fold lower-case input onto the upper-case glyphs.
  bool case_fold_upper{true};
};

struct Alphabet
{
  std::string name{"alphabet"};
  FontMetrics metrics;
  std::map<char, Glyph> glyphs;

  /// Look a character up, applying case folding. Null when the font has no
  /// glyph for it. Space is not a glyph — see FontMetrics::space_advance.
  const Glyph * find(char character) const;
};

/// Parse a font file. Throws std::runtime_error naming the offending glyph and
/// stroke on any schema or geometry problem — including an arc whose endpoints
/// do not lie on its circle, which would otherwise only surface much later as a
/// rejected shape message.
Alphabet loadAlphabet(const std::string & path);

}  // namespace avatar_challenge

#endif  // AVATAR_CHALLENGE__ALPHABET_HPP_

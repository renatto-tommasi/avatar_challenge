// Copyright 2026 Renatto Tommasi
//
// Text -> glyph placements, in the 2D coordinates of the writing plane.
//
// The layout works in plane coordinates (u, v): u runs along the writing
// direction, v points up, and the baseline of the first line sits one cap
// height below the top of the writing area. Both are metres, and both are
// absolute — the node fixes the writing area from the robot's reachable
// workspace, so "does this letter fit" and "can the arm get there" are the same
// question, asked once, here.

#ifndef AVATAR_CHALLENGE__TEXT_LAYOUT_HPP_
#define AVATAR_CHALLENGE__TEXT_LAYOUT_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "avatar_challenge/alphabet.hpp"

namespace avatar_challenge
{

/// The rectangle on the writing plane that text is allowed to occupy, metres.
struct WritingArea
{
  double u_min{-0.30};
  double u_max{0.30};
  double v_min{0.15};
  double v_max{0.50};

  double width() const {return u_max - u_min;}
  double height() const {return v_max - v_min;}
};

/// Everything the layout needs beyond the font itself. The three spacings are
/// in metres; a negative value means "take the font's own metric, scaled".
struct LayoutOptions
{
  /// Metres per cap height — the size of a capital letter.
  double letter_height{0.06};
  double letter_spacing{-1.0};
  double word_spacing{-1.0};
  double line_spacing{-1.0};
  WritingArea area;
};

/// One glyph, placed. `origin` is the glyph's design origin — the point on the
/// baseline at its left edge — in plane coordinates, and `scale` converts the
/// glyph's cap-height units to metres.
struct PlacedGlyph
{
  const Glyph * glyph{nullptr};
  char character{'\0'};
  double u{0.0};
  double v{0.0};
  double scale{1.0};
  std::size_t line{0};
};

struct TextLayout
{
  std::vector<PlacedGlyph> glyphs;

  /// Lines actually used.
  std::size_t lines{0};

  /// Characters the font had no glyph for, in the order they were met.
  std::string unknown;

  /// Glyphs left undrawn because the text ran off the bottom of the area.
  std::size_t dropped{0};
  bool truncated{false};

  /// Resolved metrics, so the caller can report and reuse them.
  double letter_spacing{0.0};
  double word_spacing{0.0};
  double line_spacing{0.0};
};

/// Lay `text` out in the area, wrapping at word boundaries and breaking a word
/// that is too long for one line. '\n' forces a line break.
TextLayout layoutText(
  const std::string & text, const Alphabet & alphabet, const LayoutOptions & options);

}  // namespace avatar_challenge

#endif  // AVATAR_CHALLENGE__TEXT_LAYOUT_HPP_

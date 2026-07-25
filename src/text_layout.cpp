// Copyright 2026 Renatto Tommasi

#include "avatar_challenge/text_layout.hpp"

#include <cctype>
#include <cstddef>
#include <string>
#include <vector>

namespace avatar_challenge
{
namespace
{

constexpr double kEps = 1e-9;

/// One item of the source text after glyph lookup: a glyph to draw, a word
/// break, or a forced line break.
struct Item
{
  enum class Kind { kGlyph, kSpace, kNewline } kind{Kind::kGlyph};
  const Glyph * glyph{nullptr};
  char character{'\0'};
};

std::vector<Item> tokenize(const std::string & text, const Alphabet & alphabet, std::string & unknown)
{
  std::vector<Item> items;
  items.reserve(text.size());
  for (const char character : text) {
    if (character == '\n' || character == '\r') {
      items.push_back(Item{Item::Kind::kNewline, nullptr, character});
      continue;
    }
    if (std::isspace(static_cast<unsigned char>(character))) {
      items.push_back(Item{Item::Kind::kSpace, nullptr, character});
      continue;
    }
    const Glyph * glyph = alphabet.find(character);
    if (glyph == nullptr) {
      unknown.push_back(character);
      continue;
    }
    items.push_back(Item{Item::Kind::kGlyph, glyph, character});
  }
  return items;
}

}  // namespace

TextLayout layoutText(
  const std::string & text, const Alphabet & alphabet, const LayoutOptions & options)
{
  const FontMetrics & metrics = alphabet.metrics;
  const double scale = options.letter_height;

  TextLayout layout;
  layout.letter_spacing = options.letter_spacing >= 0.0 ?
    options.letter_spacing : metrics.letter_spacing * scale;
  layout.word_spacing = options.word_spacing >= 0.0 ?
    options.word_spacing : metrics.space_advance * scale;
  layout.line_spacing = options.line_spacing >= 0.0 ?
    options.line_spacing : metrics.line_spacing * scale;

  const std::vector<Item> items = tokenize(text, alphabet, layout.unknown);
  const WritingArea & area = options.area;

  // A line fits while its baseline leaves room for the font's descender above
  // the floor of the area — the Q tail and the comma are the reason that is not
  // simply the baseline itself.
  const double descender = metrics.descender * scale;
  const auto baseline = [&](std::size_t line) {
      return area.v_max - scale - static_cast<double>(line) * layout.line_spacing;
    };
  const auto line_fits = [&](std::size_t line) {
      return baseline(line) + descender >= area.v_min - kEps;
    };

  std::size_t line = 0;
  double cursor = area.u_min;
  bool at_line_start = true;

  const auto break_line = [&]() {
      ++line;
      cursor = area.u_min;
      at_line_start = true;
    };

  for (std::size_t i = 0; i < items.size(); ++i) {
    const Item & item = items[i];

    if (item.kind == Item::Kind::kNewline) {
      break_line();
      if (!line_fits(line)) {
        layout.truncated = true;
      }
      continue;
    }
    if (item.kind == Item::Kind::kSpace) {
      // A space at the start of a line would push the text off its own left
      // margin, so it is swallowed instead.
      if (!at_line_start) {
        cursor += layout.word_spacing;
      }
      continue;
    }

    // Measure the whole word this glyph opens, so it can be moved to the next
    // line intact instead of being split at the margin.
    if (i == 0 || items[i - 1].kind != Item::Kind::kGlyph) {
      double word_width = 0.0;
      std::size_t count = 0;
      for (std::size_t j = i; j < items.size() && items[j].kind == Item::Kind::kGlyph; ++j) {
        word_width += items[j].glyph->advance * scale;
        ++count;
      }
      if (count > 1) {
        word_width += layout.letter_spacing * static_cast<double>(count - 1);
      }
      if (!at_line_start && cursor + word_width > area.u_max + kEps) {
        break_line();
      }
    }

    const double advance = item.glyph->advance * scale;
    // A word longer than the whole line still has to go somewhere: break it at
    // the letter that overflows. The `!at_line_start` guard is what keeps a
    // single oversized glyph from looping forever.
    if (!at_line_start && cursor + advance > area.u_max + kEps) {
      break_line();
    }

    if (!line_fits(line)) {
      layout.truncated = true;
      ++layout.dropped;
      continue;
    }

    PlacedGlyph placed;
    placed.glyph = item.glyph;
    placed.character = item.character;
    placed.u = cursor;
    placed.v = baseline(line);
    placed.scale = scale;
    placed.line = line;
    layout.glyphs.push_back(placed);

    cursor += advance + layout.letter_spacing;
    at_line_start = false;
    layout.lines = line + 1;
  }

  return layout;
}

}  // namespace avatar_challenge

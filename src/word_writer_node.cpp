// Copyright 2026 Renatto Tommasi
//
// word_writer: turn a string into letters for the shape tracer to draw.
//
// Subscribes to a std_msgs/String on ~/word, lays the text out on a writing
// plane parallel to the robot's YZ plane, and publishes one
// avatar_challenge/msg/ShapeArray per letter on the tracer's ~/add_shapes.
//
// Why one message per letter
// --------------------------
// A Shape is a single pen-down outline, and most capitals are not: E is three
// strokes, B is three, A is two. So a letter is a *batch* — every stroke of it
// in one ShapeArray — and the letters go out one after another, in reading
// order. The tracer queues batches and draws them in the order they arrive, so
// that ordering is what puts the word on the plane left to right.
//
// The writing plane
// -----------------
// The plane is x = `plane_distance` in the robot base frame, i.e. parallel to
// the base YZ plane, with the plane normal along X. Plane coordinates (u, v)
// are u along the writing direction and v up:
//
//     p_base = (plane_distance, -u, v)          (mirror_text flips u's sign)
//
// so u = -y and v = z. Reading left to right for someone standing behind the
// robot and looking out along +X means u runs along -Y, which is where the
// minus sign comes from. The shape frame handed to the tracer has X_S along u,
// Y_S along v and therefore Z_S = -X_base, so a tool that faces "into the
// plane" points away from the robot, like a pen held against a whiteboard in
// front of it.

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "avatar_challenge/alphabet.hpp"
#include "avatar_challenge/glyph_shapes.hpp"
#include "avatar_challenge/msg/shape_array.hpp"
#include "avatar_challenge/text_layout.hpp"

namespace avatar_challenge
{
namespace
{

/// Fractions of the reachable disc's radius used for the inscribed writing
/// rectangle when the area is derived rather than given. A rectangle inscribed
/// in a circle can trade width for height; text wants width, so this one is
/// 1.73 R wide and 1.0 R tall (the corners sit at +-30 degrees).
constexpr double kAreaHalfWidthFraction = 0.866;
constexpr double kAreaHalfHeightFraction = 0.5;

std::string sanitize(const std::string & text)
{
  std::string out;
  out.reserve(text.size());
  for (const char character : text) {
    out.push_back(std::isalnum(static_cast<unsigned char>(character)) ? character : '_');
  }
  return out;
}

}  // namespace

class WordWriter
{
public:
  explicit WordWriter(const rclcpp::Node::SharedPtr & node)
  : node_(node), logger_(node->get_logger())
  {
    declareParameters();
    alphabet_ = loadAlphabet(alphabetPath());
    RCLCPP_INFO(
      logger_, "Loaded font '%s' with %zu glyphs from %s", alphabet_.name.c_str(),
      alphabet_.glyphs.size(), alphabetPath().c_str());

    resolveWritingArea();

    // Same QoS as the tracer's subscription: RELIABLE + TRANSIENT_LOCAL. The
    // default VOLATILE profile is incompatible with it and nothing would be
    // delivered — see the README.
    publisher_ = node_->create_publisher<msg::ShapeArray>(
      node_->get_parameter("shapes_topic").as_string(),
      rclcpp::QoS(10).reliable().transient_local());

    subscription_ = node_->create_subscription<std_msgs::msg::String>(
      node_->get_parameter("word_topic").as_string(), rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr message) {onWord(message->data);});

    const double interval = node_->get_parameter("letter_interval").as_double();
    timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(std::max(0.01, interval)), [this]() {publishNext();});

    RCLCPP_INFO(
      logger_, "Listening for words on '%s'; letters go out on '%s'",
      subscription_->get_topic_name(), publisher_->get_topic_name());

    const std::string initial = node_->get_parameter("initial_word").as_string();
    if (!initial.empty()) {
      const double delay = node_->get_parameter("initial_delay").as_double();
      initial_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(std::max(0.0, delay)), [this, initial]() {
          initial_timer_->cancel();
          onWord(initial);
        });
    }
  }

private:
  void declareParameters()
  {
    node_->declare_parameter<std::string>("alphabet_file", "");
    node_->declare_parameter<std::string>("shapes_topic", "/shape_tracer/add_shapes");
    node_->declare_parameter<std::string>("word_topic", "~/word");
    node_->declare_parameter<std::string>("frame_id", "link_base");

    // --- the writing plane and the type on it ---------------------------
    node_->declare_parameter<double>("plane_distance", 0.42);
    node_->declare_parameter<bool>("mirror_text", false);
    node_->declare_parameter<double>("letter_height", 0.06);
    node_->declare_parameter<double>("letter_spacing", -1.0);
    node_->declare_parameter<double>("word_spacing", -1.0);
    node_->declare_parameter<double>("line_spacing", -1.0);

    // --- what the arm may reach on that plane ---------------------------
    node_->declare_parameter<bool>("area.derive_from_reach", true);
    node_->declare_parameter<double>("area.u_min", -0.30);
    node_->declare_parameter<double>("area.u_max", 0.30);
    node_->declare_parameter<double>("area.v_min", 0.15);
    node_->declare_parameter<double>("area.v_max", 0.50);
    node_->declare_parameter<double>("workspace.max_reach", 0.70);
    node_->declare_parameter<double>("workspace.min_reach", 0.25);
    node_->declare_parameter<double>("workspace.shoulder_height", 0.267);
    node_->declare_parameter<double>("workspace.margin", 0.10);
    node_->declare_parameter<double>("workspace.min_height", 0.15);
    node_->declare_parameter<bool>("workspace.check_glyphs", true);

    // --- how the shapes are drawn ---------------------------------------
    node_->declare_parameter<double>("tool.standoff", 0.0);
    node_->declare_parameter<double>("tool.spin", 0.0);
    node_->declare_parameter<bool>("tool.free_spin", false);
    node_->declare_parameter<double>("sampling.max_segment_length", 0.002);
    node_->declare_parameter<double>("sampling.chord_tolerance", 0.0001);
    node_->declare_parameter<double>("sampling.blend_distance", 0.0);
    node_->declare_parameter<double>("velocity_scaling", 0.0);
    node_->declare_parameter<double>("acceleration_scaling", 0.0);

    // --- pacing ----------------------------------------------------------
    node_->declare_parameter<double>("letter_interval", 0.5);
    node_->declare_parameter<bool>("replace_previous", true);
    node_->declare_parameter<std::string>("initial_word", "");
    node_->declare_parameter<double>("initial_delay", 3.0);
  }

  std::string alphabetPath() const
  {
    const std::string configured = node_->get_parameter("alphabet_file").as_string();
    if (!configured.empty()) {
      return configured;
    }
    return ament_index_cpp::get_package_share_directory("avatar_challenge") +
           "/config/alphabet.yaml";
  }

  /// Distance from the shoulder to a point on the writing plane, and the disc
  /// of the plane the arm can cover. The model is deliberately crude — a sphere
  /// about the shoulder, shrunk by a margin — because it only has to keep the
  /// layout inside the arm's envelope; the tracer's redundancy search is what
  /// decides whether a particular pose is actually solvable.
  double reachRadiusOnPlane() const
  {
    const double reach = node_->get_parameter("workspace.max_reach").as_double() -
      node_->get_parameter("workspace.margin").as_double();
    const double distance = node_->get_parameter("plane_distance").as_double();
    const double squared = reach * reach - distance * distance;
    return squared > 0.0 ? std::sqrt(squared) : 0.0;
  }

  void resolveWritingArea()
  {
    if (!node_->get_parameter("area.derive_from_reach").as_bool()) {
      area_.u_min = node_->get_parameter("area.u_min").as_double();
      area_.u_max = node_->get_parameter("area.u_max").as_double();
      area_.v_min = node_->get_parameter("area.v_min").as_double();
      area_.v_max = node_->get_parameter("area.v_max").as_double();
    } else {
      const double radius = reachRadiusOnPlane();
      if (radius <= 0.0) {
        throw std::runtime_error(
                "plane_distance is beyond the arm's reach; nothing on that plane is drawable");
      }
      // The shoulder is on the base axis, so the disc is centred on u = 0 and
      // on the shoulder height in v.
      const double shoulder = node_->get_parameter("workspace.shoulder_height").as_double();
      area_.u_min = -kAreaHalfWidthFraction * radius;
      area_.u_max = kAreaHalfWidthFraction * radius;
      area_.v_min = shoulder - kAreaHalfHeightFraction * radius;
      area_.v_max = shoulder + kAreaHalfHeightFraction * radius;
      // Keep the bottom line off the floor even when the arm could reach it.
      area_.v_min = std::max(area_.v_min, node_->get_parameter("workspace.min_height").as_double());
    }

    if (area_.width() <= 0.0 || area_.height() <= 0.0) {
      throw std::runtime_error("the writing area is empty; check the area.* parameters");
    }
    RCLCPP_INFO(
      logger_, "Writing area on the plane x = %.3f: u [%.3f, %.3f], v [%.3f, %.3f] m",
      node_->get_parameter("plane_distance").as_double(), area_.u_min, area_.u_max, area_.v_min,
      area_.v_max);
  }

  WritingPlane writingPlane() const
  {
    WritingPlane plane;
    plane.frame = node_->get_parameter("frame_id").as_string();
    plane.distance = node_->get_parameter("plane_distance").as_double();
    plane.mirror = node_->get_parameter("mirror_text").as_bool();
    return plane;
  }

  /// Reject a placement the arm plainly cannot cover, by checking the corners
  /// of the glyph's ink box against the same sphere the area came from. With a
  /// derived area this is belt and braces; with an explicit one it is the only
  /// thing standing between a typo in a parameter file and a letter the arm
  /// will fail to plan halfway through.
  bool reachable(const PlacedGlyph & placed, std::string & why) const
  {
    if (!node_->get_parameter("workspace.check_glyphs").as_bool()) {
      return true;
    }
    const double max_reach = node_->get_parameter("workspace.max_reach").as_double() -
      node_->get_parameter("workspace.margin").as_double();
    const double min_reach = node_->get_parameter("workspace.min_reach").as_double();
    const Eigen::Vector3d shoulder(
      0.0, 0.0, node_->get_parameter("workspace.shoulder_height").as_double());

    const double u0 = placed.u + placed.glyph->ink_min.x() * placed.scale;
    const double u1 = placed.u + placed.glyph->ink_max.x() * placed.scale;
    const double v0 = placed.v + placed.glyph->ink_min.y() * placed.scale;
    const double v1 = placed.v + placed.glyph->ink_max.y() * placed.scale;

    for (const double u : {u0, u1}) {
      for (const double v : {v0, v1}) {
        const double distance = (writingPlane().point(u, v) - shoulder).norm();
        if (distance > max_reach) {
          why = "corner (" + std::to_string(u) + ", " + std::to_string(v) + ") is " +
            std::to_string(distance) + " m from the shoulder, past the usable reach of " +
            std::to_string(max_reach) + " m";
          return false;
        }
        if (distance < min_reach) {
          why = "corner (" + std::to_string(u) + ", " + std::to_string(v) + ") is " +
            std::to_string(distance) + " m from the shoulder, inside the minimum reach of " +
            std::to_string(min_reach) + " m";
          return false;
        }
      }
    }
    return true;
  }

  /// Tool pose and sampling for every stroke. The sampling matters: 2 mm
  /// waypoints keep a 60 mm letter smooth, and a blend distance of anything but
  /// zero would round the point off an A.
  StrokeStyle strokeStyle() const
  {
    StrokeStyle style;
    style.tool.face = msg::ToolSpec::FACE_INTO_PLANE;
    style.tool.spin = node_->get_parameter("tool.spin").as_double();
    style.tool.free_spin = node_->get_parameter("tool.free_spin").as_bool();
    style.tool.spin_samples = 0;
    style.tool.standoff = node_->get_parameter("tool.standoff").as_double();
    style.sampling.max_segment_length =
      node_->get_parameter("sampling.max_segment_length").as_double();
    style.sampling.chord_tolerance = node_->get_parameter("sampling.chord_tolerance").as_double();
    style.sampling.blend_distance = node_->get_parameter("sampling.blend_distance").as_double();
    style.velocity_scaling = node_->get_parameter("velocity_scaling").as_double();
    style.acceleration_scaling = node_->get_parameter("acceleration_scaling").as_double();
    return style;
  }

  void onWord(const std::string & text)
  {
    if (text.empty()) {
      RCLCPP_WARN(logger_, "Received an empty word; nothing to write");
      return;
    }

    LayoutOptions options;
    options.letter_height = node_->get_parameter("letter_height").as_double();
    options.letter_spacing = node_->get_parameter("letter_spacing").as_double();
    options.word_spacing = node_->get_parameter("word_spacing").as_double();
    options.line_spacing = node_->get_parameter("line_spacing").as_double();
    options.area = area_;

    const TextLayout layout = layoutText(text, alphabet_, options);
    if (!layout.unknown.empty()) {
      RCLCPP_WARN(
        logger_, "The font has no glyph for these characters, and they were skipped: '%s'",
        layout.unknown.c_str());
    }
    if (layout.truncated) {
      RCLCPP_WARN(
        logger_, "Ran out of writing area after %zu line(s); %zu letter(s) were dropped",
        layout.lines, layout.dropped);
    }
    if (layout.glyphs.empty()) {
      RCLCPP_WARN(logger_, "Nothing left to draw from \"%s\"", text.c_str());
      return;
    }

    ++word_count_;
    const std::string prefix = "w" + std::to_string(word_count_);
    const WritingPlane plane = writingPlane();
    const StrokeStyle style = strokeStyle();
    std::deque<msg::ShapeArray> batches;
    std::size_t strokes = 0;
    std::size_t unreachable = 0;

    for (std::size_t i = 0; i < layout.glyphs.size(); ++i) {
      const PlacedGlyph & placed = layout.glyphs[i];
      std::string why;
      if (!reachable(placed, why)) {
        RCLCPP_WARN(
          logger_, "Skipping '%c' at (%.3f, %.3f): %s", placed.character, placed.u, placed.v,
          why.c_str());
        ++unreachable;
        continue;
      }

      // Only the first letter of a word may clear what is already drawn; the
      // rest have to append or they would rub out the letters before them.
      const uint8_t mode =
        (batches.empty() && node_->get_parameter("replace_previous").as_bool()) ?
        msg::ShapeArray::REPLACE : msg::ShapeArray::APPEND;
      const std::string name = prefix + "_" + (i < 10 ? "0" : "") + std::to_string(i) + "_" +
        sanitize(placed.glyph->id);
      msg::ShapeArray batch = glyphToShapes(placed, plane, style, name, mode);
      strokes += batch.shapes.size();
      batches.push_back(std::move(batch));
    }

    if (batches.empty()) {
      RCLCPP_ERROR(logger_, "Every letter of \"%s\" was out of reach; nothing published", text.c_str());
      return;
    }

    if (!pending_.empty()) {
      RCLCPP_WARN(
        logger_, "A new word arrived with %zu letter(s) still unsent; dropping those",
        pending_.size());
    }
    pending_ = std::move(batches);
    const std::string skipped =
      unreachable ? " (" + std::to_string(unreachable) + " out of reach)" : "";
    RCLCPP_INFO(
      logger_,
      "Writing \"%s\": %zu letter(s), %zu stroke(s) over %zu line(s), %.0f mm cap height%s",
      text.c_str(), pending_.size(), strokes, layout.lines,
      options.letter_height * 1000.0, skipped.c_str());
  }

  /// Send the next letter. The tracer queues whole batches and draws them in
  /// arrival order, so pacing is not needed for correctness — but it keeps a
  /// long word from filling the subscription's queue in one go, and it makes
  /// the letters show up in RViz at a rate a human can follow.
  void publishNext()
  {
    if (pending_.empty()) {
      return;
    }
    publisher_->publish(pending_.front());
    pending_.pop_front();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  Alphabet alphabet_;
  WritingArea area_;

  rclcpp::Publisher<msg::ShapeArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr initial_timer_;

  /// Letters built but not yet sent, in reading order.
  std::deque<msg::ShapeArray> pending_;
  std::size_t word_count_{0};
};

}  // namespace avatar_challenge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(false);
  auto node = std::make_shared<rclcpp::Node>("word_writer", options);

  int exit_code = 0;
  try {
    avatar_challenge::WordWriter writer(node);
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "word_writer failed: %s", e.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}

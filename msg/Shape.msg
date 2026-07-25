# One shape to trace — the message form of an entry in the `shapes:` list of
# config/shapes.yaml (ShapeSpec in shape_spec.hpp).
#
# Units
# -----
# Unlike the YAML, this message has no `units:` block: everything is SI, metres
# and radians. The YAML supports mm/deg because it is hand-authored; a message
# carrying a geometry_msgs/Pose cannot coherently be in millimetres.
#
# Inheritance
# -----------
# In YAML an absent key means "inherit the file-level default". A message has no
# absent fields, so the has_* flags below carry that distinction explicitly. It
# matters because 0.0 is a meaningful value for tool.standoff (trace on the
# plane) and sampling.blend_distance (sharp corners).

# Used for logging, the RViz label and the TF frame "shape_<name>". Empty gets
# an auto-generated name; duplicates are suffixed rather than rejected.
string name

# Frame the start pose is expressed in. Empty means the node's base_frame.
string frame

# Pose of the shape frame in `frame`: the origin sits on the shape's first
# vertex and the Z axis is the drawing-plane normal.
geometry_msgs/Pose start

# Geometry: fill in EXACTLY ONE of these two.
#   vertices - a polygon. The first entry must be (0, 0): it defines the shape
#              frame origin. Needs at least 2 entries.
#   path     - a list of primitives (line/arc/circle/bspline).
avatar_challenge/Point2D[] vertices
avatar_challenge/Segment[] path

# Close the outline by returning to (0, 0) after the last primitive.
bool closed

# Set has_tool/has_sampling false to inherit the program defaults wholesale.
avatar_challenge/ToolSpec tool
bool has_tool
avatar_challenge/SamplingSpec sampling
bool has_sampling

# Per-shape motion overrides. <= 0 inherits, matching ShapeSpec.
float64 velocity_scaling
float64 acceleration_scaling

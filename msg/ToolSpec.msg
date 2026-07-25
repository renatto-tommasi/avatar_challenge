# How the tool is posed relative to the shape plane.
# Mirrors the `tool:` block of config/shapes.yaml (ToolSpec in shape_spec.hpp).

# Which way the tool's approach axis points along the plane normal Z_S.
# INTO_PLANE is a pen pressing on the page; it is the YAML default.
uint8 FACE_INTO_PLANE=0
uint8 FACE_ALONG_NORMAL=1
uint8 face

# Rotation of the tool about the plane normal, radians. The task only constrains
# the tool to be *normal* to the plane, so this angle is free.
float64 spin

# Treat `spin` as a second redundancy parameter and sweep it instead of holding
# it fixed. `spin_samples` <= 0 inherits the program default.
bool free_spin
int32 spin_samples

# Constant offset from the drawing plane along +Z_S, metres. 0 traces exactly on
# the plane, which is why Shape/has_tool exists rather than a sentinel value.
float64 standoff

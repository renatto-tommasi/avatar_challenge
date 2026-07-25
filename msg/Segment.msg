# One primitive of a shape outline, in the shape's 2D frame (metres).
# Mirrors an entry of a shape's `path:` list in config/shapes.yaml (Segment in
# shape_spec.hpp).
#
# A segment's start point is the previous segment's end point, or the shape
# frame origin for the first one, so only the terminating geometry is carried.

uint8 LINE=0      # straight line to `to`
uint8 ARC=1       # circular arc to `to`, about `center` or of radius `radius`
uint8 CIRCLE=2    # full circle about `center`; ignores `to`
uint8 BSPLINE=3   # B-spline over `control_points`; ignores `to`
uint8 type

# End point. Required by LINE and ARC, unused by CIRCLE and BSPLINE.
avatar_challenge/Point2D to

# ---- arc / circle ----------------------------------------------------------
# Arc centre. Required for CIRCLE; for ARC give either this or `radius`.
# has_center distinguishes "centre at the shape origin" from "no centre given".
bool has_center
avatar_challenge/Point2D center

# Only consulted when has_center is false.
float64 radius

# Sweep direction: true = counter-clockwise in the shape frame's XY plane.
bool counter_clockwise

# For radius-defined arcs, pick the sweep > pi. Ignored when a centre is given,
# since centre + direction already determine the sweep uniquely.
bool large_arc

# ---- b-spline --------------------------------------------------------------
avatar_challenge/Point2D[] control_points

# Curve degree; <= 0 falls back to 3 (cubic).
int32 degree

# Periodic (wrapped) rather than clamped.
bool periodic

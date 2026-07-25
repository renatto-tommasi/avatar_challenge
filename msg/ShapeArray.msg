# A batch of shapes for the tracer to draw, published on ~/add_shapes.
#
# One publication is one batch: the shapes in it are ordered and traced together,
# so the draw-order optimisation can reason about all of them at once. Publish
# again to add more.

# APPEND leaves everything already drawn on screen and traces these shapes next.
# REPLACE clears the accumulated markers first, so a producer can retract shapes
# it published earlier. APPEND is the zero value, so an unset field appends.
uint8 APPEND=0
uint8 REPLACE=1
uint8 mode

avatar_challenge/Shape[] shapes

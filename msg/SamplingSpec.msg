# Discretisation and smoothing knobs, all in metres.
# Mirrors the `sampling:` block of config/shapes.yaml (SamplingSpec in
# shape_spec.hpp).

# Upper bound on the distance between consecutive waypoints.
float64 max_segment_length

# Upper bound on how far a sampled chord may deviate from the true curve.
float64 chord_tolerance

# Corner rounding distance. 0 is meaningful — it keeps exact, sharp corners —
# so use Shape/has_sampling to inherit rather than leaving this at zero.
float64 blend_distance

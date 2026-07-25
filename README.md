# avatar_challenge — drawing 2D shapes in 3D with an xArm 7

Reads a set of 2D shapes from YAML and traces them in the air with a simulated
UFactory xArm 7, holding the tool normal to each shape's plane and using the
arm's redundant 7th degree of freedom to pick the cheapest way to draw each one
from wherever the arm happens to be.

```bash
cd ~/dev_ws
colcon build --packages-select avatar_challenge
source install/setup.bash
ros2 launch avatar_challenge start.launch.py
```

That brings up move_group, the fake ros2_control stack, RViz, and the tracer.
The arm draws the four demo shapes in `config/shapes.yaml`: a square, a
triangle, an arc-based stadium and a closed cubic B-spline.

![RViz after tracing the demo shapes](docs/rviz.png)

Green is the commanded outline, magenta is the tool path actually swept during
execution (reconstructed by FK over the executed trajectory), the translucent
blue patch is each shape's plane, and the small RGB triads are the shape frames.
The arm has parked at `home` so the drawings are unobstructed.

| argument | default | meaning |
| --- | --- | --- |
| `shapes_file` | `config/shapes.yaml` | shape program to trace |
| `params_file` | `config/shape_tracer.yaml` | node parameters |
| `plan_only` | `false` | plan and visualise without commanding the arm |
| `autostart` | `true` | `false` brings up the simulation only |
| `use_custom_rviz` | `true` | `false` restores the stock MoveIt RViz config |
| `trace_delay` | `12.0` | seconds to wait for move_group before tracing |

To iterate without restarting the simulation:

```bash
ros2 launch avatar_challenge start.launch.py autostart:=false   # terminal 1
ros2 launch avatar_challenge trace_shapes.launch.py             # terminal 2, re-run freely
```

---

## The coordinate model: a frame per shape

Every shape is authored in 2D inside its own **shape frame** `S`:

- the origin of `S` is the shape's first vertex — which the brief fixes at
  `(0, 0)`, so it is a validated invariant rather than a convention;
- the **XY plane of `S` is the drawing plane**, so the plane normal is just
  `Z_S`.

`start` in the YAML places `S` in the robot's base frame. Every authored
coordinate then maps back to robot coordinates with a single rigid transform:

```
p_base = T_base_shape · (x, y, standoff)
```

This is what makes the two hard requirements fall out for free:

- **Tool normal to the plane.** "Normal to the plane" is "the tool's approach
  axis is parallel to `Z_S`", which is a property of the frame, not of any
  individual waypoint. The tool orientation is therefore computed **once per
  shape** and shared by every waypoint —
  `R_base_tool = R_base_S · Rx(π) · Rz(spin)`. The `Rx(π)` points the
  end-effector's `+Z` along `−Z_S` ("into the page", like a pen); `tool.face:
  along_normal` drops it.
- **Rotating the shape.** Because the geometry never leaves 2D, changing
  `orientation_rpy` rotates the shape *and* the tool together with no extra
  work. The brief's "angled 45° about Z" is literally `orientation_rpy: [0, 0,
  45]`.

Each shape frame is also published on TF as `shape_<name>`, so the mapping is
directly inspectable in RViz.

`start.orientation_rpy` is fixed-axis roll-pitch-yaw, `R = Rz(yaw)·Ry(pitch)·Rx(roll)`
— the same convention as tf and URDF `<origin>` tags.

---

## Using the 7th degree of freedom

For a 7-DoF arm and a fully constrained 6-DoF task, the inverse kinematics has a
one-parameter family of solutions: the **self-motion manifold**. Every point on
it puts the tool in exactly the same place with the elbow swung somewhere else.
Which point you pick decides

- whether the *whole* shape stays inside the joint limits,
- how far the arm has to travel to get into position from where it is now,
- and how close it drifts to a singularity halfway through the trace.

Solving IK at the first waypoint and starting to draw throws that choice away —
it takes whatever the numeric solver converges to from the current seed.
`RedundancyResolver` (`src/redundancy.cpp`) instead:

1. **lands on the manifold once**, seeding IK from the robot's *current* joint
   state (random restarts only as a fallback);
2. **walks along it in both directions.** `J` is 6×7, so its null space is
   spanned by the last right-singular vector of its SVD. Stepping along that
   vector leaves the tool pose unchanged to first order; re-solving IK for the
   same entry pose cancels the second-order drift. Repeating gives a set of
   genuinely distinct postures for one entry pose;
3. **dry-runs the entire shape from each candidate**, IK-chaining waypoint to
   waypoint exactly the way MoveIt's Cartesian interpolator will, so a
   configuration that dies two thirds of the way round is caught *before*
   anything moves;
4. **scores and ranks** them:

   | term | what it measures |
   | --- | --- |
   | `approach` | weighted joint distance from the current configuration |
   | `path` | weighted joint travel accumulated while drawing |
   | `limits` | worst normalised joint-limit proximity along the trace |
   | `manip` | reciprocal of the worst manipulability `√det(JJᵀ)` on the trace |

   Joints are weighted individually (`joint_weights`, default
   `[2.0, 1.8, 1.4, 1.2, 0.8, 0.6, 0.4]`) because swinging joint 1 drags the
   whole arm through the workspace while spinning joint 7 barely moves anything.
   Candidates that cannot reach the whole outline are pushed behind every
   feasible one and ranked by how far they got.

The winner is commanded as a **joint-space goal**, not a pose goal, so the arm
arrives in the posture that was actually evaluated rather than whatever IK finds
at execution time. Only then does the Cartesian trace run, from that
configuration. If the Cartesian path still comes back short, the next-ranked
configuration is tried (`max_config_attempts`).

The ranking is printed for every shape, which makes the effect visible. From the
demo run — the stadium is reachable, but only from part of its manifold:

```
Shape 'stadium': evaluated 11 configuration(s) on the self-motion manifold
      rank  psi[rad]  spin  reach  approach   path  limits  manip  min|J|   total
         0    -1.400  0.000  1.000     3.852  2.338   0.443  1.359   0.037  10.551
         1    -1.200  0.000  1.000     3.705  2.438   0.418  1.512   0.033  10.719
         2    -1.000  0.000  1.000     3.557  2.640   0.395  1.809   0.028  11.239
         3    -0.800  0.000  0.593     3.409  1.551   0.374  1.901   0.026  1416.379  (incomplete)
         4    -0.600  0.000  0.444     3.264  1.258   0.356  1.981   0.025  1563.850  (incomplete)
         5    -0.400  0.000  0.407     3.122  1.205   0.344  2.307   0.022  1600.947  (incomplete)
         6    -1.600  0.000  0.222     3.999  0.714   0.435  1.122   0.045  1784.979  (incomplete)
         7    -1.800  0.000  0.111     4.145  0.553   0.440  1.041   0.048  1895.842  (incomplete)
```

Only three of the eleven candidates can trace the whole outline. Note ranks 3–5:
they are all *closer* to the arm's current configuration than the winner
(`approach` 3.1–3.4 vs 3.85) and have a lower drawing cost, so a greedy IK seed
would happily pick one — and stall between 40% and 60% of the way round.

### Two redundancy parameters, if you want them

Tracing a shape with a pen only really constrains 5 DoF: the rotation of the
tool *about* the plane normal is task-irrelevant. Setting `tool.free_spin: true`
treats that angle as a second search dimension — the manifold is re-enumerated
at each of `spin_samples` angles and the best configuration across all of them
wins. One spin angle is then held constant for the whole shape, so the tool
still stays rigidly normal to the plane. It is off by default so runs stay
deterministic and match the authored tool pose.

On a square 400 mm out on a vertical plane, enabling it evaluates 112
configurations instead of 21 and settles on a tool roll of 2.09 rad rather than
the authored 0.

### Self-collision

Every candidate configuration, and every waypoint of its dry run, is checked
against a `planning_scene::PlanningScene` built from the robot model, so it
carries the SRDF's allowed-collision matrix. A posture that folds the arm into
itself is discarded during the search rather than rejected later by the
Cartesian planner.

### Ordering multiple shapes

A *set* of shapes has no inherent order, so `optimize_shape_order` (default
true) picks one greedily: at each step it scores every remaining shape's entry
pose from the arm's current configuration and commits to the cheapest. Distance
is measured in weighted joint space, not Cartesian space — two shapes can be
centimetres apart and still need a completely different arm posture. Set it to
false to trace in file order.

---

## Geometry

`vertices` is sugar for a polygon; `path` takes a list of primitives. A shape
uses one or the other.

```yaml
shapes:
  - name: square
    start:
      position: [350.0, -120.0, 350.0]
      orientation_rpy: [0.0, 90.0, 45.0]
    vertices: [[0, 0], [0, 100], [100, 100], [100, 0]]
    closed: true
```

| primitive | fields |
| --- | --- |
| `line` | `to` |
| `arc` | `to` + (`center` \| `radius`), `direction: ccw\|cw`, `large_arc` |
| `circle` | `center`, `direction` |
| `bspline` | `control_points`, `degree`, `periodic` |

- **Arcs** may be given by centre (unambiguous) or by radius with SVG-style
  direction/large-arc disambiguation. The angular step is chosen so that *both*
  the chord length and the sagitta stay within `max_segment_length` and
  `chord_tolerance`.
- **B-splines** are evaluated with de Boor's algorithm, clamped by default or
  wrapped periodically with `periodic: true`. The curve is prepended with the
  current point if the author did not start it there, so outlines stay
  connected.
- **Corner blending** (`sampling.blend_distance`) rounds every tangent
  discontinuity with a quadratic Bézier that starts and ends that far back along
  the neighbouring edges. It is G¹-continuous with both, which is what removes
  the full stop the trajectory would otherwise take at each vertex. It runs on
  the sampled polyline, so it works for line↔line, line↔arc and arc↔spline joins
  alike. Set it to 0 for exact, sharp corners.

Units are per-file and per-shape: `units: {length: m|cm|mm, angle: rad|deg}`.

---

## Execution

Per shape:

1. **transit** — joint-space plan (OMPL RRTConnect) to the chosen configuration
   at the approach pose, one `approach_distance` back along `+Z_S`, so transits
   between shapes never drag the tool across a drawing plane;
2. **trace** — `computeCartesianPath` through descend → outline → retreat, held
   on a single IK branch by the jump threshold;
3. **retime** — `computeCartesianPath` interpolates in configuration space and
   returns a path that is geometrically right but dynamically meaningless, so
   the result is re-parameterised with **TOTG**
   (`TimeOptimalTrajectoryGeneration`) against the real velocity and
   acceleration limits. Its `path_tolerance` rounds corners in joint space,
   complementing the geometric blending upstream;
4. **execute**, then publish the swept tool path as a marker.

Once every shape is drawn the arm returns to the named SRDF state in
`park_state` (default `home`), so it ends somewhere known rather than hovering
over the last outline it traced. Set it to `""` to stay put.

The joint limits TOTG retimes against come from `joint_limits.yaml`, which
reaches the node as `robot_description_planning` — see below.

`plan_only:=true` runs the whole pipeline without commanding the arm. It keeps a
virtual joint cursor across shapes, so the previewed ordering and approach costs
are the ones a real run would see rather than every shape being costed from the
same start pose. The full four-shape program previews in under a second.

---

## Layout

```
avatar_challenge/
├── config/
│   ├── shapes.yaml            # the shape program (documented inline)
│   └── shape_tracer.yaml      # node parameters
├── include/avatar_challenge/
│   ├── shape_spec.hpp         # data model
│   ├── path_sampler.hpp       # 2D sampling + the lift into robot coordinates
│   ├── redundancy.hpp         # 7th-DoF search
│   └── markers.hpp
├── src/
│   ├── yaml_loader.cpp        # YAML -> ShapeProgram, with unit conversion
│   ├── path_sampler.cpp       # lines, arcs, circles, B-splines, blending
│   ├── redundancy.cpp         # null-space walk, dry-run, scoring
│   ├── markers.cpp
│   └── shape_tracer_node.cpp  # orchestration
├── avatar_challenge_launch/
│   └── moveit_params.py       # shared MoveIt config for the launch files
├── launch/
│   ├── start.launch.py        # simulation + RViz + tracer
│   └── trace_shapes.launch.py # tracer only
├── rviz/shape_tracer.rviz
├── docs/rviz.png
└── test/test_shape_geometry.cpp
```

Published topics: `~/shape_markers` (outlines, planes, shape frames, labels) and
`~/traced_path` (the executed tool path), both latched. Shape frames are also on
TF as `shape_<name>`.

The geometry, YAML and redundancy code is a library (`shape_tracing`) separate
from the node, so it can be tested without a running move_group:

```bash
colcon test --packages-select avatar_challenge && colcon test-result --all
```

13 tests cover unit conversion, every primitive kind, arc direction and
large-arc handling, the B-spline convex-hull property, that blending removes the
90° tangent jumps of a square, and that every waypoint lands on the plane with
the tool normal to it.

---

## Notes on integrating with the controller workspace

Two things about `~/xarm_ws` are worth knowing, because both cost a debugging
cycle:

- **move_group's robot description is a parameter, not a topic.** Anything that
  builds its own `RobotModel` — `MoveGroupInterface` here, and RViz's MoveIt
  displays — has to be handed `robot_description`,
  `robot_description_semantic`, `robot_description_kinematics` and
  `robot_description_planning` directly. `xarm_moveit_config` builds that set
  inside `_robot_moveit_fake.launch.py` and never exports it, so
  `avatar_challenge_launch/moveit_params.py` repeats the same
  `MoveItConfigsBuilder` call with the same arguments (`dof=7`,
  `robot_type=xarm`, `limited=True`, fake hardware plugin). Keeping the
  arguments identical is what guarantees the tracer plans against exactly the
  robot move_group executes on.
- **The stock launch always starts its own RViz.** `_robot_moveit_common2.launch.py`
  gates that node on a `show_rviz` launch configuration that nothing along the
  include chain sets, so `SetLaunchConfiguration('show_rviz', 'false')` in
  `start.launch.py` propagates into the include and suppresses it. The stock
  config has no MarkerArray display, so the shape markers would otherwise be
  invisible. `use_custom_rviz:=false` restores it.

The IK plugin configured for `xarm7` is KDL, a seed-based numerical solver.
That is a good fit here: the seed *is* the redundancy parameter, which is
exactly what the manifold walk manipulates. Switching to TRAC-IK (commented out
in `xarm_moveit_config/config/xarm7/kinematics.yaml`) would work too, but its
internal randomised restarts make the manifold walk less repeatable.

## Tuning

Everything is in `config/shape_tracer.yaml`. The knobs that matter most:

- `redundancy.samples_per_direction` × `redundancy.null_step` sets how much of
  the manifold gets explored — the default `10 × 0.20 rad` covers roughly ±2 rad
  of elbow swing.
- `redundancy.evaluation_stride` trades search fidelity for speed; every shape
  in the demo evaluates in well under a second at the default of 4.
- `redundancy.w_approach` up, `w_path` down favours postures near where the arm
  already is; `w_manip` up keeps it further from singularities at the cost of a
  longer transit.
- `sampling.blend_distance` in the shapes file controls corner smoothing.

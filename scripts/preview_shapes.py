#!/usr/bin/env python3
"""
Draw what the tracer is about to draw, into a PNG, without a robot.

    ros2 run avatar_challenge preview_shapes.py --output /tmp/preview.png
    ros2 run avatar_challenge preview_shapes.py --seconds 20 --view yz

It listens on the same ~/add_shapes topic the tracer does, samples every shape
it hears about, projects the result onto a plane and writes an image. That makes
it the fast way to check a shape generator -- word_writer especially -- because
you see the letters in a second instead of waiting out a full trace, and you can
do it with no simulation running at all.

It is a preview, not a second implementation: the sampling here is coarser than
src/path_sampler.cpp and does no corner blending, so trust the tracer for
anything subtle. Shapes are drawn in the order they arrive, oldest first.

Views: `yz` (the default) looks along the robot's +X axis, which is the one to
use for a writing plane parallel to YZ; `xz` looks along +Y; `xy` from above.
"""

import argparse
import math
import struct
import sys
import zlib

from avatar_challenge.msg import Segment, ShapeArray
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# Angular step for arcs, and sample count for splines. Fine enough that a 60 mm
# letter looks like itself.
ARC_STEP = math.radians(3.0)
SPLINE_STEPS = 120


# --------------------------------------------------------------------------
# geometry
# --------------------------------------------------------------------------


def quat_to_matrix(q):
    """Rotation matrix of a geometry_msgs/Quaternion, as three column vectors."""
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def to_base(pose, point2d):
    """Lift a point in the shape's 2D frame into the robot base frame."""
    r = quat_to_matrix(pose.orientation)
    u, v = point2d
    return (
        pose.position.x + r[0][0] * u + r[0][1] * v,
        pose.position.y + r[1][0] * u + r[1][1] * v,
        pose.position.z + r[2][0] * u + r[2][1] * v,
    )


def sample_arc(start, center, to, counter_clockwise, out):
    radius = math.hypot(start[0] - center[0], start[1] - center[1])
    if radius < 1e-9:
        return start
    a0 = math.atan2(start[1] - center[1], start[0] - center[0])
    a1 = math.atan2(to[1] - center[1], to[0] - center[0])
    sweep = (a1 - a0) % (2 * math.pi) if counter_clockwise else -((a0 - a1) % (2 * math.pi))
    if abs(sweep) < 1e-9:
        sweep = 2 * math.pi if counter_clockwise else -2 * math.pi
    steps = max(1, int(math.ceil(abs(sweep) / ARC_STEP)))
    for i in range(1, steps + 1):
        a = a0 + sweep * i / steps
        out.append((center[0] + radius * math.cos(a), center[1] + radius * math.sin(a)))
    return out[-1]


def arc_center_from_radius(start, to, radius, counter_clockwise, large_arc):
    """The SVG-style construction src/path_sampler.cpp uses."""
    cx, cy = to[0] - start[0], to[1] - start[1]
    d = math.hypot(cx, cy)
    if d < 1e-9:
        return start
    r = max(abs(radius), d / 2.0)
    h = math.sqrt(max(0.0, r * r - 0.25 * d * d))
    mid = ((start[0] + to[0]) / 2.0, (start[1] + to[1]) / 2.0)
    normal = (-cy / d, cx / d)
    left = counter_clockwise != large_arc
    sign = 1.0 if left else -1.0
    return (mid[0] + sign * h * normal[0], mid[1] + sign * h * normal[1])


def de_boor(ctrl, knots, p, u):
    n = len(ctrl)
    k = p
    for i in range(p, n):
        if knots[i] <= u < knots[i + 1]:
            k = i
            break
        k = n - 1
    d = [ctrl[k - p + j] for j in range(p + 1)]
    for r in range(1, p + 1):
        for j in range(p, r - 1, -1):
            idx = k - p + j
            den = knots[idx + p - r + 1] - knots[idx]
            a = 0.0 if abs(den) < 1e-12 else (u - knots[idx]) / den
            d[j] = (
                (1 - a) * d[j - 1][0] + a * d[j][0],
                (1 - a) * d[j - 1][1] + a * d[j][1],
            )
    return d[p]


def sample_bspline(start, segment, out):
    ctrl = [(p.x, p.y) for p in segment.control_points]
    if not ctrl:
        return start
    if math.dist(ctrl[0], start) > 1e-6:
        ctrl.insert(0, start)
    p = max(1, segment.degree or 3)
    if segment.periodic:
        original = len(ctrl)
        ctrl += [ctrl[i % original] for i in range(p)]
    p = min(p, len(ctrl) - 1)
    if segment.periodic:
        knots = [float(i - p) for i in range(len(ctrl) + p + 1)]
    else:
        interior = len(ctrl) - p - 1
        knots = [0.0] * (p + 1) + [
            (i + 1) / (interior + 1) for i in range(max(0, interior))
        ] + [1.0] * (p + 1)
    lo, hi = knots[p], knots[len(ctrl)]
    for i in range(1, SPLINE_STEPS + 1):
        u = lo + (hi - lo) * i / SPLINE_STEPS - (1e-9 if i == SPLINE_STEPS else 0.0)
        out.append(de_boor(ctrl, knots, p, u))
    return out[-1]


def sample_shape(shape):
    """Every point of one shape, in the robot base frame."""
    points = [(0.0, 0.0)]
    if shape.vertices:
        points = [(v.x, v.y) for v in shape.vertices]
    else:
        cursor = (0.0, 0.0)
        for segment in shape.path:
            if segment.type == Segment.LINE:
                points.append((segment.to.x, segment.to.y))
                cursor = points[-1]
            elif segment.type == Segment.ARC:
                center = (
                    (segment.center.x, segment.center.y)
                    if segment.has_center
                    else arc_center_from_radius(
                        cursor,
                        (segment.to.x, segment.to.y),
                        segment.radius,
                        segment.counter_clockwise,
                        segment.large_arc,
                    )
                )
                cursor = sample_arc(
                    cursor, center, (segment.to.x, segment.to.y),
                    segment.counter_clockwise, points)
            elif segment.type == Segment.CIRCLE:
                cursor = sample_arc(
                    cursor, (segment.center.x, segment.center.y), cursor,
                    segment.counter_clockwise, points)
            elif segment.type == Segment.BSPLINE:
                cursor = sample_bspline(cursor, segment, points)
    if shape.closed and points[0] != points[-1]:
        points.append(points[0])
    return [to_base(shape.start, p) for p in points]


# --------------------------------------------------------------------------
# a very small PNG writer -- no numpy, no PIL, nothing to install
# --------------------------------------------------------------------------


class Canvas:
    """8-bit greyscale raster with 3x supersampled line drawing."""

    SUPER = 3

    def __init__(self, width, height):
        self.width, self.height = width, height
        self.w, self.h = width * self.SUPER, height * self.SUPER
        self.pixels = bytearray([255]) * (self.w * self.h)

    def dot(self, x, y, value, radius):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                px, py = int(x) + dx, int(y) + dy
                if 0 <= px < self.w and 0 <= py < self.h and dx * dx + dy * dy <= radius * radius:
                    index = py * self.w + px
                    if self.pixels[index] > value:
                        self.pixels[index] = value

    def line(self, p0, p1, value=0, radius=2):
        x0, y0 = p0
        x1, y1 = p1
        steps = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
        for i in range(steps + 1):
            t = i / steps
            self.dot(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, value, radius)

    def downsample(self):
        out = bytearray(self.width * self.height)
        s = self.SUPER
        area = s * s
        for y in range(self.height):
            for x in range(self.width):
                total = 0
                for dy in range(s):
                    row = (y * s + dy) * self.w + x * s
                    total += sum(self.pixels[row:row + s])
                out[y * self.width + x] = total // area
        return out

    def write_png(self, path):
        raw = self.downsample()
        lines = b"".join(
            b"\x00" + bytes(raw[y * self.width:(y + 1) * self.width])
            for y in range(self.height)
        )

        def chunk(tag, data):
            body = tag + data
            return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

        header = struct.pack(">IIBBBBB", self.width, self.height, 8, 0, 0, 0, 0)
        with open(path, "wb") as handle:
            handle.write(b"\x89PNG\r\n\x1a\n")
            handle.write(chunk(b"IHDR", header))
            handle.write(chunk(b"IDAT", zlib.compress(lines, 9)))
            handle.write(chunk(b"IEND", b""))


PROJECTIONS = {
    # name: (horizontal axis, vertical axis) as (index, sign) pairs
    "yz": ((1, -1.0), (2, 1.0)),
    "xz": ((0, 1.0), (2, 1.0)),
    "xy": ((0, 1.0), (1, -1.0)),
}


def render(paths, path, view, width, height, margin):
    axes = PROJECTIONS[view]
    flat = [
        (axes[0][1] * p[axes[0][0]], axes[1][1] * p[axes[1][0]])
        for polyline in paths for p in polyline
    ]
    if not flat:
        return None
    lo_x, hi_x = min(p[0] for p in flat), max(p[0] for p in flat)
    lo_y, hi_y = min(p[1] for p in flat), max(p[1] for p in flat)
    span = max(hi_x - lo_x, hi_y - lo_y, 1e-6)
    scale = (min(width, height) - 2 * margin) * Canvas.SUPER / span
    off_x = (width * Canvas.SUPER - (hi_x - lo_x) * scale) / 2.0
    off_y = (height * Canvas.SUPER - (hi_y - lo_y) * scale) / 2.0

    canvas = Canvas(width, height)
    for polyline in paths:
        previous = None
        for point in polyline:
            x = off_x + (axes[0][1] * point[axes[0][0]] - lo_x) * scale
            # Image rows grow downwards, so the vertical axis is flipped here.
            y = height * Canvas.SUPER - off_y - (axes[1][1] * point[axes[1][0]] - lo_y) * scale
            if previous is not None:
                canvas.line(previous, (x, y))
            previous = (x, y)
    canvas.write_png(path)
    return (lo_x, hi_x, lo_y, hi_y)


# --------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/shape_tracer/add_shapes")
    parser.add_argument("--output", default="preview.png")
    parser.add_argument("--seconds", type=float, default=10.0,
                        help="how long to listen for shapes")
    parser.add_argument("--quiet-for", type=float, default=2.5,
                        help="stop early once no shape has arrived for this long")
    parser.add_argument("--view", default="yz", choices=sorted(PROJECTIONS))
    parser.add_argument("--width", type=int, default=900)
    parser.add_argument("--height", type=int, default=520)
    parser.add_argument("--margin", type=int, default=20)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Node("preview_shapes")
    collected = []
    names = []

    def on_shapes(message):
        for shape in message.shapes:
            collected.append(sample_shape(shape))
            names.append(shape.name)
        node.get_logger().info(f"{len(collected)} shape(s) so far")

    node.create_subscription(
        ShapeArray, args.topic, on_shapes,
        QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))

    clock = node.get_clock()
    deadline = clock.now().nanoseconds + int(args.seconds * 1e9)
    last_seen = clock.now().nanoseconds
    count = 0
    while rclpy.ok() and clock.now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(collected) != count:
            count = len(collected)
            last_seen = clock.now().nanoseconds
        elif count and clock.now().nanoseconds - last_seen > args.quiet_for * 1e9:
            break

    node.destroy_node()
    rclpy.shutdown()

    if not collected:
        print(f"no shapes arrived on {args.topic}", file=sys.stderr)
        return 1
    extent = render(collected, args.output, args.view,
                    args.width, args.height, args.margin)
    print(f"{len(collected)} shape(s) -> {args.output}")
    print(f"extent in the {args.view} view: "
          f"[{extent[0]:.3f}, {extent[1]:.3f}] x [{extent[2]:.3f}, {extent[3]:.3f}] m")
    return 0


if __name__ == "__main__":
    sys.exit(main())

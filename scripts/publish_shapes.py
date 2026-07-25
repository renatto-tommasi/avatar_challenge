#!/usr/bin/env python3
"""
Publish a shapes YAML file on the tracer's ~/add_shapes topic.

    ros2 run avatar_challenge publish_shapes.py my_shapes.yaml
    ros2 run avatar_challenge publish_shapes.py my_shapes.yaml --replace
    ros2 run avatar_challenge publish_shapes.py my_shapes.yaml --topic /other/add_shapes

This is a development and testing convenience -- `ros2 topic pub` is impractical
for a message this deeply nested -- and a worked example for a node that
generates shapes of its own. It is NOT a second parser: the node's own
src/yaml_loader.cpp remains the reference implementation of the schema, and this
script only covers what it needs to in order to build the message.

The message is SI throughout, so `units:` is applied here rather than shipped.
"""

import argparse
import math
import sys

from avatar_challenge.msg import (
    Point2D,
    Segment,
    SamplingSpec,
    Shape,
    ShapeArray,
    ToolSpec,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml

LENGTH_SCALE = {"m": 1.0, "meters": 1.0, "metres": 1.0, "cm": 0.01,
                "mm": 0.001, "millimeters": 0.001, "millimetres": 0.001}
ANGLE_SCALE = {"rad": 1.0, "radians": 1.0, "deg": math.pi / 180.0,
               "degrees": math.pi / 180.0}

SEGMENT_TYPES = {"line": Segment.LINE, "arc": Segment.ARC,
                 "circle": Segment.CIRCLE, "bspline": Segment.BSPLINE}


def _units(node, inherited):
    """Resolve a `units:` block against the enclosing scope's scales."""
    length, angle = inherited
    if not node:
        return length, angle
    if "length" in node:
        length = LENGTH_SCALE[node["length"]]
    if "angle" in node:
        angle = ANGLE_SCALE[node["angle"]]
    return length, angle


def _point(pair, scale):
    return Point2D(x=float(pair[0]) * scale, y=float(pair[1]) * scale)


def _quaternion_from_rpy(roll, pitch, yaw):
    """Fixed-axis RPY, i.e. R = Rz(yaw) * Ry(pitch) * Rx(roll).

    The same convention parseStartPose() uses in src/yaml_loader.cpp, which is
    also what tf and URDF origins use.
    """
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    )


def _segment(node, length, angle):
    seg = Segment()
    seg.type = SEGMENT_TYPES[node["type"]]
    if "to" in node:
        seg.to = _point(node["to"], length)
    if "center" in node:
        seg.has_center = True
        seg.center = _point(node["center"], length)
    if "radius" in node:
        seg.radius = float(node["radius"]) * length
    # cw|ccw; the message default (False) would silently mean clockwise, so be
    # explicit and match the loader's default of counter-clockwise.
    seg.counter_clockwise = node.get("direction", "ccw") in ("ccw", "counter_clockwise")
    seg.large_arc = bool(node.get("large_arc", False))
    seg.degree = int(node.get("degree", 3))
    seg.periodic = bool(node.get("periodic", False))
    for point in node.get("control_points", []):
        seg.control_points.append(_point(point, length))
    del angle  # no angular quantities inside a segment yet
    return seg


def _shape(node, file_units, frame):
    length, angle = _units(node.get("units"), file_units)

    shape = Shape()
    shape.name = str(node.get("name", ""))
    shape.frame = str(node.get("frame", frame or ""))
    shape.closed = bool(node.get("closed", True))

    start = node["start"]
    position = start["position"]
    shape.start.position.x = float(position[0]) * length
    shape.start.position.y = float(position[1]) * length
    shape.start.position.z = float(position[2]) * length
    if "orientation_quat_xyzw" in start:
        qx, qy, qz, qw = (float(v) for v in start["orientation_quat_xyzw"])
    else:
        roll, pitch, yaw = (float(v) * angle for v in start["orientation_rpy"])
        qx, qy, qz, qw = _quaternion_from_rpy(roll, pitch, yaw)
    shape.start.orientation.x = qx
    shape.start.orientation.y = qy
    shape.start.orientation.z = qz
    shape.start.orientation.w = qw

    for vertex in node.get("vertices", []):
        shape.vertices.append(_point(vertex, length))
    for segment in node.get("path", []):
        shape.path.append(_segment(segment, length, angle))

    # An absent block means "inherit", which is exactly what has_* is for.
    if "tool" in node:
        shape.has_tool = True
        tool = node["tool"]
        shape.tool = ToolSpec()
        shape.tool.face = (ToolSpec.FACE_ALONG_NORMAL
                           if tool.get("face") == "along_normal"
                           else ToolSpec.FACE_INTO_PLANE)
        shape.tool.spin = float(tool.get("spin", 0.0)) * angle
        shape.tool.free_spin = bool(tool.get("free_spin", False))
        shape.tool.spin_samples = int(tool.get("spin_samples", 0))
        shape.tool.standoff = float(tool.get("standoff", 0.0)) * length
    if "sampling" in node:
        shape.has_sampling = True
        sampling = node["sampling"]
        shape.sampling = SamplingSpec()
        shape.sampling.max_segment_length = float(sampling["max_segment_length"]) * length
        shape.sampling.chord_tolerance = float(sampling["chord_tolerance"]) * length
        shape.sampling.blend_distance = float(sampling.get("blend_distance", 0.0)) * length

    shape.velocity_scaling = float(node.get("velocity_scaling", 0.0))
    shape.acceleration_scaling = float(node.get("acceleration_scaling", 0.0))
    return shape


def build_message(path, replace):
    with open(path) as handle:
        root = yaml.safe_load(handle) or {}

    defaults = root.get("defaults") or {}
    file_units = _units(defaults.get("units"), (1.0, 1.0))
    frame = defaults.get("reference_frame", "")

    # A per-shape `sampling:` block in the YAML overrides only the keys it names,
    # but the message carries the whole struct, so merge the file defaults in
    # first or an unset key would arrive as zero.
    default_sampling = defaults.get("sampling") or {}

    message = ShapeArray()
    message.mode = ShapeArray.REPLACE if replace else ShapeArray.APPEND
    for node in root.get("shapes") or []:
        if "sampling" in node:
            node["sampling"] = {**default_sampling, **node["sampling"]}
        message.shapes.append(_shape(node, file_units, frame))
    return message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("shapes_file", help="YAML file in the config/shapes.yaml schema")
    parser.add_argument("--topic", default="/shape_tracer/add_shapes")
    parser.add_argument(
        "--replace", action="store_true",
        help="clear previously drawn shapes instead of appending to them")
    args, ros_args = parser.parse_known_args()

    message = build_message(args.shapes_file, args.replace)
    if not message.shapes and not args.replace:
        print(f"{args.shapes_file} has no shapes; nothing to publish", file=sys.stderr)
        return 1

    rclpy.init(args=ros_args)
    node = Node("publish_shapes")
    # Transient-local so the tracer picks this up even if it starts afterwards;
    # it must match the subscription QoS in src/shape_tracer_node.cpp.
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(ShapeArray, args.topic, qos)

    publisher.publish(message)
    node.get_logger().info(
        f"Published {len(message.shapes)} shape(s) on {args.topic} "
        f"({'replace' if args.replace else 'append'})")

    # A transient-local publisher has to outlive the publish call for a late
    # subscriber to get the sample, and even a live one needs a moment for
    # delivery. Spin briefly rather than exiting immediately.
    end = node.get_clock().now().nanoseconds + 2_000_000_000
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

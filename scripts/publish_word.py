#!/usr/bin/env python3
"""
Send a word to the word_writer node for the arm to write.

    ros2 run avatar_challenge publish_word.py HELLO WORLD
    ros2 run avatar_challenge publish_word.py "HELLO, ROBOT!"
    ros2 run avatar_challenge publish_word.py --line AVATAR --line ROBOTICS
    ros2 run avatar_challenge publish_word.py 'AVATAR\\nROBOTICS'

Every positional argument is joined with a space, so quoting is optional for a
plain sentence.

Line breaks
-----------
`--line` is the reliable way to get one: each occurrence is its own line and
nothing has to survive the shell.

The `\\n` form works too, but **only inside quotes**. Unquoted, bash removes the
backslash before this script ever sees it:

    publish_word.py AVATAR\\nROBOTICS     ->  "AVATARnROBOTICS"   (one line, with an N)
    publish_word.py 'AVATAR\\nROBOTICS'   ->  two lines

so the script warns when the text looks like that happened. word_writer treats a
literal backslash-n as a break as well, which is what makes the quoted form work
without this script decoding anything.

Why not just `ros2 topic pub`
-----------------------------
It gets two things wrong. The word topic is a plain VOLATILE subscription, so a
message fired before discovery has finished goes nowhere, silently, which looks
exactly like a broken node; and a publisher that exits immediately may never
flush. This waits for word_writer to subscribe, publishes, then spins a moment.

The node does the rest: it splits the text into letters, works out where each one
starts from its own width and what the arm can reach, and publishes them to the
tracer one letter at a time. Watch its log to see where the text landed.
"""

import argparse
import re
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# A lone 'n' wedged in front of a capital is what an unquoted \n leaves behind.
EATEN_ESCAPE = re.compile(r"(?:^|[^a-z])n[A-Z]")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("text", nargs="*", help="what to write; joined with spaces")
    parser.add_argument(
        "--line", "-l", action="append", default=[], metavar="TEXT",
        help="one line of text; repeat for more. Needs no escaping, so this is "
             "the reliable way to break a line")
    parser.add_argument("--topic", default="/word_writer/word")
    parser.add_argument(
        "--wait", type=float, default=5.0,
        help="seconds to wait for word_writer to subscribe (0 publishes immediately)")
    args, ros_args = parser.parse_known_args()

    lines = list(args.line)
    if args.text:
        lines.append(" ".join(args.text))
    if not lines:
        parser.error("give some text, or --line")
    text = "\n".join(lines)

    if "\n" not in text and "\\n" not in text and EATEN_ESCAPE.search(text):
        print(
            f"warning: {text!r} contains a lone 'n' before a capital, which is what an "
            "unquoted \\n turns into -- your shell ate the backslash.\n"
            "         Quote it, or use --line for each line.",
            file=sys.stderr)

    rclpy.init(args=ros_args)
    node = Node("publish_word")
    # Matches the node's subscription: RELIABLE + VOLATILE, depth 10. A word is
    # a command, not state, so there is nothing to latch for a late joiner --
    # hence the wait below instead of TRANSIENT_LOCAL.
    publisher = node.create_publisher(
        String, args.topic, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    deadline = node.get_clock().now().nanoseconds + int(args.wait * 1e9)
    while (rclpy.ok() and publisher.get_subscription_count() == 0 and
           node.get_clock().now().nanoseconds < deadline):
        rclpy.spin_once(node, timeout_sec=0.05)

    if publisher.get_subscription_count() == 0:
        node.get_logger().warn(
            f"nobody is subscribed to {args.topic}; is word_writer running? "
            "Publishing anyway")

    publisher.publish(String(data=text))
    node.get_logger().info(
        f"Published {text!r} on {args.topic}"
        f"{f' ({len(lines)} lines)' if len(lines) > 1 else ''}")

    # Give the middleware a moment to send it before the context goes away.
    end = node.get_clock().now().nanoseconds + 500_000_000
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

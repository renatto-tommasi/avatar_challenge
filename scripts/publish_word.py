#!/usr/bin/env python3
"""
Send a word to the word_writer node for the arm to write.

    ros2 run avatar_challenge publish_word.py HELLO WORLD
    ros2 run avatar_challenge publish_word.py "HELLO, ROBOT!"
    ros2 run avatar_challenge publish_word.py 'LINE ONE\\nLINE TWO'
    ros2 run avatar_challenge publish_word.py --topic /word_writer/word ABC

Every argument is joined with a space, so quoting is optional for a plain
sentence. A literal `\\n` in the text becomes a line break, which word_writer
honours as a forced one rather than wrapping.

This is `ros2 topic pub` with the two things that bite you taken care of:

  * it waits for word_writer to be subscribed before publishing. The word topic
    is a plain VOLATILE subscription -- fire a message at it before discovery
    has finished and it goes nowhere, silently, which looks exactly like a
    broken node;
  * it then spins for a moment so the message is actually flushed before the
    process exits.

The node does the rest: it splits the text into letters, works out where each
one starts from its own width and what the arm can reach, and publishes them to
the tracer one letter at a time. Watch its log to see where the text landed.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("text", nargs="+", help="what to write; joined with spaces")
    parser.add_argument("--topic", default="/word_writer/word")
    parser.add_argument(
        "--wait", type=float, default=5.0,
        help="seconds to wait for word_writer to subscribe (0 publishes immediately)")
    args, ros_args = parser.parse_known_args()

    # argparse hands over the shell's literal backslash-n; the layout wants a
    # real newline.
    text = " ".join(args.text).replace("\\n", "\n")

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
    node.get_logger().info(f"Published {text!r} on {args.topic}")

    # Give the middleware a moment to send it before the context goes away.
    end = node.get_clock().now().nanoseconds + 500_000_000
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

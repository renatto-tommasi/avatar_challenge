"""
Start the word_writer node, which turns strings into letters for the tracer.

It needs a running shape_tracer to draw them; bring one up with an empty shape
program so the plane starts clean:

    ros2 launch avatar_challenge start.launch.py shapes_file:=""      # terminal 1
    ros2 launch avatar_challenge write_word.launch.py                 # terminal 2

then write words at it:

    ros2 topic pub --once /word_writer/word std_msgs/msg/String "{data: 'HELLO'}"

or have the node write one by itself as soon as it starts:

    ros2 launch avatar_challenge write_word.launch.py word:="HELLO WORLD"

word_writer does not talk to MoveIt at all -- it only publishes shape messages --
so it is cheap to restart while the simulation keeps running.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _writer(context):
    overrides = {
        "initial_word": LaunchConfiguration("word").perform(context),
        "letter_height": float(LaunchConfiguration("letter_height").perform(context)),
        "plane_distance": float(LaunchConfiguration("plane_distance").perform(context)),
    }
    alphabet_file = LaunchConfiguration("alphabet_file").perform(context)
    if alphabet_file:
        overrides["alphabet_file"] = alphabet_file

    return [
        Node(
            package="avatar_challenge",
            executable="word_writer",
            name="word_writer",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file").perform(context),
                overrides,
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("avatar_challenge"), "config", "word_writer.yaml"]
                ),
                description="YAML file with word_writer node parameters.",
            ),
            DeclareLaunchArgument(
                "alphabet_file",
                default_value="",
                description="Font to write with. Empty uses the packaged "
                "config/alphabet.yaml.",
            ),
            DeclareLaunchArgument(
                "word",
                default_value="",
                description="Write this as soon as the node starts. Empty just "
                "waits for the topic.",
            ),
            DeclareLaunchArgument(
                "letter_height",
                default_value="0.06",
                description="Metres per capital letter.",
            ),
            DeclareLaunchArgument(
                "plane_distance",
                default_value="0.42",
                description="Distance in front of the robot of the writing "
                "plane, which is parallel to the base YZ plane.",
            ),
            OpaqueFunction(function=_writer),
        ]
    )

"""
Start only the shape_tracer node, against an already-running move_group.

Useful for iterating: leave the simulation up in one terminal with

    ros2 launch avatar_challenge start.launch.py autostart:=false

and re-run this after every edit:

    ros2 launch avatar_challenge trace_shapes.launch.py
    ros2 launch avatar_challenge trace_shapes.launch.py shapes_file:=/abs/path.yaml plan_only:=true
"""

from avatar_challenge_launch.moveit_params import robot_description_params
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _tracer(context):
    return [
        Node(
            package="avatar_challenge",
            executable="shape_tracer",
            name="shape_tracer",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file").perform(context),
                robot_description_params(context),
                {
                    "shapes_file": LaunchConfiguration("shapes_file").perform(context),
                    "plan_only": LaunchConfiguration("plan_only")
                    .perform(context)
                    .lower()
                    in ("true", "1"),
                    "exit_when_done": LaunchConfiguration("exit_when_done")
                    .perform(context)
                    .lower()
                    in ("true", "1"),
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "shapes_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("avatar_challenge"), "config", "shapes.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("avatar_challenge"), "config", "shape_tracer.yaml"]
                ),
            ),
            DeclareLaunchArgument("plan_only", default_value="false"),
            DeclareLaunchArgument(
                "exit_when_done",
                default_value="false",
                description="Shut the node down after the last shape instead of "
                "idling to keep the latched markers alive.",
            ),
            OpaqueFunction(function=_tracer),
        ]
    )

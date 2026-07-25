"""
Bring up the faked xArm 7 (move_group + ros2_control + RViz) and trace the
shapes described by a YAML file.

    ros2 launch avatar_challenge start.launch.py
    ros2 launch avatar_challenge start.launch.py shapes_file:=/abs/path/to/my_shapes.yaml
    ros2 launch avatar_challenge start.launch.py plan_only:=true
    ros2 launch avatar_challenge start.launch.py autostart:=false   # sim only

RViz
----
The upstream xarm7_moveit_fake launch always starts its own RViz with the stock
MoveIt config, which has no MarkerArray display, so the shape outlines and shape
frames would be invisible. With `use_custom_rviz:=true` (the default) that node
is suppressed and RViz is started here instead with this package's config.
Pass `use_custom_rviz:=false` to get the stock MoveIt RViz back.
"""

from avatar_challenge_launch.moveit_params import (
    robot_description_params,
    rviz_params,
)
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def _truthy(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1")


def _moveit_clients(context):
    """RViz and the tracer, both parameterised with the MoveIt config."""
    actions = []

    if _truthy(context, "use_custom_rviz"):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("avatar_challenge"),
                        "rviz",
                        "shape_tracer.rviz",
                    ),
                ],
                parameters=[rviz_params(context)],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            )
        )

    if _truthy(context, "autostart"):
        # move_group has to be up and advertising, and the fake controllers have
        # to be spawned, before MoveGroupInterface can be constructed.
        actions.append(
            TimerAction(
                period=float(LaunchConfiguration("trace_delay").perform(context)),
                actions=[
                    Node(
                        package="avatar_challenge",
                        executable="shape_tracer",
                        name="shape_tracer",
                        output="screen",
                        parameters=[
                            LaunchConfiguration("params_file").perform(context),
                            robot_description_params(context),
                            {
                                "shapes_file": LaunchConfiguration(
                                    "shapes_file"
                                ).perform(context),
                                "plan_only": _truthy(context, "plan_only"),
                            },
                        ],
                    )
                ],
            )
        )

    return actions


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(
            "shapes_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("avatar_challenge"), "config", "shapes.yaml"]
            ),
            description="YAML file describing the shapes to trace.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("avatar_challenge"), "config", "shape_tracer.yaml"]
            ),
            description="YAML file with shape_tracer node parameters.",
        ),
        DeclareLaunchArgument(
            "use_custom_rviz",
            default_value="true",
            description="Use this package's RViz config (shows shape markers) "
            "instead of the stock xarm_moveit_config one.",
        ),
        DeclareLaunchArgument(
            "plan_only",
            default_value="false",
            description="Plan and visualise without commanding the arm.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Start tracing automatically. Set false to bring up the "
            "simulation only and run trace_shapes.launch.py by hand.",
        ),
        DeclareLaunchArgument(
            "trace_delay",
            default_value="12.0",
            description="Seconds to wait for move_group before tracing starts.",
        ),
    ]

    # xarm_moveit_config's RViz node is gated on the `show_rviz` launch
    # configuration, which nothing along the include chain overrides -- setting
    # it here propagates into the include and suppresses that node.
    suppress_stock_rviz = SetLaunchConfiguration(
        "show_rviz", "false", condition=IfCondition(LaunchConfiguration("use_custom_rviz"))
    )

    xarm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("xarm_moveit_config"),
                    "launch",
                    "xarm7_moveit_fake.launch.py",
                ]
            )
        ),
    )

    return LaunchDescription(
        declared
        + [
            suppress_stock_rviz,
            xarm_moveit_launch,
            OpaqueFunction(function=_moveit_clients),
        ]
    )

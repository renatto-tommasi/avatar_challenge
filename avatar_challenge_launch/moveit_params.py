"""
Rebuild the xArm 7 MoveIt configuration for nodes that need it.

move_group receives its robot description, SRDF, kinematics solver and planning
limits as *parameters*, not on a topic, so any process that constructs a
RobotModel of its own -- MoveGroupInterface in the tracer, and RViz's MoveIt
displays -- has to be handed the same set.

xarm_moveit_config builds that set inside `_robot_moveit_fake.launch.py` and
never exports it, so this helper repeats the same MoveItConfigsBuilder call with
the same arguments the fake launch uses (dof 7, xarm, fake hardware plugin,
limited joint ranges). Keeping the arguments identical is what guarantees the
tracer plans against exactly the robot move_group is executing on.
"""

import os

from ament_index_python import get_package_share_directory
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file

# Must match _robot_moveit_fake.launch.py, which xarm7_moveit_fake.launch.py
# invokes with dof=7 / robot_type=xarm and the defaults for everything else.
_DOF = 7
_ROBOT_TYPE = "xarm"
_HW_NS = "xarm"
_LIMITED = True
_FAKE_PLUGIN = "uf_robot_hardware/UFRobotFakeSystemHardware"

_cache = {}


def moveit_config_dict(context):
    """Full MoveIt config as a parameter dict. Cached per launch process."""
    if "config" in _cache:
        return _cache["config"]

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(
            get_package_share_directory("xarm_controller"),
            "config",
            "{}{}_controllers.yaml".format(_ROBOT_TYPE, _DOF),
        ),
        prefix="",
        add_gripper=False,
        add_bio_gripper=False,
        ros_namespace="",
        robot_type=_ROBOT_TYPE,
    )

    config = (
        MoveItConfigsBuilder(
            context=context,
            controllers_name="fake_controllers",
            dof=_DOF,
            robot_type=_ROBOT_TYPE,
            prefix="",
            hw_ns=_HW_NS,
            limited=_LIMITED,
            ros2_control_plugin=_FAKE_PLUGIN,
            ros2_control_params=ros2_control_params,
        )
        .to_moveit_configs()
        .to_dict()
    )
    _cache["config"] = config
    return config


def robot_description_params(context):
    """The subset a MoveGroupInterface client needs to build a RobotModel.

    `robot_description_planning` carries joint_limits.yaml, which is what the
    time-optimal retimer reads its velocity and acceleration ceilings from, so
    dropping it would silently retime against the URDF limits instead.
    """
    config = moveit_config_dict(context)
    return {
        key: config[key]
        for key in (
            "robot_description",
            "robot_description_semantic",
            "robot_description_kinematics",
            "robot_description_planning",
        )
    }


def rviz_params(context):
    """What RViz's MoveIt displays need on top of the above."""
    config = moveit_config_dict(context)
    params = robot_description_params(context)
    params["planning_pipelines"] = config["planning_pipelines"]
    params["use_sim_time"] = False
    return params

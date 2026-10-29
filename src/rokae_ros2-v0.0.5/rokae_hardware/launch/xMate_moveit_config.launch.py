import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_LAUNCH_DIR = Path(__file__).resolve().parent
if str(_LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(_LAUNCH_DIR))
from moveit_planning_utils import (
    extract_ros_parameters,
    load_robot_description_planning,
    load_yaml_file,
)


def launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    local_ip = LaunchConfiguration("local_ip").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    enable_movej = LaunchConfiguration("enable_movej").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    default_velocity_scaling = float(
        LaunchConfiguration("default_velocity_scaling_factor").perform(context)
    )
    default_acceleration_scaling = float(
        LaunchConfiguration("default_acceleration_scaling_factor").perform(context)
    )

    if use_fake_hardware.lower() in ("1", "true", "yes"):
        use_sim_time = False
    elif (
        robot_type.startswith(("CR", "AR", "ER", "Pro", "SR"))
        or robot_type in ("XB7s", "XB7h", "NB25s", "NB25h", "NB12s", "NB12h", "EB4", "NB4", "XB4s", "XB4h", "NB220s")
    ) and use_fake_hardware.lower() in ("0", "false", "no"):
        use_sim_time = False

    description_pkg = get_package_share_directory("rokae_description")
    urdf_file = os.path.join(description_pkg, "urdf", "xMate.urdf.xacro")

    moveit_config_pkg_name = f"rokae_xMate{robot_type}_moveit_config"
    moveit_config_pkg_share = get_package_share_directory(moveit_config_pkg_name)

    srdf_file = os.path.join(
        moveit_config_pkg_share, "config", f"xMate{robot_type}.srdf"
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    urdf_file,
                    " robot_type:=",
                    robot_type,
                    " robot_ip:=",
                    robot_ip,
                    " local_ip:=",
                    local_ip,
                    " use_fake_hardware:=",
                    use_fake_hardware,
                ]
            ),
            value_type=str,
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(["cat ", srdf_file]),
            value_type=str,
        )
    }

    robot_description_kinematics = {
        "robot_description_kinematics": extract_ros_parameters(
            load_yaml_file(moveit_config_pkg_name, "config/kinematics.yaml")
        )
    }

    robot_description_planning = load_robot_description_planning(
        moveit_config_pkg_name,
        default_velocity_scaling_factor=default_velocity_scaling,
        default_acceleration_scaling_factor=default_acceleration_scaling,
    )

    ompl_yaml = extract_ros_parameters(
        load_yaml_file(moveit_config_pkg_name, "config/ompl_planning.yaml")
    )
    ompl_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
            "start_state_max_bounds_error": 0.1,
        }
    }
    if ompl_yaml:
        ompl_pipeline_config["ompl"].update(ompl_yaml)

    controllers_yaml = load_yaml_file(
        moveit_config_pkg_name, "config/simple_moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "execution_duration_monitoring": False,
        "trajectory_execution.wait_for_trajectory_completion": True,
        "trajectory_execution.update_state_after_execution": True,
        "trajectory_execution.update_state_before_execution": True,
        "manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 5.0,
        "trajectory_execution.allowed_goal_duration_margin": 2.0,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    shared_moveit_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_pipeline_config,
        moveit_controllers,
        trajectory_execution,
        planning_scene_monitor_parameters,
        {"use_sim_time": use_sim_time},
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=shared_moveit_params,
    )

    rviz_config_file = os.path.join(moveit_config_pkg_share, "rviz", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=shared_moveit_params,
    )

    actions = [move_group_node, rviz_node]
    if enable_movej:
        movej_node = Node(
            package="rokae_hardware",
            executable="movej",
            name="movej",
            parameters=[*shared_moveit_params, {"auto_execute": True}],
        )
        actions.append(movej_node)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_type",
                default_value="CR7",
                description="Robot type for rokae_xMate{robot_type}_moveit_config.",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.21.10",
                description="Robot controller IP.",
            ),
            DeclareLaunchArgument(
                "local_ip",
                default_value="192.168.21.131",
                description="This PC IP on the robot subnet.",
            ),
            DeclareLaunchArgument(
                "warehouse_sqlite_path",
                default_value="",
                description="Warehouse path",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="false",
                description="Forwarded to xacro (mock vs real hardware plugin).",
            ),
            DeclareLaunchArgument(
                "enable_movej",
                default_value="false",
                description=(
                    "Start demo movej node (auto-plans to [0.5 rad]*6 on launch). "
                    "Keep false for normal MoveIt+RViz; only enable for trajectory demo."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time only when /clock exists.",
            ),
            DeclareLaunchArgument(
                "default_velocity_scaling_factor",
                default_value="0.1",
                description=(
                    "MoveIt default velocity scale (0~1). RViz slider multiplies "
                    "joint_limits max_velocity by this factor at plan time."
                ),
            ),
            DeclareLaunchArgument(
                "default_acceleration_scaling_factor",
                default_value="0.1",
                description="MoveIt default acceleration scale (0~1).",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

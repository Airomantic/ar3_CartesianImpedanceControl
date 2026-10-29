"""Helpers for loading MoveIt robot_description_planning parameters."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory


def load_yaml_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except OSError:
        return None


def extract_ros_parameters(yaml_data):
    """Unwrap /**/ros__parameters from ROS2 param YAML exports."""
    if not yaml_data:
        return {}
    if "/**" in yaml_data and isinstance(yaml_data["/**"], dict):
        return dict(yaml_data["/**"].get("ros__parameters", {}))
    if "ros__parameters" in yaml_data:
        return dict(yaml_data["ros__parameters"])
    return dict(yaml_data)


def load_robot_description_planning(
    moveit_config_pkg_name,
    default_velocity_scaling_factor=0.1,
    default_acceleration_scaling_factor=0.1,
):
    """Build robot_description_planning dict for move_group / RViz."""
    planning = extract_ros_parameters(
        load_yaml_file(moveit_config_pkg_name, "config/joint_limits.yaml")
    )
    cartesian = extract_ros_parameters(
        load_yaml_file(moveit_config_pkg_name, "config/cartesian_limits.yaml")
    )
    if cartesian:
        planning.update(cartesian)

    planning["default_velocity_scaling_factor"] = float(default_velocity_scaling_factor)
    planning["default_acceleration_scaling_factor"] = float(
        default_acceleration_scaling_factor
    )
    return {"robot_description_planning": planning}

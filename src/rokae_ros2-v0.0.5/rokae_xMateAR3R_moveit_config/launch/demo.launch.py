from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("AR3-3_07R-W4C5C5", package_name="rokae_xMateAR3R_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)

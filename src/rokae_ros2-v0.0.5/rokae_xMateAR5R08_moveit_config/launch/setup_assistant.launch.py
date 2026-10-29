from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("AR5-5_08R-W4C1C5", package_name="rokae_xMateAR5R08_moveit_config").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)

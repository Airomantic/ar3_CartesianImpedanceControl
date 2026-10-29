from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import re
import yaml
import xacro
import xml.etree.ElementTree as ET
from pathlib import Path


def _extract_joint_names(controllers_cfg):
    for key in (
        "arm_controller",
        "position_joint_trajectory_controller",
        "joint_state_broadcaster",
    ):
        params = controllers_cfg.get(key, {}).get("ros__parameters", {})
        joints = params.get("joints", [])
        if isinstance(joints, list) and joints:
            return joints
    return []


def _select_trajectory_controller_name(controllers_cfg):
    cm_cfg = controllers_cfg.get("controller_manager", {}).get("ros__parameters", {})
    if "arm_controller" in cm_cfg:
        return "arm_controller"
    if "position_joint_trajectory_controller" in cm_cfg:
        return "position_joint_trajectory_controller"
    return "arm_controller"


def _find_robot_base_link(root):
    """Pick the kinematic root (never a joint child), preferring *_base names."""
    child_links = set()
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is not None and child.attrib.get("link"):
            child_links.add(child.attrib["link"])
    candidates = []
    for link in root.findall("link"):
        name = link.attrib.get("name", "")
        if not name or name == "world":
            continue
        if name not in child_links:
            candidates.append(name)
    if not candidates:
        return None
    for name in candidates:
        if name.endswith("_base") or name in ("base_link", "base"):
            return name
    return candidates[0]


def _ensure_world_fixed_base(robot_desc_xml: str) -> str:
    """Keep/add world→base fixed joint so collision forces do not tip the whole arm."""
    root = ET.fromstring(robot_desc_xml)
    has_world = any(link.attrib.get("name") == "world" for link in root.findall("link"))
    has_world_fixed = False
    for joint in root.findall("joint"):
        if joint.attrib.get("type") != "fixed":
            continue
        parent = joint.find("parent")
        if parent is not None and parent.attrib.get("link") == "world":
            has_world_fixed = True
            break
    if has_world and has_world_fixed:
        return robot_desc_xml

    base_link = _find_robot_base_link(root)
    if not base_link:
        return robot_desc_xml

    if not has_world:
        # Insert world link first so it is the model root for Gazebo.
        world_link = ET.Element("link", {"name": "world"})
        root.insert(0, world_link)

    if not has_world_fixed:
        fixed = ET.Element("joint", {"name": "fixed", "type": "fixed"})
        ET.SubElement(fixed, "parent", {"link": "world"})
        ET.SubElement(fixed, "child", {"link": base_link})
        ET.SubElement(fixed, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        # Place immediately after world link if present.
        insert_at = 1 if any(
            c.tag == "link" and c.attrib.get("name") == "world" for c in list(root)[:2]
        ) else 0
        root.insert(insert_at, fixed)

    return ET.tostring(root, encoding="unicode")


def _inject_gazebo_blocks(robot_desc_xml, robot_type, controllers_param_path, joint_names):
    root = ET.fromstring(robot_desc_xml)

    for joint_name in joint_names:
        gz_joint = ET.SubElement(root, "gazebo", {"reference": joint_name})
        ET.SubElement(gz_joint, "kp").text = "1000.0"
        ET.SubElement(gz_joint, "kd").text = "100.0"
        ET.SubElement(gz_joint, "mu1").text = "0.1"
        ET.SubElement(gz_joint, "mu2").text = "0.1"
        ET.SubElement(gz_joint, "friction1").text = "0.0"
        ET.SubElement(gz_joint, "friction2").text = "0.0"

    for link in root.findall("link"):
        link_name = link.attrib.get("name", "")
        if not link_name or link_name == "world":
            continue
        gz_link = ET.SubElement(root, "gazebo", {"reference": link_name})
        ET.SubElement(gz_link, "material").text = "Gazebo/Blue"
        collision = ET.SubElement(gz_link, "collision")
        ET.SubElement(collision, "laser_retro").text = "0.5"

    # Some legacy models contain inconsistent transmission joint names.
    # Rebuild transmissions from controller joint list to keep gazebo_ros2_control stable.
    for old_transmission in list(root.findall("transmission")):
        root.remove(old_transmission)
    for idx, joint_name in enumerate(joint_names, start=1):
        transmission = ET.SubElement(root, "transmission", {"name": f"tran{idx}"})
        ET.SubElement(transmission, "type").text = "transmission_interface/SimpleTransmission"
        transmission_joint = ET.SubElement(transmission, "joint", {"name": joint_name})
        ET.SubElement(transmission_joint, "hardwareInterface").text = "PositionJointInterface"
        actuator = ET.SubElement(transmission, "actuator", {"name": f"motor{idx}"})
        ET.SubElement(actuator, "hardwareInterface").text = "PositionJointInterface"
        ET.SubElement(actuator, "mechanicalReduction").text = "1"

    ros2_control = ET.SubElement(
        root,
        "ros2_control",
        {"name": f"xMate{robot_type}_system", "type": "system"},
    )
    hardware = ET.SubElement(ros2_control, "hardware")
    ET.SubElement(hardware, "plugin").text = "gazebo_ros2_control/GazeboSystem"
    ET.SubElement(hardware, "param", {"name": "use_sim_time"}).text = "true"

    for joint_name in joint_names:
        joint = ET.SubElement(ros2_control, "joint", {"name": joint_name})
        ET.SubElement(joint, "command_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "velocity"})

    gazebo = ET.SubElement(root, "gazebo")
    plugin = ET.SubElement(
        gazebo,
        "plugin",
        {"name": "gazebo_ros2_control", "filename": "libgazebo_ros2_control.so"},
    )
    ET.SubElement(plugin, "robot_param").text = "robot_description"
    ET.SubElement(plugin, "robot_param_node").text = "robot_state_publisher"
    ET.SubElement(plugin, "parameters").text = controllers_param_path

    return ET.tostring(root, encoding="unicode")


def _strip_xml_comments_for_gazebo_ros2(robot_desc_xml: str) -> str:
    """gazebo_ros2_control may pass URDF through rcl param rules; ':' + space in
    <!-- comments --> can be misparsed (see gz_ros2_control #503). Strip comments."""
    return re.sub(r"<!--(.*?)-->", "", robot_desc_xml, flags=re.DOTALL)


def _resolve_gazebo_controller_params_path(robot_desc_xml, controllers_yaml_path):
    """Gazebo 插件需绝对路径加载控制器 YAML（与 CR7 一致）。"""
    abs_path = os.path.abspath(controllers_yaml_path)
    return re.sub(
        r"(<parameters>)([^<]*controllers\.yaml)(</parameters>)",
        rf"\1{abs_path}\3",
        robot_desc_xml,
        count=1,
    )


def launch_setup(context, *args, **kwargs):
    logger = get_logger("test_model.launch")
    robot_type = LaunchConfiguration("robot_type").perform(context)
    gazebo_world_file = LaunchConfiguration("gazebo_world_file").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    # gui = joint-slider GUI only; gazebo_gui = gzclient 3D window (must not share one flag)
    gui = LaunchConfiguration("gui").perform(context)
    gazebo_gui = LaunchConfiguration("gazebo_gui").perform(context)

    supported_types = [
        "CR7", "CR12", "CR18", "CR20", "CR35",
        "ER3", "ER7", "SR3", "SR4", "SR5",
        "Pro3", "Pro7", "AR5L", "AR5R", "AR5R08", "AR5R08ZY2", "AR5L08", "AR5L08ZY2", "AR3R", "AR3L",
        "XB7s", "XB7h", "NB25s", "NB25h", "NB12s", "NB12h", "EB4", "NB4", "XB4s", "XB4h", "NB220s",
    ]
    if robot_type not in supported_types:
        raise RuntimeError(f"Unsupported robot type: {robot_type}")

    description_pkg = get_package_share_directory("rokae_description")
    custom_model_base = {
        "AR5L": "AR5-5_07L-W4C4A2",
        "AR5R": "AR5-5_07R-W4C4A2",
        "AR5R08": "AR5-5_08R-W4C1C5",
        "AR5R08ZY2": "AR5-5_08R-W4C1C9-ZY2",
        "AR5L08": "AR5-5_08L-W4C1C5",
        "AR5L08ZY2": "AR5-5_08L-W4C1C9-ZY2",
        "AR3R": "AR3-3_07R-W4C5C5",
        "AR3L": "AR3-3_07L-W4C5C5",
    }
    custom_gazebo_urdf_names = {
        "AR5L": "AR5-5_07L-W4C4A2.urdf.xacro_Gazebo.urdf.xacro",
        "AR5R": "AR5-5_07R-W4C4A2.urdf.xacro_Gazebo.urdf.xacro",
        "AR5L08": "AR5-5_08L-W4C1C5.urdf.xacro_Gazebo.urdf.xacro",
        "AR5L08ZY2": "AR5-5_08L-W4C1C9-ZY2.urdf.xacro_Gazebo.urdf.xacro",
        "AR5R08": "AR5-5_08R-W4C1C5.urdf.xacro_Gazebo.urdf.xacro",
        "AR5R08ZY2": "AR5-5_08R-W4C1C9-ZY2.urdf.xacro_Gazebo.urdf.xacro",
        "AR3L": "AR3-3_07L-W4C5C5.urdf.xacro_Gazebo.urdf.xacro",
        "AR3R": "AR3-3_07R-W4C5C5.urdf.xacro_Gazebo.urdf.xacro",
    }
    gazebo_urdf_filename = custom_gazebo_urdf_names.get(
        robot_type, f"xMate{robot_type}_Gazebo.urdf.xacro"
    )
    gazebo_urdf_path = os.path.join(description_pkg, "urdf", gazebo_urdf_filename)
    use_model_gazebo_xacro = os.path.exists(gazebo_urdf_path)

    if use_model_gazebo_xacro:
        urdf_path = gazebo_urdf_path
    else:
        model_base = custom_model_base.get(robot_type, f"xMate{robot_type}")
        urdf_path = os.path.join(description_pkg, "urdf", f"{model_base}.urdf.xacro")
    if not os.path.exists(urdf_path):
        raise RuntimeError(f"URDF not found: {urdf_path}")

    controllers_yaml_path = os.path.join(
        get_package_share_directory("rokae_hardware"),
        "config",
        f"xMate{robot_type}_controllers.yaml",
    )
    if not os.path.exists(controllers_yaml_path):
        raise RuntimeError(f"Controller config not found: {controllers_yaml_path}")
    with open(controllers_yaml_path, "r", encoding="utf-8") as f:
        controllers_cfg = yaml.safe_load(f) or {}
    joint_names = _extract_joint_names(controllers_cfg)
    if not joint_names:
        raise RuntimeError(
            f"Could not determine controlled joints from {controllers_yaml_path}"
        )
    trajectory_controller_name = _select_trajectory_controller_name(controllers_cfg)
    trajectory_topic = f"/{trajectory_controller_name}/joint_trajectory"
    follow_joint_trajectory_action = (
        f"/{trajectory_controller_name}/follow_joint_trajectory"
    )

    mesh_prefix = (Path(description_pkg) / "meshes").as_uri()
    robot_desc_xml = xacro.process_file(urdf_path, mappings={"mesh_prefix": mesh_prefix}).toxml()
    if not use_model_gazebo_xacro:
        robot_desc_xml = _inject_gazebo_blocks(
            robot_desc_xml=robot_desc_xml,
            robot_type=robot_type,
            controllers_param_path=controllers_yaml_path,
            joint_names=joint_names,
        )
    else:
        robot_desc_xml = _resolve_gazebo_controller_params_path(
            robot_desc_xml, controllers_yaml_path
        )

    robot_desc_xml = _strip_xml_comments_for_gazebo_ros2(robot_desc_xml)
    # Must keep world→base fixed joint; otherwise collision reaction tips/flies the whole model.
    robot_desc_xml = _ensure_world_fixed_base(robot_desc_xml)
    logger.info(
        f"robot_type={robot_type}, use_model_gazebo_xacro={use_model_gazebo_xacro}, "
        f"urdf_path={urdf_path}, gazebo_gui={gazebo_gui}, joint_gui={gui}"
    )

    gazebo_ros_share_dir = get_package_share_directory("gazebo_ros")
    rokae_gazebo_share_dir = get_package_share_directory("rokae_gazebo")
    world_path = os.path.join(rokae_gazebo_share_dir, "worlds", gazebo_world_file)
    # Start paused: industrial arms (NB*) fall under gravity in the ~1s window before
    # gazebo_ros2_control claims POSITION and SetPosition can hold joints.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            # Always independent of joint-slider `gui` (see controll_movej: Gazebo GUI stays on).
            "gui": gazebo_gui,
            "pause": "true",
            # -u also forces pause if nested gazebo.launch does not forward `pause`.
            "extra_gazebo_args": "--verbose -u",
        }.items()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc_xml}, {"use_sim_time": True}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", f"xMate{robot_type}",
            "-x", "0.0", "-y", "0.0", "-z", spawn_z,
        ],
        output="screen",
    )

    # Load controllers required for joint motion validation.
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--service-call-timeout",
            "120",
        ],
        output="screen",
    )
    spawn_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            trajectory_controller_name,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--service-call-timeout",
            "120",
        ],
        output="screen",
    )

    # Controllers while paused, then unpause so the arm never free-falls.
    delayed_joint_state_spawner = TimerAction(
        period=0.5,
        actions=[spawn_joint_state_broadcaster],
    )
    delayed_trajectory_spawner = TimerAction(
        period=1.2,
        actions=[spawn_trajectory_controller],
    )
    unpause_physics = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/unpause_physics",
            "std_srvs/srv/Empty",
            "{}",
        ],
        output="screen",
    )
    delayed_unpause = TimerAction(period=2.0, actions=[unpause_physics])

    actions = [
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delayed_joint_state_spawner,
        delayed_trajectory_spawner,
        delayed_unpause,
    ]

    if gui.lower() in ("true", "1", "yes"):
        # JSP GUI: wall clock (user input). Bridge uses sim time for controller sync.
        joint_state_publisher_gui = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            parameters=[
                {"robot_description": robot_desc_xml},
                {"use_sim_time": False},
                # Match Gazebo/ros2_control initial_value (upright home = all zeros).
                {
                    "zeros": {
                        "joint1": 0.0,
                        "joint2": 0.0,
                        "joint3": 0.0,
                        "joint4": 0.0,
                        "joint5": 0.0,
                        "joint6": 0.0,
                    }
                },
            ],
            remappings=[("/joint_states", "/joint_states_gui")],
        )
        # Topic streaming is more reliable than preempting FollowJointTrajectory every 100ms.
        gui_control_node = Node(
            package="rokae_description",
            executable="gui_to_joint_trajectory.py",
            name="gui_to_joint_trajectory",
            output="screen",
            parameters=[
                {"use_sim_time": True},
                {"joint_names": joint_names},
                {"trajectory_topic": trajectory_topic},
                {"follow_joint_trajectory_action": follow_joint_trajectory_action},
                {"use_follow_joint_trajectory_action": False},
                # Longer/smoother than 0.25s@10Hz overlapping cmds (reduces Gazebo jitter).
                {"trajectory_duration": 0.5},
                {"publish_period": 0.2},
            ],
        )
        delayed_gui = TimerAction(
            period=2.5,
            actions=[joint_state_publisher_gui, gui_control_node],
        )
        actions.append(delayed_gui)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_type",
            default_value="CR7",
            description=(
                "机型后缀，对应 urdf/xMate{TYPE}_*.xacro 与 "
                "rokae_hardware/config/xMate{TYPE}_controllers.yaml。例 CR35、CR7、SR3、Pro3、AR5L08。"
            ),
        ),
        DeclareLaunchArgument(
            "gazebo_world_file",
            default_value="empty.world",
            description=(
                "rokae_gazebo/worlds 下文件名（empty.world、obstacles.world）或绝对路径；与机型无关。"
            ),
        ),
        DeclareLaunchArgument("spawn_z", default_value="0.0"),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="是否启动 Gazebo 客户端窗口（gzclient）。与关节滑块 gui 独立。",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description=(
                "是否启动 joint_state_publisher_gui 滑块 + gui_to_joint_trajectory。"
                "不影响 Gazebo 3D 窗口（见 gazebo_gui）。"
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])

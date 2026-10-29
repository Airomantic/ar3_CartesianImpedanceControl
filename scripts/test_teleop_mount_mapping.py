"""Offline side-mount regression: production mapping and IK, no ROS node or I/O."""

import importlib.util
import os
from pathlib import Path
from types import MethodType, SimpleNamespace
import unittest
import warnings
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import pink
from pink.limits import ConfigurationLimit, VelocityLimit
from pink.tasks import DampingTask, FrameTask, PostureTask
import yaml

WS = Path(__file__).resolve().parents[1]
BRIDGE = WS.parent / 'remote/pp_ubuntu24.04_V1.1.2.38_202606301404_release_ros2_x86_jazzy/teleop_bridge'
spec = importlib.util.spec_from_file_location('ar3r_teleop', BRIDGE / 'ar3r_cartesian_retargeting.py')
teleop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(teleop)
config_path = Path(os.environ.get('AR3_MOUNT_TEST_CONFIG', BRIDGE / 'ar3r_cartesian_retargeting.yaml'))
PARAMS = yaml.safe_load(config_path.read_text())['ar3r_cartesian_retargeting']['ros__parameters']

# Columns are robot base +X/+Y/+Z expressed in site forward/left/up coordinates.
# This is the physical mounting described by the user, independently of RPY code.
SITE_FROM_BASE = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
MOTIONS = {
    'forward': ([1, 0, 0], [1, 0, 0]),
    'back': ([-1, 0, 0], [-1, 0, 0]),
    'up': ([0, 0, 1], [0, 1, 0]),
    'down': ([0, 0, -1], [0, -1, 0]),
    'left': ([0, 1, 0], [0, 0, -1]),
    'right': ([0, -1, 0], [0, 0, 1]),
}


class MountMappingTests(unittest.TestCase):
    def setUp(self):
        self.correction = pin.rpy.rpyToMatrix(*np.deg2rad(PARAMS['target_direction_rpy_deg']))
        self.source = pin.SE3(pin.rpy.rpyToMatrix(.3, -.4, .7), np.array([.7, -.2, .5]))
        self.robot = pin.SE3(pin.rpy.rpyToMatrix(-.2, .1, -.8), np.array([.3, .4, .6]))

    def mapped(self, target):
        return teleop.reorient_target_delta(target, self.source, self.robot, self.correction)

    def test_mount_config_and_six_translations(self):
        self.assertTrue(PARAMS['dry_run'])
        np.testing.assert_allclose(SITE_FROM_BASE @ self.correction, np.eye(3), atol=1e-12)
        for name, (source_axis, base_axis) in MOTIONS.items():
            with self.subTest(direction=name):
                target = self.source.copy()
                target.translation += .03 * np.array(source_axis)
                np.testing.assert_allclose(self.mapped(target).translation - self.robot.translation,
                                           .03 * np.array(base_axis), atol=1e-12)

    def test_spatial_rotations_preserve_site_direction(self):
        for name, (source_axis, _) in MOTIONS.items():
            with self.subTest(direction=name):
                delta_site = pin.exp3(.15 * np.array(source_axis))
                target = pin.SE3(delta_site @ self.source.rotation, self.source.translation)
                expected_site = delta_site @ SITE_FROM_BASE @ self.robot.rotation
                np.testing.assert_allclose(SITE_FROM_BASE @ self.mapped(target).rotation,
                                           expected_site, atol=1e-12)

    def test_fresh_target_and_reclutch_have_no_initial_jump(self):
        for robot in (self.robot, pin.SE3(pin.rpy.rpyToMatrix(.6, -.5, .1), np.array([.2, -.3, .4]))):
            state = SimpleNamespace(latest_target_in_base=self.source.copy(),
                                    direction_correction_enabled=True,
                                    source_anchor_pose=None, target_sequence=10,
                                    takeover_target_sequence=10, anchor_pose_in_base=robot,
                                    direction_correction=self.correction)
            self.assertIsNone(teleop.AR3RCartesianRetargeting._mapped_target(state))
            state.target_sequence += 1
            result = teleop.AR3RCartesianRetargeting._mapped_target(state)
            np.testing.assert_allclose(result.homogeneous, robot.homogeneous, atol=1e-12)

    def test_production_ik_preview_tracks_six_small_motions_without_commands(self):
        source_xml = ET.parse(WS / 'src/rokae_ros2-v0.0.5/rokae_description/urdf/AR3-3_07R-W4C5C5.urdf.xacro').getroot()
        # This model file has static links/joints; geometry and transmissions are irrelevant to FK.
        robot_xml = ET.Element('robot', name='ar3_mount_test')
        robot_xml.extend(child for child in source_xml if child.tag in ('link', 'joint'))
        for link in robot_xml.findall('link'):
            for child in list(link):
                if child.tag in ('visual', 'collision'):
                    link.remove(child)
        for name, (source_axis, base_axis) in MOTIONS.items():
            with self.subTest(direction=name):
                model = pin.buildModelFromXML(ET.tostring(robot_xml, encoding='unicode'))
                q0 = np.array([1.586, -1.337, -1.418, 1.049, -.615, -.061, -.445])
                configuration = pink.Configuration(model, model.createData(), q0.copy())
                ee = PARAMS['end_effector_frame']
                base = PARAMS['base_frame']
                anchor = configuration.get_transform_frame_to_world(base).inverse() * configuration.get_transform_frame_to_world(ee)
                tasks = {'ee': FrameTask(ee, position_cost=PARAMS['position_cost'], orientation_cost=PARAMS['orientation_cost']),
                         'posture': PostureTask(cost=PARAMS['posture_cost']),
                         'damping': DampingTask(cost=PARAMS['damping_cost'])}
                tasks['posture'].set_target(q0)
                commands, previews, targets = [], [], []
                state = SimpleNamespace(model=model, configuration=configuration, tasks=tasks,
                    q_commanded=q0.copy(), last_velocity=np.zeros(7),
                    latest_target_in_base=self.source.copy(), source_anchor_pose=self.source.copy(),
                    anchor_pose_in_base=anchor, direction_correction=self.correction,
                    direction_correction_enabled=True, base_frame=base,
                    limits=[ConfigurationLimit(model), VelocityLimit(model)],
                    soft_start_elapsed=0., q_indices=list(range(7)), v_indices=list(range(7)),
                    max_velocity=np.array(PARAMS['max_velocity_rad_s']),
                    max_acceleration=np.array(PARAMS['max_acceleration_rad_s2']),
                    position_min=np.array(PARAMS['position_min_rad']),
                    position_max=np.array(PARAMS['position_max_rad']),
                    last_status_log=float('inf'),
                    command_pub=SimpleNamespace(publish=commands.append, get_subscription_count=lambda: 1),
                    get_parameter=lambda key: SimpleNamespace(value=PARAMS[key]),
                    _publish_mapped_target=lambda pose: targets.append(pose.copy()),
                    _publish_preview=lambda positions, velocities: previews.append(list(positions)))
                state._mapped_target = MethodType(teleop.AR3RCartesianRetargeting._mapped_target, state)
                with warnings.catch_warnings():
                    warnings.filterwarnings('ignore', message='Converted matrix')
                    warnings.filterwarnings('ignore', category=PendingDeprecationWarning, module='osqp')
                    for step in range(300):
                        target = self.source.copy()
                        target.translation += .01 * min(1., (step + 1) / 200.) * np.array(source_axis)
                        state.latest_target_in_base = target
                        teleop.AR3RCartesianRetargeting._solve_and_publish(state, .01, True, step * .01)
                self.assertEqual(commands, [])
                self.assertEqual(len(previews), 300)
                np.testing.assert_allclose(targets[-1].translation - anchor.translation,
                                           .01 * np.array(base_axis), atol=1e-12)
                configuration.update(state.q_commanded)
                actual = configuration.get_transform_frame_to_world(base).inverse() * configuration.get_transform_frame_to_world(ee)
                np.testing.assert_allclose(actual.translation - anchor.translation,
                                           .01 * np.array(base_axis), atol=.001)


if __name__ == '__main__':
    unittest.main(verbosity=2)

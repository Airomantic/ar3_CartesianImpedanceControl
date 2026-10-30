"""Offline checks for clutch anchoring, mounting rotation and pose rate limits."""
import unittest

import numpy as np

from cartesian_teleop_math import limit_pose, map_pose, matrix_rpy, quaternion_matrix, rpy_matrix


class CartesianMappingTest(unittest.TestCase):
    def test_takeover_has_no_absolute_pose_jump(self):
        source = (np.array([10., 20., 30.]), rpy_matrix([.1, -.3, .4]))
        robot = (np.array([.3, .1, .4]), rpy_matrix([.8, -.7, .2]))
        correction = rpy_matrix([-np.pi/2, 0., 0.])
        p, r = map_pose(source, source, robot, correction)
        np.testing.assert_allclose(p, robot[0])
        np.testing.assert_allclose(r, robot[1], atol=1e-12)

    def test_side_mount_six_directions_with_different_source_and_robot_origins(self):
        source = (np.array([1., 2., 3.]), rpy_matrix([.3, -.4, .5]))
        robot = (np.array([.2, -.1, -.3]), rpy_matrix([2.7, .08, 1.8]))
        directions = (
            ("forward", [1, 0, 0], [1, 0, 0]),
            ("back", [-1, 0, 0], [-1, 0, 0]),
            ("up", [0, 0, 1], [0, 1, 0]),
            ("down", [0, 0, -1], [0, -1, 0]),
            ("right", [0, -1, 0], [0, 0, 1]),
            ("left", [0, 1, 0], [0, 0, -1]),
        )
        correction = rpy_matrix([-np.pi/2, 0., 0.])
        for name, source_axis, base_axis in directions:
            with self.subTest(direction=name):
                target = (source[0] + .03*np.array(source_axis), source[1])
                p, r = map_pose(target, source, robot, correction)
                np.testing.assert_allclose(p-robot[0], .03*np.array(base_axis), atol=1e-12)
                np.testing.assert_allclose(r, robot[1], atol=1e-12)

    def test_side_mount_spatial_rotations_about_each_physical_axis(self):
        source = (np.array([1., 2., 3.]), rpy_matrix([.3, -.4, .5]))
        robot = (np.array([.2, -.1, -.3]), rpy_matrix([2.7, .08, 1.8]))
        # Columns are physical base axes expressed in source forward/left/up.
        source_from_base = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
        correction = rpy_matrix([-np.pi/2, 0., 0.])
        for axis in range(3):
            for sign in (-1., 1.):
                delta = rpy_matrix(sign*.1*np.eye(3)[axis])
                target = (source[0], delta @ source[1])
                p, r = map_pose(target, source, robot, correction)
                np.testing.assert_allclose(p, robot[0], atol=1e-12)
                np.testing.assert_allclose(source_from_base @ r,
                                           delta @ source_from_base @ robot[1], atol=1e-12)

    def test_limits_translation_and_rotation_through_euler_wrap(self):
        previous = (np.zeros(3), rpy_matrix([0., 0., np.pi-.01]))
        target = (np.array([1., 0., 0.]), rpy_matrix([0., 0., -np.pi+.01]))
        p, r = limit_pose(previous, target, .001, .005)
        np.testing.assert_allclose(p, [.001, 0., 0.])
        delta = r @ previous[1].T
        self.assertAlmostEqual(np.arccos((np.trace(delta)-1)/2), .005)

    def test_rotation_roundtrip_and_invalid_quaternion(self):
        for rpy in ([.2, .5, -.8], [0., np.pi/2, .3], [0., -np.pi/2, .3]):
            r = rpy_matrix(rpy)
            np.testing.assert_allclose(rpy_matrix(matrix_rpy(r)), r, atol=1e-12)
        with self.assertRaises(ValueError):
            quaternion_matrix([0., 0., 0., 0.])
        with self.assertRaises(ValueError):
            quaternion_matrix([float('nan'), 0., 0., 1.])


if __name__ == "__main__":
    unittest.main()

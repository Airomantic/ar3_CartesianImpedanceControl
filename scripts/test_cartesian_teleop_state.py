"""Offline service/clutch races using the production methods, without importing ROS."""

import ast
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from cartesian_teleop_math import limit_pose, map_pose, matrix_rpy, rpy_matrix


class CartesianTeleopStateTest(unittest.TestCase):
    def setUp(self):
        self.clock = SimpleNamespace(now=100.0)
        tree = ast.parse(Path(__file__).with_name("ar3_cartesian_impedance_teleop.py").read_text())
        production_class = next(node for node in tree.body if isinstance(node, ast.ClassDef))
        namespace = {
            "Node": object,
            "np": np,
            "time": SimpleNamespace(monotonic=lambda: self.clock.now),
            "SetBool": SimpleNamespace(Request=SimpleNamespace),
            "Trigger": SimpleNamespace(Request=SimpleNamespace),
            "Float64MultiArray": SimpleNamespace,
            "map_pose": map_pose,
            "limit_pose": limit_pose,
            "matrix_rpy": matrix_rpy,
        }
        exec(compile(ast.Module(body=[production_class], type_ignores=[]), str(__file__), "exec"), namespace)
        cls = namespace[production_class.name]
        self.node = node = cls.__new__(cls)
        self.parameters = {
            "dry_run": False, "rate_hz": 20.0, "service_timeout_s": 2.0,
            "target_timeout_s": 0.25, "feedback_timeout_s": 0.5, "clutch_timeout_s": 0.0,
            "max_linear_speed_m_s": 0.02, "max_angular_speed_rad_s": 0.15,
            "max_translation_lead_m": 0.01, "max_rotation_lead_rad": 0.05,
            "direction_diagnostics_hz": 0.0,
            "desired_wrench": [0.0] * 6,
        }
        node.p = self.parameters.__getitem__
        node.state = np.zeros(30)
        node.pose = (np.array([0.3, 0.1, 0.4]), np.eye(3))
        node.target = (np.array([9.0, 8.0, 7.0]), np.eye(3))
        node.state_at = node.pose_at = node.target_at = node.clutch_at = 100.0
        node.clutch = node.active = node.armed = False
        node.pending = None
        node.stop_waiting = False
        node.stop_failed_reason = ""
        node.stop_retry_at = 0.0
        node.must_release = True
        node.source_anchor = node.robot_anchor = node.last_command = None
        node.takeover_at = 0.0
        node.direction_report_at = 0.0
        node.last_tick = 100.0
        node.correction = np.eye(3)
        node.command_pub = MagicMock()
        node.preview_pub = MagicMock()
        node.get_logger = MagicMock(return_value=MagicMock())
        node.enable_client = MagicMock()
        node.stop_client = MagicMock()
        node.enable_client.service_is_ready.return_value = True
        node.stop_client.service_is_ready.return_value = True
        node.enable_client.call_async.return_value.done.return_value = False
        node.stop_client.call_async.return_value.done.return_value = False

    def takeover(self):
        node = self.node
        node.on_clutch(SimpleNamespace(data=False))
        node.on_clutch(SimpleNamespace(data=True))
        node.tick()
        self.assertTrue(node.active)
        self.clock.now += 0.05
        node.target_at = self.clock.now
        node.tick()
        self.assertEqual(node.pending[0], "enable")
        self.assertTrue(node.armed)  # An enable request can have taken effect before its reply.

    def complete(self, future, success=True, message="stopped; enable again to re-arm"):
        future.done.return_value = True
        future.result.return_value = SimpleNamespace(success=success, message=message)

    def advance_with_feedback(self, seconds=0.05):
        self.clock.now += seconds
        self.node.state_at = self.node.pose_at = self.node.target_at = self.clock.now

    def enable(self):
        self.takeover()
        self.complete(self.node.pending[1], message="armed; waiting for a NEW command")
        self.advance_with_feedback()
        self.node.tick()

    def test_initial_pressed_clutch_cannot_enable(self):
        self.node.on_clutch(SimpleNamespace(data=True))
        self.node.tick()
        self.node.enable_client.call_async.assert_not_called()
        self.node.command_pub.publish.assert_not_called()

    def test_takeover_needs_a_new_target_then_first_command_matches_measured_tcp(self):
        node = self.node
        node.on_clutch(SimpleNamespace(data=False))
        node.on_clutch(SimpleNamespace(data=True))
        node.tick()
        self.clock.now += 0.05
        node.tick()  # A pre-takeover source pose cannot arm or command the robot.
        node.enable_client.call_async.assert_not_called()
        node.target_at = self.clock.now
        node.tick()
        self.complete(node.pending[1])
        self.clock.now += 0.05
        node.tick()
        values = node.command_pub.publish.call_args.args[0].data
        np.testing.assert_allclose(values[:3], node.pose[0])

    def test_release_during_enable_is_followed_by_stop_without_a_command(self):
        self.takeover()
        node = self.node
        enable_future = node.pending[1]
        node.on_clutch(SimpleNamespace(data=False))
        node.stop_client.call_async.assert_not_called()
        self.complete(enable_future)
        node.tick()
        self.assertEqual(node.pending[0], "stop")
        node.command_pub.publish.assert_not_called()
        self.complete(node.pending[1])
        node.tick()
        self.assertFalse(node.armed)
        self.assertFalse(node.active)
        self.assertTrue(node.must_release)

    def test_lost_enable_reply_requests_stop(self):
        self.takeover()
        node = self.node
        future = node.pending[1]
        self.clock.now += 2.01
        node.tick()
        future.cancel.assert_called_once()
        self.assertEqual(node.pending[0], "stop")
        self.assertTrue(node.must_release)
        node.command_pub.publish.assert_not_called()

    def test_failed_enable_reply_requests_stop(self):
        self.takeover()
        node = self.node
        future = node.pending[1]
        future.done.return_value = True
        future.result.side_effect = RuntimeError("response lost after controller accepted request")
        node.tick()
        self.assertEqual(node.pending[0], "stop")
        node.command_pub.publish.assert_not_called()

    def test_stale_target_after_enable_stops_and_requires_release(self):
        self.takeover()
        node = self.node
        self.complete(node.pending[1])
        self.clock.now += 0.3
        node.tick()
        self.assertEqual(node.pending[0], "stop")
        self.assertTrue(node.must_release)
        node.command_pub.publish.assert_not_called()

    def test_failed_stop_is_retried_without_allowing_rearm(self):
        self.takeover()
        node = self.node
        self.complete(node.pending[1])
        node.on_clutch(SimpleNamespace(data=False))
        node.tick()
        self.complete(node.pending[1], success=False)
        node.tick()
        self.assertIsNone(node.pending)
        self.assertTrue(node.armed)
        self.assertFalse(node.active)
        self.advance_with_feedback(0.21)
        node.tick()
        self.assertEqual(node.pending[0], "stop")
        self.assertEqual(node.stop_client.call_async.call_count, 2)
        node.command_pub.publish.assert_not_called()

    def test_explicit_stop_failure_reply_blocks_takeover_without_retrying(self):
        self.enable()
        node = self.node
        command_count = node.command_pub.publish.call_count
        node.on_clutch(SimpleNamespace(data=False))
        self.complete(node.pending[1], success=False,
                      message="stop failed: idle confirmation timed out")
        node.tick()
        self.assertIn("idle confirmation timed out", node.stop_failed_reason)
        self.assertIn("idle confirmation timed out", node.get_logger().error.call_args.args[0])
        self.assertIsNone(node.pending)
        for _ in range(10):
            node.on_clutch(SimpleNamespace(data=False))
            node.on_clutch(SimpleNamespace(data=True))
            self.advance_with_feedback(1.0)
            node.tick()
        self.assertTrue(node.armed)
        self.assertFalse(node.active)
        self.assertEqual(node.enable_client.call_async.call_count, 1)
        self.assertEqual(node.stop_client.call_async.call_count, 1)
        self.assertEqual(node.command_pub.publish.call_count, command_count)
        node.get_logger().error.assert_called_once()

    def test_stop_failure_status_cancels_pending_enable_and_latches_once(self):
        self.takeover()
        node = self.node
        enable_future = node.pending[1]
        status = SimpleNamespace(data="stop failed: RT loop did not finish")
        node.on_status(status)
        self.complete(enable_future, message="armed; waiting for a NEW command")
        for _ in range(5):
            node.on_status(status)
            self.advance_with_feedback()
            node.tick()
        enable_future.cancel.assert_called_once()
        self.assertIsNone(node.pending)
        self.assertTrue(node.must_release)
        node.get_logger().error.assert_called_once()
        node.stop_client.call_async.assert_not_called()
        node.command_pub.publish.assert_not_called()

    def test_failed_stop_requires_stopped_status_then_new_release_press(self):
        self.enable()
        node = self.node
        node.on_status(SimpleNamespace(data="stop failed: idle confirmation timed out"))
        node.on_clutch(SimpleNamespace(data=False))
        node.on_clutch(SimpleNamespace(data=True))
        node.on_status(SimpleNamespace(data="stopping: waiting for idle"))
        self.advance_with_feedback()
        node.tick()
        self.assertTrue(node.stop_failed_reason)
        self.assertFalse(node.active)
        node.on_status(SimpleNamespace(data="stopped: late idle confirmation"))
        self.advance_with_feedback()
        node.tick()
        self.assertFalse(node.stop_failed_reason)
        self.assertFalse(node.active)
        self.assertFalse(node.armed)
        self.assertTrue(node.must_release)
        self.assertEqual(node.enable_client.call_async.call_count, 1)
        node.on_clutch(SimpleNamespace(data=False))
        node.on_clutch(SimpleNamespace(data=True))
        node.tick()
        self.assertTrue(node.active)
        self.advance_with_feedback()
        node.tick()
        self.assertEqual(node.enable_client.call_async.call_count, 2)

    def test_clutch_is_latched_without_a_heartbeat_while_fresh_targets_continue(self):
        self.enable()
        node = self.node
        clutch_received_at = node.clutch_at
        for _ in range(30):
            self.advance_with_feedback()
            node.tick()
        self.assertEqual(node.clutch_at, clutch_received_at)
        self.assertTrue(node.active)
        self.assertGreater(node.command_pub.publish.call_count, 20)
        node.stop_client.call_async.assert_not_called()
        node.on_clutch(SimpleNamespace(data=False))
        self.assertFalse(node.active)
        self.assertEqual(node.pending[0], "stop")

    def test_opt_in_clutch_heartbeat_timeout_still_stops(self):
        self.parameters["clutch_timeout_s"] = 0.5
        self.enable()
        self.advance_with_feedback(0.6)
        self.node.tick()
        self.assertFalse(self.node.active)
        self.assertIn("Daimon clutch heartbeat", self.node.get_logger().warning.call_args.args[0])

    def test_stale_feedback_reports_which_stream_stalled(self):
        for stream in ("state", "pose"):
            with self.subTest(stream=stream):
                self.setUp()
                self.enable()
                self.advance_with_feedback(0.6)
                setattr(self.node, stream + "_at", 100.0)
                self.node.tick()
                self.assertFalse(self.node.active)
                message = self.node.get_logger().warning.call_args.args[0]
                self.assertIn("SDK state" if stream == "state" else "SDK TCP pose", message)
                self.assertIn("age=", message)
                self.assertEqual(self.node.pending[0], "stop")

    def test_slow_or_initializing_robot_does_not_accumulate_far_ahead_targets(self):
        self.enable()
        node = self.node
        node.target = (node.target[0] + np.array([0.3, 0., 0.]), np.eye(3))
        for _ in range(80):
            self.advance_with_feedback()
            node.tick()
        self.assertTrue(node.active)
        values = node.command_pub.publish.call_args.args[0].data
        distance = np.linalg.norm(np.asarray(values[:3]) - node.pose[0])
        self.assertAlmostEqual(distance, self.parameters["max_translation_lead_m"])
        np.testing.assert_array_equal(values[6:12], np.zeros(6))
        node.stop_client.call_async.assert_not_called()

    def test_accepted_stop_waits_for_cleanup_before_rearming(self):
        self.enable()
        node = self.node
        node.on_clutch(SimpleNamespace(data=False))
        self.complete(node.pending[1], message="stopping: waiting for idle")
        node.tick()
        self.assertTrue(node.armed)
        self.assertTrue(node.stop_waiting)
        node.on_clutch(SimpleNamespace(data=True))
        self.advance_with_feedback(0.1)
        node.tick()
        self.assertEqual(node.enable_client.call_async.call_count, 1)
        self.assertEqual(node.stop_client.call_async.call_count, 1)
        self.advance_with_feedback(0.15)
        node.tick()
        self.assertEqual(node.pending[0], "stop")
        self.complete(node.pending[1], message="stopped; enable again to re-arm")
        node.tick()
        self.assertFalse(node.armed)
        self.assertFalse(node.active)
        self.assertTrue(node.must_release)
        self.assertEqual(node.enable_client.call_async.call_count, 1)

    def test_sdk_watchdog_halts_local_session_even_when_feedback_is_fresh(self):
        self.enable()
        node = self.node
        count = node.command_pub.publish.call_count
        node.on_status(SimpleNamespace(data="stopping: command watchdog expired"))
        self.advance_with_feedback()
        node.tick()
        self.assertFalse(node.active)
        self.assertTrue(node.must_release)
        self.assertEqual(node.pending[0], "stop")
        self.assertEqual(node.command_pub.publish.call_count, count)

    def test_immediate_stop_reply_cannot_rearm_a_press_received_during_cleanup(self):
        self.enable()
        node = self.node
        node.on_clutch(SimpleNamespace(data=False))
        node.on_clutch(SimpleNamespace(data=True))
        self.complete(node.pending[1])
        node.tick()
        self.advance_with_feedback()
        node.tick()
        self.assertFalse(node.armed)
        self.assertFalse(node.active)
        self.assertTrue(node.must_release)
        self.assertEqual(node.enable_client.call_async.call_count, 1)

    def test_external_tcp_displacement_stops_instead_of_breaking_command_slew_limit(self):
        for translated in (True, False):
            with self.subTest(translated=translated):
                self.setUp()
                self.enable()
                node = self.node
                count = node.command_pub.publish.call_count
                if translated:
                    node.pose = (node.pose[0] + np.array([0.03, 0., 0.]), node.pose[1])
                else:
                    node.pose = (node.pose[0], rpy_matrix([0.2, 0., 0.]))
                self.advance_with_feedback()
                node.tick()
                self.assertFalse(node.active)
                self.assertEqual(node.command_pub.publish.call_count, count)
                self.assertIn("tracking limits", node.get_logger().warning.call_args.args[0])

    def test_following_feedback_keeps_both_command_speed_and_lead_bounded(self):
        self.enable()
        node = self.node
        node.target = (node.target[0] + np.array([0.3, 0.1, 0.]), rpy_matrix([0., 0., 0.5]))
        previous = (node.pose[0].copy(), node.pose[1].copy())
        for _ in range(80):
            self.advance_with_feedback()
            node.tick()
            self.assertTrue(node.active)
            p, r = node.last_command
            self.assertLessEqual(np.linalg.norm(p-previous[0]), 0.02*0.05 + 1e-9)
            angle = np.arccos(np.clip((np.trace(r @ previous[1].T)-1.)/2., -1., 1.))
            self.assertLessEqual(angle, 0.15*0.05 + 1e-9)
            self.assertLessEqual(np.linalg.norm(p-node.pose[0]), 0.01 + 1e-9)
            previous = (p.copy(), r.copy())
            # A slower robot follows halfway toward each requested waypoint.
            node.pose = limit_pose(node.pose, previous, 0.0005, 0.003)


if __name__ == "__main__":
    unittest.main()

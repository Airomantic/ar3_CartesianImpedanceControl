"""Offline service/clutch races using the production methods, without importing ROS."""

import ast
import json
import unittest
import uuid
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from cartesian_teleop_math import limit_pose, map_pose, matrix_rpy, rpy_matrix
from cartesian_return_home import ReturnHomePath, load_home, pose_error


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
            "json": json,
            "uuid": uuid,
            "String": SimpleNamespace,
            "ReturnHomePath": ReturnHomePath,
            "load_home": load_home,
            "pose_error": pose_error,
        }
        exec(compile(ast.Module(body=[production_class], type_ignores=[]), str(__file__), "exec"), namespace)
        cls = namespace[production_class.name]
        self.node = node = cls.__new__(cls)
        self.parameters = {
            "dry_run": False, "rate_hz": 20.0, "service_timeout_s": 2.0,
            "target_timeout_s": 0.25, "feedback_timeout_s": 0.5, "clutch_timeout_s": 0.0,
            "max_linear_speed_m_s": 0.02, "max_angular_speed_rad_s": 0.15,
            "max_translation_lead_m": 0.01, "max_rotation_lead_rad": 0.05,
            "max_tracking_error_m": 0.03, "max_tracking_error_rad": 0.15,
            "direction_diagnostics_hz": 0.0,
            "desired_wrench": [0.0] * 6,
            "base_frame": "AR3-3_07R-W4C5C5_base",
            "return_home_file": "/no/saved/home.json",
            "return_home_linear_speed_m_s": 0.02,
            "return_home_angular_speed_rad_s": 0.1,
            "return_home_timeout_s": 60.,
            "return_home_workspace_radius_m": 0.25,
            "return_home_position_tolerance_m": 0.005,
            "return_home_rotation_tolerance_rad": 0.04,
        }
        node.p = self.parameters.__getitem__
        node.state = np.zeros(30)
        node.pose = (np.array([0.3, 0.1, 0.4]), np.eye(3))
        node.target = (np.array([9.0, 8.0, 7.0]), np.eye(3))
        node.state_at = node.pose_at = node.target_at = node.clutch_at = 100.0
        node.clutch = node.active = node.armed = False
        node.sdk_active = node.holding = node.resuming = False
        node.sdk_status = "monitoring: fresh RT state"
        node.last_values = node.session_parameters = None
        node.pending = None
        node.stop_waiting = False
        node.stop_failed_reason = ""
        node.stop_retry_at = 0.0
        node.must_release = True
        node.source_anchor = node.robot_anchor = node.last_command = None
        node.session_anchor = None
        node.home_values = node.home_pose = None
        node.home_error = ""
        node.home_path = None
        node.home_started_at = node.home_settled_at = None
        node.home_request_id = ""
        node.home_cancel_service = None
        node.home_cancel_service_name = ""
        node.home_status = "idle"
        node.home_phase = ""
        node.home_message = "No return requested"
        node.home_progress = None
        node.home_report_at = 0.0
        node.home_status_pub = MagicMock()
        node.create_service = MagicMock()
        node.destroy_service = MagicMock()
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
        self.node.on_status(SimpleNamespace(data="active: RT 1000Hz continuous TCP pose"))

    def use_deployed_translation_profile(self):
        # Historical tests retain the 10/30 mm fixture. New behavior must use
        # the actual constructor defaults, not a second copy of the settings.
        tree = ast.parse(Path(__file__).with_name("ar3_cartesian_impedance_teleop.py").read_text())
        defaults = next(statement.value for statement in ast.walk(tree)
                        if isinstance(statement, ast.Assign)
                        and any(isinstance(target, ast.Name) and target.id == "defaults"
                                for target in statement.targets))
        values = {ast.literal_eval(key): value for key, value in zip(defaults.keys, defaults.values)}
        for name in ("rate_hz", "target_direction_rpy_deg", "max_linear_speed_m_s",
                     "max_translation_lead_m", "max_tracking_error_m",
                     "return_home_position_tolerance_m"):
            self.parameters[name] = ast.literal_eval(values[name])
        self.node.correction = rpy_matrix(np.deg2rad(self.parameters["target_direction_rpy_deg"]))
        self.node.validate_tracking_limits()

    def test_initial_pressed_clutch_cannot_enable(self):
        self.node.on_clutch(SimpleNamespace(data=True))
        self.node.tick()
        self.node.enable_client.call_async.assert_not_called()
        self.node.command_pub.publish.assert_not_called()

    def test_command_lead_cannot_exceed_tracking_stop_limit(self):
        self.node.validate_tracking_limits()
        for lead, tracking in (("max_translation_lead_m", "max_tracking_error_m"),
                               ("max_rotation_lead_rad", "max_tracking_error_rad")):
            with self.subTest(lead=lead):
                original = self.parameters[lead]
                self.parameters[lead] = self.parameters[tracking] + 0.001
                with self.assertRaisesRegex(ValueError, lead):
                    self.node.validate_tracking_limits()
                self.parameters[lead] = original

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
        node.halt("Explicit stop")
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
        self.assertTrue(node.holding)
        self.assertIsNone(node.pending)
        node.stop_client.call_async.assert_not_called()

    def test_release_holds_the_complete_last_command_without_tracking_targets_or_drift(self):
        self.enable()
        node = self.node
        held = np.asarray(node.command_pub.publish.call_args.args[0].data).copy()
        node.on_clutch(SimpleNamespace(data=False))
        node.pose = (node.pose[0] + np.array([0.002, 0., 0.]), node.pose[1])
        node.target = (np.array([-99., 9., 123.]), rpy_matrix([0., 0., 1.]))
        node.state[12:] = 3.0  # Feedback changes must not replace session K/D/M.
        for _ in range(15):
            self.clock.now += 0.05
            node.state_at = node.pose_at = self.clock.now
            node.on_clutch(SimpleNamespace(data=False))
            node.tick()
            np.testing.assert_array_equal(node.command_pub.publish.call_args.args[0].data, held)
        self.assertTrue(node.holding)
        self.assertFalse(node.active)
        self.assertEqual(node.enable_client.call_async.call_count, 1)
        node.stop_client.call_async.assert_not_called()

    def test_first_hold_tick_beyond_soft_lead_keeps_fixed_goal(self):
        # Real failure: a feedback sample crossed the 10 mm shaping boundary
        # just after release and incorrectly caused a stop within one tick.
        for translated in (True, False):
            with self.subTest(translated=translated):
                self.setUp()
                self.enable()
                node = self.node
                held = np.asarray(node.command_pub.publish.call_args.args[0].data).copy()
                node.on_clutch(SimpleNamespace(data=False))
                if translated:
                    node.pose = (node.pose[0] + np.array([0.01001, 0., 0.]), node.pose[1])
                else:
                    node.pose = (node.pose[0], rpy_matrix([0.0501, 0., 0.]))
                for _ in range(5):
                    self.advance_with_feedback()
                    node.tick()
                    self.assertTrue(node.holding)
                    np.testing.assert_array_equal(node.command_pub.publish.call_args.args[0].data, held)
                node.stop_client.call_async.assert_not_called()

    def test_repeated_release_during_stop_does_not_repeat_warning(self):
        self.enable()
        node = self.node
        node.halt("Explicit stop")
        warnings = node.get_logger().warning.call_count
        for _ in range(10):
            node.on_clutch(SimpleNamespace(data=False))
        self.assertEqual(node.get_logger().warning.call_count, warnings)
        self.assertEqual(node.stop_client.call_async.call_count, 1)

    def test_release_before_sdk_active_status_stops_instead_of_starting_hold(self):
        self.takeover()
        node = self.node
        self.complete(node.pending[1], message="armed; waiting for a NEW command")
        self.advance_with_feedback()
        node.tick()
        self.assertIsNotNone(node.last_values)
        self.assertFalse(node.sdk_active)
        node.on_clutch(SimpleNamespace(data=False))
        self.assertFalse(node.holding)
        self.assertIsNone(node.last_values)
        self.assertEqual(node.pending[0], "stop")

    def test_release_with_nonzero_desired_wrench_stops(self):
        self.parameters["desired_wrench"][0] = 1.0
        self.enable()
        self.node.on_clutch(SimpleNamespace(data=False))
        self.assertFalse(self.node.holding)
        self.assertEqual(self.node.pending[0], "stop")

    def test_resume_reanchors_after_fresh_target_without_enabling_again(self):
        self.enable()
        node = self.node
        held = np.asarray(node.command_pub.publish.call_args.args[0].data).copy()
        node.on_clutch(SimpleNamespace(data=False))
        self.advance_with_feedback()
        node.tick()
        node.pose = (node.pose[0] + np.array([0.0005, 0., 0.]), node.pose[1])
        node.state[12:] = 9.0
        node.target = (node.target[0] + np.array([20., -30., 40.]), np.eye(3))
        node.on_clutch(SimpleNamespace(data=True))
        node.tick()
        self.assertTrue(node.resuming)
        np.testing.assert_array_equal(node.command_pub.publish.call_args.args[0].data, held)
        self.clock.now += 0.05
        node.state_at = node.pose_at = self.clock.now
        node.tick()  # Old target is ignored; held command keeps SDK watchdog alive.
        np.testing.assert_array_equal(node.command_pub.publish.call_args.args[0].data, held)
        self.advance_with_feedback()
        node.target_at = self.clock.now
        node.tick()
        self.assertTrue(node.active)
        self.assertFalse(node.holding)
        self.assertFalse(node.resuming)
        resumed = node.command_pub.publish.call_args.args[0].data
        np.testing.assert_allclose(resumed[:3], node.pose[0])
        np.testing.assert_array_equal(resumed[6:], held[6:])
        self.assertEqual(node.enable_client.call_async.call_count, 1)
        node.stop_client.call_async.assert_not_called()

    def test_resume_without_fresh_target_keeps_hold_briefly_then_stops(self):
        self.enable()
        node = self.node
        node.on_clutch(SimpleNamespace(data=False))
        node.on_clutch(SimpleNamespace(data=True))
        node.tick()
        count = node.command_pub.publish.call_count
        self.clock.now += 0.2
        node.state_at = node.pose_at = self.clock.now
        node.tick()
        self.assertEqual(node.command_pub.publish.call_count, count + 1)
        self.clock.now += 0.06
        node.state_at = node.pose_at = self.clock.now
        node.tick()
        self.assertEqual(node.command_pub.publish.call_count, count + 1)
        self.assertFalse(node.holding)
        self.assertFalse(node.resuming)
        self.assertIsNone(node.last_values)
        self.assertEqual(node.pending[0], "stop")

    def test_resume_after_hold_beyond_soft_lead_reanchors_without_jumping(self):
        self.enable()
        node = self.node
        held = np.asarray(node.command_pub.publish.call_args.args[0].data).copy()
        node.on_clutch(SimpleNamespace(data=False))
        node.pose = (node.pose[0] + np.array([0.01001, 0., 0.]), node.pose[1])
        self.advance_with_feedback()
        node.tick()
        node.on_clutch(SimpleNamespace(data=True))
        node.tick()
        self.assertTrue(node.resuming)
        np.testing.assert_array_equal(node.command_pub.publish.call_args.args[0].data, held)
        node.target = (node.target[0] + np.array([10., -20., 30.]), np.eye(3))
        self.advance_with_feedback()
        node.tick()
        resumed = np.asarray(node.command_pub.publish.call_args.args[0].data)
        step = np.linalg.norm(resumed[:3] - held[:3])
        self.assertGreater(step, 0.)
        self.assertLessEqual(step, 0.02*0.05 + 1e-9)
        np.testing.assert_array_equal(resumed[6:], held[6:])
        self.assertTrue(node.active)
        self.assertFalse(node.resuming)
        self.assertEqual(node.enable_client.call_async.call_count, 1)
        node.stop_client.call_async.assert_not_called()

    def test_stale_sdk_feedback_hard_stops_while_holding(self):
        for stream in ("state", "pose"):
            with self.subTest(stream=stream):
                self.setUp()
                self.enable()
                node = self.node
                node.on_clutch(SimpleNamespace(data=False))
                count = node.command_pub.publish.call_count
                self.advance_with_feedback(0.6)
                setattr(node, stream + "_at", 100.0)
                node.tick()
                self.assertFalse(node.holding)
                self.assertIsNone(node.last_values)
                self.assertEqual(node.command_pub.publish.call_count, count)
                self.assertEqual(node.pending[0], "stop")

    def test_controller_faults_hard_stop_and_clear_hold(self):
        for status in ("fault: SDK disconnected", "offline: no robot", "stopping: wrench limit",
                       "stopped: RT monitoring restoration failed", "stopped: monitoring unavailable",
                       "connecting: replaced node", "monitoring: replaced node"):
            with self.subTest(status=status):
                self.setUp()
                self.enable()
                node = self.node
                node.on_clutch(SimpleNamespace(data=False))
                node.on_status(SimpleNamespace(data=status))
                self.assertFalse(node.holding)
                self.assertFalse(node.sdk_active)
                self.assertIsNone(node.last_values)
                self.assertEqual(node.pending[0], "stop")

    def test_invalid_sdk_state_clears_hold(self):
        self.enable()
        self.node.on_clutch(SimpleNamespace(data=False))
        self.node.on_state(SimpleNamespace(data=[0.0] * 29))
        self.assertFalse(self.node.holding)
        self.assertIsNone(self.node.session_parameters)
        self.assertEqual(self.node.pending[0], "stop")

    def test_unused_target_is_ignored_while_released_but_invalid_resumed_target_stops(self):
        self.enable()
        node = self.node
        node.on_clutch(SimpleNamespace(data=False))
        node.decode_pose = MagicMock(side_effect=ValueError("invalid target orientation"))
        old_target_at = node.target_at
        node.on_target(SimpleNamespace())
        node.decode_pose.assert_not_called()
        self.assertEqual(node.target_at, old_target_at)
        self.assertTrue(node.holding)
        node.stop_client.call_async.assert_not_called()
        node.on_clutch(SimpleNamespace(data=True))
        node.on_target(SimpleNamespace())
        self.assertFalse(node.holding)
        self.assertIsNone(node.last_values)
        self.assertEqual(node.pending[0], "stop")

    def test_invalid_target_while_following_stops(self):
        self.enable()
        node = self.node
        node.decode_pose = MagicMock(side_effect=ValueError("invalid target orientation"))
        node.on_target(SimpleNamespace())
        self.assertFalse(node.active)
        self.assertIsNone(node.last_values)
        self.assertEqual(node.pending[0], "stop")

    def test_excessive_measured_displacement_while_holding_still_stops(self):
        for translated in (True, False):
            with self.subTest(translated=translated):
                self.setUp()
                self.enable()
                node = self.node
                node.on_clutch(SimpleNamespace(data=False))
                if translated:
                    node.pose = (node.pose[0] + np.array([0.0301, 0., 0.]), node.pose[1])
                else:
                    node.pose = (node.pose[0], rpy_matrix([0.1501, 0., 0.]))
                count = node.command_pub.publish.call_count
                self.advance_with_feedback()
                node.tick()
                self.assertFalse(node.holding)
                self.assertEqual(node.command_pub.publish.call_count, count)
                self.assertEqual(node.pending[0], "stop")
                message = node.get_logger().warning.call_args.args[0]
                self.assertIn("tracking limits (hold)", message)
                self.assertIn("translation=", message)
                self.assertIn("limit=30.000 mm", message)
                self.assertIn("rotation=", message)
                self.assertIn("limit=0.150000 rad", message)

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
        node.halt("Explicit stop")
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
        node.halt("Explicit stop")
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
                    # A 1 mm command slew could hide this 0.1 mm overshoot if
                    # tracking were checked only after computing the command.
                    node.pose = (node.pose[0] + np.array([0.0301, 0., 0.]), node.pose[1])
                else:
                    node.pose = (node.pose[0], rpy_matrix([0.1501, 0., 0.]))
                self.advance_with_feedback()
                node.tick()
                self.assertFalse(node.active)
                self.assertEqual(node.command_pub.publish.call_count, count)
                self.assertIn("tracking limits", node.get_logger().warning.call_args.args[0])

    def test_feedback_beyond_soft_lead_does_not_break_slew_continuity(self):
        for translated in (True, False):
            with self.subTest(translated=translated):
                self.setUp()
                self.enable()
                node = self.node
                previous = (node.last_command[0].copy(), node.last_command[1].copy())
                if translated:
                    node.pose = (node.pose[0] + np.array([0.015, 0., 0.]), node.pose[1])
                else:
                    node.pose = (node.pose[0], rpy_matrix([0.075, 0., 0.]))
                self.advance_with_feedback()
                node.tick()
                self.assertTrue(node.active)
                p, r = node.last_command
                self.assertLessEqual(np.linalg.norm(p-previous[0]), 0.02*0.05 + 1e-9)
                angle = np.arccos(np.clip((np.trace(r @ previous[1].T)-1.)/2., -1., 1.))
                self.assertLessEqual(angle, 0.15*0.05 + 1e-9)
                if translated:
                    self.assertGreater(np.linalg.norm(p-node.pose[0]), 0.01)
                else:
                    error = np.arccos(np.clip((np.trace(r @ node.pose[1].T)-1.)/2., -1., 1.))
                    self.assertGreater(error, 0.05)
                node.stop_client.call_async.assert_not_called()

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

    def test_deployed_profile_maps_all_six_source_directions_to_40mm_without_stop(self):
        # The right-arm mount maps source X/Y/Z to base X/-Z/Y respectively.
        expected_axes = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
        for axis in range(3):
            for sign in (-1., 1.):
                with self.subTest(axis=axis, sign=sign):
                    self.setUp()
                    self.use_deployed_translation_profile()
                    self.enable()
                    node = self.node
                    source_offset = np.zeros(3)
                    source_offset[axis] = sign * 0.2
                    node.target = (node.target[0] + source_offset, node.target[1])
                    previous = node.last_command[0].copy()
                    dt = 1. / self.parameters["rate_hz"]
                    for step in range(125):
                        self.advance_with_feedback(dt)
                        node.tick()
                        self.assertTrue(node.active)
                        p = node.last_command[0]
                        self.assertLessEqual(np.linalg.norm(p-previous), 0.04*dt + 1e-9)
                        if step == 0:
                            self.assertAlmostEqual(np.linalg.norm(p-previous)/dt, 0.04)
                        self.assertLessEqual(np.linalg.norm(p-node.pose[0]), 0.04 + 1e-9)
                        previous = p.copy()
                    np.testing.assert_allclose(node.last_command[0]-node.pose[0],
                                               sign * expected_axes[axis] * 0.04, atol=1e-9)
                    node.stop_client.call_async.assert_not_called()

    def test_deployed_profile_diagonal_lead_is_40mm_total_not_per_axis(self):
        self.use_deployed_translation_profile()
        self.enable()
        node = self.node
        source_offset = np.array([0.2, -0.2, 0.2])
        node.target = (node.target[0] + source_offset, node.target[1])
        for _ in range(125):
            self.advance_with_feedback(1. / self.parameters["rate_hz"])
            node.tick()
            self.assertTrue(node.active)
            self.assertLessEqual(np.linalg.norm(node.last_command[0]-node.pose[0]), 0.04 + 1e-9)
        expected_direction = np.array([1., 1., 1.]) / np.sqrt(3.)
        np.testing.assert_allclose(node.last_command[0]-node.pose[0], expected_direction*0.04, atol=1e-9)
        node.stop_client.call_async.assert_not_called()

    def test_deployed_profile_hold_and_resume_allow_45mm_error_without_target_jump(self):
        self.use_deployed_translation_profile()
        self.enable()
        node = self.node
        held = np.asarray(node.command_pub.publish.call_args.args[0].data).copy()
        node.on_clutch(SimpleNamespace(data=False))
        # External motion puts the measured TCP beyond the 40 mm shaping cap,
        # but within the 50 mm stop limit; a hold must remain fixed.
        node.pose = (node.pose[0] + np.array([0.045, 0., 0.]), node.pose[1])
        for _ in range(3):
            self.advance_with_feedback(0.01)
            node.tick()
            self.assertTrue(node.holding)
            np.testing.assert_array_equal(node.command_pub.publish.call_args.args[0].data, held)
        node.on_clutch(SimpleNamespace(data=True))
        node.tick()
        self.assertTrue(node.resuming)
        np.testing.assert_array_equal(node.robot_anchor[0], node.pose[0])
        np.testing.assert_array_equal(node.command_pub.publish.call_args.args[0].data, held)
        node.target = (node.target[0] + np.array([10., -20., 30.]), node.target[1])
        self.advance_with_feedback(0.01)
        node.tick()
        resumed = np.asarray(node.command_pub.publish.call_args.args[0].data)
        np.testing.assert_array_equal(node.source_anchor[0], node.target[0])
        np.testing.assert_allclose(resumed[:3]-held[:3], [0.0004, 0., 0.], atol=1e-9)
        np.testing.assert_array_equal(resumed[3:], held[3:])
        self.assertTrue(node.active)
        self.assertFalse(node.resuming)
        self.assertEqual(node.enable_client.call_async.call_count, 1)
        node.stop_client.call_async.assert_not_called()

    def test_deployed_profile_still_stops_above_50mm_in_all_directions(self):
        for holding in (False, True):
            for axis in range(3):
                for sign in (-1., 1.):
                    with self.subTest(holding=holding, axis=axis, sign=sign):
                        self.setUp()
                        self.use_deployed_translation_profile()
                        self.enable()
                        node = self.node
                        if holding:
                            node.on_clutch(SimpleNamespace(data=False))
                        displacement = np.zeros(3)
                        displacement[axis] = sign * 0.0501
                        node.pose = (node.pose[0] + displacement, node.pose[1])
                        count = node.command_pub.publish.call_count
                        self.advance_with_feedback(0.01)
                        node.tick()
                        self.assertFalse(node.active)
                        self.assertFalse(node.holding)
                        self.assertEqual(node.command_pub.publish.call_count, count)
                        self.assertEqual(node.pending[0], "stop")
                        self.assertIn("limit=50.000 mm", node.get_logger().warning.call_args.args[0])


if __name__ == "__main__":
    unittest.main()

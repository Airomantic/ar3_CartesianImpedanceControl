"""Offline return-to-origin motion, ownership and protection regressions."""

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import test_cartesian_teleop_state as teleop_tests
from cartesian_return_home import ReturnHomePath, load_home, pose_error
from cartesian_teleop_math import matrix_rpy, rpy_matrix


class ReturnHomeTest(unittest.TestCase):
    setUp = teleop_tests.CartesianTeleopStateTest.setUp
    takeover = teleop_tests.CartesianTeleopStateTest.takeover
    complete = teleop_tests.CartesianTeleopStateTest.complete
    advance_with_feedback = teleop_tests.CartesianTeleopStateTest.advance_with_feedback
    enable = teleop_tests.CartesianTeleopStateTest.enable
    use_deployed_translation_profile = teleop_tests.CartesianTeleopStateTest.use_deployed_translation_profile

    def prepare(self, offset=(0.03, -0.02, 0.01), rotation=(0.1, -0.1, 0.1)):
        self.enable()
        node = self.node
        node.on_clutch(SimpleNamespace(data=False))
        node.home_pose = (node.pose[0] + np.asarray(offset), rpy_matrix(rotation))
        node.home_values = np.concatenate((node.home_pose[0], matrix_rpy(node.home_pose[1])))

    def request(self):
        return self.node.on_return_home(SimpleNamespace(), SimpleNamespace())

    def prepare_idle(self):
        node = self.node
        node.on_clutch(SimpleNamespace(data=False))
        node.on_status(SimpleNamespace(data="monitoring: fresh RT state"))
        node.home_pose = (node.pose[0] + np.array([0.03, -0.02, 0.01]), rpy_matrix([0.1, -0.1, 0.1]))
        node.home_values = np.concatenate((node.home_pose[0], matrix_rpy(node.home_pose[1])))
        node.state[12:18] = [500., 500., 500., 50., 50., 50.]

    def test_idle_start_initializes_with_measured_hold_then_returns_and_resumes(self):
        self.prepare_idle()
        node = self.node
        measured = tuple(value.copy() for value in node.pose)
        self.assertTrue(self.request().success)
        self.assertEqual(node.pending[0], "home_enable")
        self.assertEqual(node.home_phase, "initializing")
        status = json.loads(node.home_status_pub.publish.call_args.args[0].data)
        self.assertTrue(status["startup_home_supported"])
        node.command_pub.publish.assert_not_called()
        self.complete(node.pending[1], message="armed: waiting for a NEW command")
        for _ in range(10):
            self.advance_with_feedback()
            node.tick()
            np.testing.assert_allclose(node.last_values[:3], measured[0])
            np.testing.assert_allclose(node.last_values[3:6], matrix_rpy(measured[1]))
            np.testing.assert_array_equal(node.last_values[6:12], np.zeros(6))
            self.assertEqual(node.home_path.elapsed, 0.)
        node.on_status(SimpleNamespace(data="active: RT 1000Hz continuous TCP pose"))
        self.run_following_to_completion()
        self.assertEqual(node.home_status, "succeeded")
        self.assertTrue(node.sdk_active)
        self.assertTrue(node.holding)
        node.on_clutch(SimpleNamespace(data=True))
        node.tick()
        self.assertTrue(node.resuming)
        np.testing.assert_allclose(node.robot_anchor[0], node.home_pose[0])
        node.enable_client.call_async.assert_called_once()
        node.stop_client.call_async.assert_not_called()

    def test_idle_start_rejections_never_enable(self):
        for case in ("unknown_clutch", "clutch_pressed", "foreign_active", "armed", "fault",
                     "stopping", "unknown_sdk", "stale_pose", "nonzero_wrench", "far_home", "dry_run"):
            with self.subTest(case=case):
                self.setUp()
                self.prepare_idle()
                node = self.node
                if case == "unknown_clutch":
                    node.clutch_at = 0.
                elif case == "clutch_pressed":
                    node.clutch = True
                elif case in ("foreign_active", "armed", "fault", "stopping", "unknown_sdk"):
                    node.sdk_status = {"foreign_active": "active: another controller", "armed": "armed: waiting",
                                       "fault": "fault: failed", "stopping": "stopping: waiting", "unknown_sdk": ""}[case]
                elif case == "stale_pose":
                    node.pose_at = 0.
                elif case == "nonzero_wrench":
                    self.parameters["desired_wrench"][0] = 1.
                elif case == "far_home":
                    node.home_pose = (node.pose[0] + np.array([0.251, 0., 0.]), node.pose[1])
                else:
                    self.parameters["dry_run"] = True
                self.assertFalse(self.request().success)
                node.enable_client.call_async.assert_not_called()
                node.command_pub.publish.assert_not_called()
                node.stop_client.call_async.assert_not_called()

    def test_idle_start_pending_cancel_and_clutch_press_serialize_stop_after_enable(self):
        for cancel in (True, False):
            with self.subTest(cancel=cancel):
                self.setUp()
                self.prepare_idle()
                node = self.node
                self.assertTrue(self.request().success)
                future = node.pending[1]
                if cancel:
                    node.on_cancel_return_home(SimpleNamespace(), SimpleNamespace())
                else:
                    node.on_clutch(SimpleNamespace(data=True))
                node.stop_client.call_async.assert_not_called()
                self.complete(future, message="armed: waiting for a NEW command")
                node.tick()
                self.assertEqual(node.pending[0], "stop")
                self.assertEqual(node.home_status, "failed")
                node.command_pub.publish.assert_not_called()

    def test_idle_start_foreign_session_race_never_commands_or_stops_other_owner(self):
        for cancelled in (False, True):
            with self.subTest(cancelled=cancelled):
                self.setUp()
                self.prepare_idle()
                node = self.node
                self.assertTrue(self.request().success)
                future = node.pending[1]
                node.on_status(SimpleNamespace(data="active: another controller"))
                if cancelled:
                    node.on_cancel_return_home(SimpleNamespace(), SimpleNamespace())
                self.complete(future, message="armed: existing session retained; waiting for commands")
                node.tick()
                self.assertFalse(node.armed)
                self.assertFalse(node.holding)
                self.assertEqual(node.home_status, "failed")
                node.command_pub.publish.assert_not_called()
                node.stop_client.call_async.assert_not_called()

    def test_idle_start_activation_timeout_and_feedback_loss_request_protective_stop(self):
        for stale in (False, True):
            with self.subTest(stale=stale):
                self.setUp()
                self.prepare_idle()
                node = self.node
                self.assertTrue(self.request().success)
                self.complete(node.pending[1], message="armed: waiting for a NEW command")
                node.tick()
                if stale:
                    self.clock.now += 0.6
                else:
                    self.advance_with_feedback(61.)
                node.tick()
                self.assertEqual(node.home_status, "failed")
                self.assertEqual(node.pending[0], "stop")

    def test_idle_start_enable_failure_timeout_and_force_fault_never_advance_path(self):
        for case in ("rpc_rejected", "rpc_lost", "force_fault"):
            with self.subTest(case=case):
                self.setUp()
                self.prepare_idle()
                node = self.node
                self.assertTrue(self.request().success)
                future = node.pending[1]
                if case == "rpc_lost":
                    self.advance_with_feedback(2.01)
                elif case == "rpc_rejected":
                    self.complete(future, success=False, message="enable rejected before motion: wrench limit")
                else:
                    node.on_status(SimpleNamespace(data="stopping: external wrench exceeded limit"))
                    self.complete(future, message="armed: waiting for a NEW command")
                node.tick()
                self.assertEqual(node.home_status, "failed")
                self.assertEqual(node.pending[0], "stop")
                node.command_pub.publish.assert_not_called()

    def run_following_to_completion(self):
        node = self.node
        for _ in range(1200):
            previous = tuple(value.copy() for value in node.last_command)
            self.advance_with_feedback()
            node.tick()
            step, angular_step = pose_error(previous, node.last_command)
            self.assertLessEqual(step, 0.02*0.05 + 1e-9)
            self.assertLessEqual(angular_step, 0.1*0.05 + 1e-9)
            lead, angular_lead = pose_error(node.last_command, node.pose)
            self.assertLessEqual(lead, self.parameters["max_translation_lead_m"] + 1e-9)
            self.assertLessEqual(angular_lead, self.parameters["max_rotation_lead_rad"] + 1e-9)
            node.pose = tuple(value.copy() for value in node.last_command)
            if node.home_status != "running":
                return
        self.fail("Return home never completed")

    def test_returns_smoothly_in_same_session_then_holds_and_clutch_reanchors(self):
        self.prepare()
        node = self.node
        held_parameters = node.last_values[6:].copy()
        response = self.request()
        self.assertTrue(response.success)
        request_id = json.loads(response.message)["request_id"]
        self.run_following_to_completion()
        self.assertEqual(node.home_status, "succeeded")
        np.testing.assert_allclose(node.last_command[0], node.home_pose[0])
        np.testing.assert_allclose(node.last_command[1], node.home_pose[1])
        np.testing.assert_array_equal(node.last_values[6:], held_parameters)
        self.assertTrue(node.holding)
        for _ in range(10):
            self.advance_with_feedback()
            node.tick()
        status = json.loads(node.home_status_pub.publish.call_args.args[0].data)
        self.assertEqual(status["request_id"], request_id)
        self.assertEqual(status["state"], "succeeded")
        node.on_clutch(SimpleNamespace(data=True))
        node.tick()
        self.assertTrue(node.resuming)
        np.testing.assert_allclose(node.robot_anchor[0], node.home_pose[0])
        self.assertEqual(node.enable_client.call_async.call_count, 1)
        node.stop_client.call_async.assert_not_called()

    def test_pause_path_for_obstructed_robot_then_timeout_protective_stop(self):
        self.prepare(offset=(0.15, 0., 0.), rotation=(0., 0., 0.))
        self.assertTrue(self.request().success)
        for _ in range(400):
            self.advance_with_feedback()
            self.node.tick()
        self.assertEqual(self.node.home_status, "running")
        self.assertLessEqual(pose_error(self.node.pose, self.node.last_command)[0], 0.01 + 1e-9)
        self.assertLess(self.node.home_path.elapsed, self.node.home_path.duration)
        self.advance_with_feedback(61.)
        self.node.tick()
        self.assertEqual(self.node.home_status, "failed")
        self.assertIn("timed out", self.node.home_message)
        self.assertEqual(self.node.pending[0], "stop")

    def test_timeout_retains_rotation_gate_and_pose_errors_before_stop(self):
        self.prepare(offset=(0.03, 0., 0.), rotation=(0.2, 0., 0.))
        node = self.node
        self.assertTrue(self.request().success)
        # Compliant orientation moves outside the soft lead cap, but inside
        # the independent hard tracking guard: XYZ path progress also pauses.
        node.pose = (node.pose[0], rpy_matrix([-0.06, 0., 0.]))
        self.advance_with_feedback()
        node.tick()
        self.assertEqual(node.home_path.elapsed, 0.)
        self.advance_with_feedback(61.)
        node.tick()
        self.assertEqual(node.home_status, "failed")
        self.assertIn("paused_rotation_lead", node.home_message)
        self.assertAlmostEqual(node.home_progress["tracking_mm_rad"][1], 0.06)
        self.assertAlmostEqual(node.home_progress["command_to_home_mm_rad"][0], 30.)
        final = dict(node.home_progress)
        node.pose = (node.pose[0] + 0.1, np.eye(3))
        node.publish_home_status()
        self.assertEqual(node.home_progress, final)
        self.assertEqual(node.pending[0], "stop")

    def test_static_impedance_offset_is_reported_without_false_completion(self):
        self.prepare(offset=(0.03, 0., 0.), rotation=(0., 0., 0.))
        node = self.node
        self.assertTrue(self.request().success)
        for _ in range(400):
            self.advance_with_feedback()
            node.pose = (node.last_command[0] - np.array([0.006, 0., 0.]), node.last_command[1].copy())
            node.tick()
        self.assertEqual(node.home_path.elapsed, node.home_path.duration)
        self.assertEqual(node.home_status, "running")
        node.publish_home_status()
        self.assertEqual(node.home_progress["gate"], "settling_at_home")
        self.assertEqual(node.home_progress["actual_to_home_mm_rad"], [6., 0.])
        self.assertEqual(node.home_progress["command_to_home_mm_rad"], [0., 0.])
        self.advance_with_feedback(61.)
        node.tick()
        self.assertEqual(node.home_status, "failed")
        self.assertIn("settling_at_home", node.home_message)
        self.assertEqual(node.pending[0], "stop")

    def test_deployed_home_window_accepts_reported_offset_but_rejects_larger_errors(self):
        for translation, rotation, accepted in ((0.012931, 0.03484, True),
                                                 (0.0201, 0.03484, False),
                                                 (0.012931, 0.0401, False)):
            with self.subTest(translation=translation, rotation=rotation):
                self.setUp()
                self.use_deployed_translation_profile()
                self.assertEqual(self.parameters["return_home_position_tolerance_m"], 0.02)
                self.prepare(offset=(0.03, 0., 0.), rotation=(0.1, 0., 0.))
                node = self.node
                self.assertTrue(self.request().success)
                for _ in range(1500):
                    self.advance_with_feedback(0.01)
                    node.pose = (node.last_command[0] - np.array([translation, 0., 0.]),
                                 rpy_matrix([-rotation, 0., 0.]) @ node.last_command[1])
                    node.tick()
                    if node.home_status != "running":
                        break
                np.testing.assert_allclose(node.last_command[0], node.home_pose[0])
                if accepted:
                    self.assertEqual(node.home_status, "succeeded")
                    self.assertTrue(node.holding)
                    self.assertTrue(node.sdk_active)
                    node.stop_client.call_async.assert_not_called()
                else:
                    self.assertEqual(node.home_status, "running")
                    self.assertEqual(node.home_path.elapsed, node.home_path.duration)
                    self.advance_with_feedback(61.)
                    node.tick()
                    self.assertEqual(node.home_status, "failed")
                    self.assertEqual(node.pending[0], "stop")

    def test_requires_released_clutch_and_existing_healthy_zero_wrench_hold(self):
        changes = (
            ("clutch", True), ("sdk_active", False), ("holding", False),
            ("active", True), ("must_release", True), ("stop_waiting", True),
            ("pending", ("enable", None, 100.)), ("stop_failed_reason", "failed"),
        )
        for key, value in changes:
            with self.subTest(key=key):
                self.setUp()
                self.prepare()
                setattr(self.node, key, value)
                count = self.node.command_pub.publish.call_count
                response = self.request()
                self.assertFalse(response.success)
                self.assertEqual(self.node.command_pub.publish.call_count, count)
                self.assertIsNone(self.node.home_path)
        for which in ("last_values", "session_parameters"):
            self.setUp()
            self.prepare()
            getattr(self.node, which)[6 if which == "last_values" else 0] = 1.
            self.assertFalse(self.request().success)

    def test_stale_or_missing_origin_rejects_without_motion_or_new_enable(self):
        for case in ("feedback", "origin", "dry_run"):
            with self.subTest(case=case):
                self.setUp()
                self.prepare()
                if case == "feedback":
                    self.node.pose_at = 0.
                elif case == "origin":
                    self.node.home_pose = self.node.home_values = None
                else:
                    self.parameters["dry_run"] = True
                self.assertFalse(self.request().success)
                self.assertIsNone(self.node.home_path)
                self.assertEqual(self.node.enable_client.call_async.call_count, 1)

    def test_workspace_uses_original_session_anchor_not_resumed_anchor(self):
        self.prepare(offset=(0.26, 0., 0.))
        # A clutch re-anchor near home does not move the SDK session workspace.
        self.node.robot_anchor = self.node.home_pose
        response = self.request()
        self.assertFalse(response.success)
        self.assertIn("original RT session anchor", response.message)
        self.assertIsNone(self.node.home_path)

    def test_duplicate_request_cannot_replace_running_request_or_goal(self):
        self.prepare()
        self.assertTrue(self.request().success)
        request_id, path = self.node.home_request_id, self.node.home_path
        self.assertFalse(self.request().success)
        self.assertEqual(self.node.home_request_id, request_id)
        self.assertIs(self.node.home_path, path)
        self.assertEqual(self.node.home_status, "running")

    def test_clutch_press_cancel_feedback_loss_and_fault_all_protective_stop(self):
        for case in ("clutch", "cancel", "feedback", "fault", "tracking"):
            with self.subTest(case=case):
                self.setUp()
                self.prepare()
                self.assertTrue(self.request().success)
                if case == "clutch":
                    self.node.on_clutch(SimpleNamespace(data=True))
                elif case == "cancel":
                    self.node.on_cancel_return_home(SimpleNamespace(), SimpleNamespace())
                elif case == "feedback":
                    self.clock.now += 0.6
                    self.node.tick()
                elif case == "fault":
                    self.node.on_status(SimpleNamespace(data="stopping: external wrench exceeded limit"))
                else:
                    self.node.pose = (self.node.pose[0] + np.array([0.031, 0., 0.]), self.node.pose[1])
                    self.advance_with_feedback()
                    self.node.tick()
                self.assertIsNone(self.node.home_path)
                self.assertEqual(self.node.home_status, "failed")
                self.assertEqual(self.node.pending[0], "stop")
                self.assertTrue(self.node.must_release)

    def test_normal_cancel_without_return_keeps_servo_hold(self):
        self.prepare()
        response = self.node.on_cancel_return_home(SimpleNamespace(), SimpleNamespace())
        self.assertTrue(response.success)
        self.assertTrue(self.node.holding)
        self.node.stop_client.call_async.assert_not_called()

    def test_late_scoped_cancel_cannot_stop_a_different_return_request(self):
        self.prepare()
        first_response = self.request()
        first_identity = json.loads(first_response.message)["request_id"]
        first_cancel = self.node.create_service.call_args.args[2]
        self.assertTrue(json.loads(first_response.message)["cancel_service"].endswith(first_identity))
        self.run_following_to_completion()
        second_response = self.request()
        second_identity = json.loads(second_response.message)["request_id"]
        self.assertNotEqual(first_identity, second_identity)
        self.node.destroy_service.assert_called_once()
        late_response = first_cancel(SimpleNamespace(), SimpleNamespace())
        self.assertFalse(late_response.success)
        self.assertEqual(self.node.home_status, "running")
        self.assertEqual(self.node.home_request_id, second_identity)
        self.node.stop_client.call_async.assert_not_called()
        current_cancel = self.node.create_service.call_args.args[2]
        current_response = current_cancel(SimpleNamespace(), SimpleNamespace())
        self.assertTrue(current_response.success)
        self.assertEqual(self.node.home_status, "failed")
        self.assertEqual(self.node.pending[0], "stop")

    def test_goal_requires_measured_arrival_not_only_final_command(self):
        self.prepare(offset=(0.008, 0., 0.), rotation=(0., 0., 0.))
        self.assertTrue(self.request().success)
        for _ in range(60):
            self.advance_with_feedback()
            self.node.tick()
        self.assertEqual(self.node.home_path.elapsed, self.node.home_path.duration)
        self.assertEqual(self.node.home_status, "running")
        self.node.pose = self.node.home_pose
        for _ in range(10):
            self.advance_with_feedback()
            self.node.tick()
        self.assertEqual(self.node.home_status, "succeeded")


class ReturnHomeFileTest(unittest.TestCase):
    def test_requires_measured_pose_source_valid_base_and_finite_coordinates(self):
        good = {"schema_version": 1, "frame_id": "base", "source": "measured_tcp_pose",
                "tcp_pose": [0.1, 0.2, 0.3, 0.4, -0.5, 0.6]}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "home.json"
            path.write_text(json.dumps(good))
            values, pose = load_home(path, "base")
            np.testing.assert_allclose(values, good["tcp_pose"])
            np.testing.assert_allclose(pose[0], values[:3])
            for key, value in (("source", "recorded_target"), ("frame_id", "tool"),
                               ("schema_version", 2), ("tcp_pose", [0.] * 5),
                               ("tcp_pose", [float("nan")] * 6)):
                path.write_text(json.dumps({**good, key: value}))
                with self.assertRaises(ValueError):
                    load_home(path, "base")

    def test_quintic_rotation_uses_shortest_path_across_euler_wrap(self):
        start = (np.zeros(3), rpy_matrix([0., 0., np.pi-0.05]))
        goal = (np.ones(3)*0.01, rpy_matrix([0., 0., -np.pi+0.05]))
        path = ReturnHomePath(start, goal, 0.02, 0.1)
        self.assertAlmostEqual(path.angle, 0.1)
        previous = start
        for _ in range(500):
            candidate, elapsed = path.candidate(0.01)
            step, angle = pose_error(previous, candidate)
            self.assertLessEqual(step, 0.0002 + 1e-9)
            self.assertLessEqual(angle, 0.001 + 1e-9)
            previous, path.elapsed = candidate, elapsed
        np.testing.assert_allclose(previous[1], goal[1])


if __name__ == "__main__":
    unittest.main()

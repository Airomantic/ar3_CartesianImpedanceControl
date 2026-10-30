"""Offline startup-status and QoS contract checks; no ROS graph or hardware."""

import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from ar3_wait_cartesian_state import CartesianStateWait, make_wait_node


class CartesianStateWaitTest(unittest.TestCase):
    def setUp(self):
        self.clock = SimpleNamespace(now=100.0)
        self.waiter = CartesianStateWait(180.0, clock=lambda: self.clock.now)

    def metadata(self):
        self.waiter.on_metadata(json.dumps({"motion_backend": "rt", "control_mode": "cartesianImpedance"}))

    def test_fault_arriving_before_any_state_fails_immediately_and_stays_failed(self):
        self.waiter.on_status("fault: RT takeover refused: operationState=rtControlling(4)")
        result = self.waiter.poll()
        self.assertEqual(result[0], 2)
        self.assertIn("operationState=rtControlling(4)", result[1])
        self.waiter.on_state([0.0] * 30)
        self.metadata()
        self.waiter.on_status("monitoring: fresh state")
        self.assertEqual(self.waiter.poll(), result)

    def test_stop_failed_and_offline_are_terminal(self):
        for status in ("stop failed: controller idle unconfirmed", "offline: no robot connection"):
            with self.subTest(status=status):
                self.setUp()
                self.waiter.on_status(status)
                self.assertEqual(self.waiter.poll()[0], 2)
                self.assertIn(status, self.waiter.poll()[1])

    def test_only_real_valid_state_with_rt_metadata_and_nonfault_status_succeeds(self):
        self.metadata()
        self.waiter.on_status("monitoring: RT 1ms state")
        self.assertIsNone(self.waiter.poll())
        self.waiter.on_state([0.0] * 30)
        self.assertEqual(self.waiter.poll()[0], 0)

    def test_invalid_state_dimensions_and_nonfinite_channels_fail(self):
        for values in ([0.] * 29, [0.] * 31, [float("nan")] + [0.] * 29,
                       [0.] * 29 + [float("inf")], [0.] * 7 + [-float("inf")] + [0.] * 22):
            with self.subTest(values=values):
                self.setUp()
                self.waiter.on_state(values)
                self.assertEqual(self.waiter.poll()[0], 2)
                self.assertFalse(self.waiter.state_received)

    def test_state_alone_and_stopping_status_cannot_report_ready(self):
        self.waiter.on_state([0.] * 30)
        self.assertIsNone(self.waiter.poll())
        self.metadata()
        self.assertIsNone(self.waiter.poll())
        self.waiter.on_status("stopping: waiting for idle")
        self.assertIsNone(self.waiter.poll())
        self.waiter.on_status("stopped: idle confirmed")
        self.assertIsNone(self.waiter.poll())
        self.waiter.on_state([0.] * 30)
        self.assertEqual(self.waiter.poll()[0], 0)

    def test_old_state_does_not_hide_monitoring_restoration_failure(self):
        for status in (
                "stopped: controller idle confirmed; RT monitoring restoration failed: timeout",
                "stopped: controller idle confirmed; RT monitoring unavailable, restart node"):
            with self.subTest(status=status):
                self.setUp()
                self.metadata()
                self.waiter.on_state([0.] * 30)
                self.waiter.on_status(status)
                self.assertEqual(self.waiter.poll()[0], 2)
                self.assertIn(status, self.waiter.poll()[1])

    def test_state_expires_even_with_healthy_persisted_status_and_metadata(self):
        self.metadata()
        self.waiter.on_state([0.] * 30)
        self.clock.now += 0.501
        self.waiter.on_status("monitoring: RT 1ms state")
        self.assertIsNone(self.waiter.poll())
        self.waiter.on_state([0.] * 30)
        self.assertEqual(self.waiter.poll()[0], 0)

    def test_connecting_or_stopping_requires_a_sample_after_recovery(self):
        for status in ("connecting: recreating RT monitor", "stopping: waiting for idle"):
            with self.subTest(status=status):
                self.setUp()
                self.metadata()
                self.waiter.on_state([0.] * 30)
                self.waiter.on_status(status)
                self.assertFalse(self.waiter.state_received)
                self.waiter.on_state([0.] * 30)  # In-progress cleanup is not recovery.
                self.waiter.on_status("stopped: RT monitoring restored")
                self.assertIsNone(self.waiter.poll())
                self.waiter.on_state([0.] * 30)
                self.assertEqual(self.waiter.poll()[0], 0)

    def test_wrong_or_malformed_metadata_fails(self):
        for text in ("invalid JSON", "[]", '{"motion_backend":"nrt"}',
                     '{"motion_backend":"rt","control_mode":"jointPosition"}'):
            with self.subTest(text=text):
                self.setUp()
                self.waiter.on_metadata(text)
                self.assertEqual(self.waiter.poll()[0], 2)

    def test_timeout_preserves_last_controller_status(self):
        self.waiter.on_status("connecting: RT robot/state initialization")
        self.clock.now = 279.999
        self.assertIsNone(self.waiter.poll())
        self.clock.now = 280.0
        code, message = self.waiter.poll()
        self.assertEqual(code, 124)
        self.assertIn("connecting:", message)
        self.assertIn("实际 state", message)

    def test_dependency_exit_is_checked_while_waiting_and_before_success(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "impedance.exit"
            self.waiter.dependency_exit_paths = (path,)
            self.assertIsNone(self.waiter.poll())
            path.write_text("1\n")
            self.waiter.on_state([0.] * 30)
            self.metadata()
            self.waiter.on_status("monitoring: ready")
            code, message = self.waiter.poll()
            self.assertEqual(code, 2)
            self.assertIn("impedance", message)

    def test_subscription_qos_receives_persisted_fault_and_no_fake_ready_publishers(self):
        node = MagicMock()
        qos_sensor = object()
        modules = {
            "rclpy.node": SimpleNamespace(Node=MagicMock(return_value=node)),
            "rclpy.qos": SimpleNamespace(
                QoSProfile=SimpleNamespace, qos_profile_sensor_data=qos_sensor,
                DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL="transient_local"),
                ReliabilityPolicy=SimpleNamespace(RELIABLE="reliable")),
            "std_msgs.msg": SimpleNamespace(Float64MultiArray=object, String=str),
        }
        with patch.dict(sys.modules, modules):
            self.assertIs(make_wait_node(self.waiter), node)
        modules["rclpy.node"].Node.assert_called_once_with(
            "ar3_wait_cartesian_state", enable_rosout=False, start_parameter_services=False)
        subscriptions = {call.args[1].rsplit("/", 1)[1]: call.args for call in node.create_subscription.call_args_list}
        self.assertEqual(set(subscriptions), {"state", "status", "metadata"})
        self.assertIs(subscriptions["state"][3], qos_sensor)
        for topic in ("status", "metadata"):
            qos = subscriptions[topic][3]
            self.assertEqual(qos.depth, 1)
            self.assertEqual(qos.reliability, "reliable")
            self.assertEqual(qos.durability, "transient_local")
        subscriptions["status"][2](SimpleNamespace(data="fault: already failed before waiter joined"))
        self.assertEqual(self.waiter.poll()[0], 2)
        node.create_publisher.assert_not_called()
        node.create_client.assert_not_called()


if __name__ == "__main__":
    unittest.main()

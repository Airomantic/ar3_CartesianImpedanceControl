"""Offline checks for measured home capture; no ROS or robot connection."""

import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from ar3_capture_home import BASE_FRAME, StationaryHomeCapture, check_publishers, write_home


class CaptureHomeTest(unittest.TestCase):
    def feed(self, capture, step, x=0.2):
        received = 100.0 + step * 0.02
        source = 1000.0 + step * 0.02
        capture.on_state([0.0] * 30, received)
        capture.on_pose([x, -0.3, 0.1], [0.0, 0.0, 0.0, 1.0], BASE_FRAME,
                        source, source + 0.01, received)
        return capture.result(received, source + 0.01)

    def test_stationary_capture_records_last_measured_pose(self):
        capture = StationaryHomeCapture()
        self.assertIsNone(self.feed(capture, 0))
        result = None
        for step in range(1, 53):
            result = self.feed(capture, step, x=0.2001)
        self.assertIsNotNone(result)
        self.assertEqual(result["source"], "measured_tcp_pose")
        self.assertEqual(result["frame_id"], BASE_FRAME)
        self.assertEqual(result["tcp_pose"], [0.2001, -0.3, 0.1, 0.0, -0.0, 0.0])

    def test_continuous_slow_motion_cannot_count_as_stationary(self):
        capture = StationaryHomeCapture()
        for step in range(150):
            self.assertIsNone(self.feed(capture, step, x=0.2 + step * 0.0001))

    def test_old_or_repeated_pose_and_missing_state_cannot_capture(self):
        capture = StationaryHomeCapture()
        with self.assertRaisesRegex(ValueError, "not live"):
            capture.on_pose([0.2, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], BASE_FRAME,
                            1000.0, 1001.0, 100.0)
        for step in range(60):
            capture.on_state([0.0] * 30, 100.0 + step * 0.02)
            capture.on_pose([0.2, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], BASE_FRAME,
                            1000.0, 1000.01, 100.0 + step * 0.02)
        self.assertIsNone(capture.result(101.18, 1000.01))
        stable = StationaryHomeCapture()
        for step in range(60):
            self.feed(stable, step)
        stable.state_at = 90.0
        self.assertIsNone(stable.result(101.18, 1001.19))

    def test_bad_state_frame_and_quaternion_rejected(self):
        capture = StationaryHomeCapture()
        with self.assertRaises(ValueError):
            capture.on_state([0.0] * 29, 100.0)
        with self.assertRaises(ValueError):
            capture.on_pose([0.2, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], "world", 1000, 1000, 100)
        with self.assertRaises(ValueError):
            capture.on_pose([0.2, 0.0, 0.0], [0.0] * 4, BASE_FRAME, 1000, 1000, 100)

    def test_home_creation_does_not_replace_existing_origin(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "home.json"
            write_home(path, {"tcp_pose": [1, 2, 3]})
            with self.assertRaises(FileExistsError):
                write_home(path, {"tcp_pose": [4, 5, 6]})
            self.assertEqual(json.loads(path.read_text())["tcp_pose"], [1, 2, 3])
            write_home(path, {"tcp_pose": [4, 5, 6]}, overwrite=True)
            self.assertEqual(json.loads(path.read_text())["tcp_pose"], [4, 5, 6])
            self.assertEqual(list(Path(directory).iterdir()), [path])

    def test_discovery_unknown_and_ambiguous_publishers_wait_until_correct(self):
        node = Mock()
        known = SimpleNamespace(node_name="cartesian_impedance_node", node_namespace="/")
        unknown = SimpleNamespace(node_name="_NODE_NAME_UNKNOWN_", node_namespace="_NODE_NAMESPACE_UNKNOWN_")
        for publishers in ([], [unknown], [known, known]):
            node.get_publishers_info_by_topic.return_value = publishers
            ready, diagnostic = check_publishers(node)
            self.assertFalse(ready)
            self.assertTrue(diagnostic)
        node.get_publishers_info_by_topic.return_value = [known]
        self.assertTrue(check_publishers(node)[0])
        node.get_publishers_info_by_topic.side_effect = [
            [known], [SimpleNamespace(node_name="cartesian_impedance_node", node_namespace="/other")],
        ]
        self.assertFalse(check_publishers(node)[0])


if __name__ == "__main__":
    unittest.main()

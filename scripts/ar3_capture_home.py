#!/usr/bin/env python3
"""Capture a stationary measured TCP as persistent home, using ROS subscriptions only."""

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile
import time

import numpy as np

from cartesian_teleop_math import matrix_rpy, quaternion_matrix


BASE_FRAME = "AR3-3_07R-W4C5C5_base"
PREFIX = "/ar3/cartesian_impedance"
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "config" / "cartesian_home.json"


class StationaryHomeCapture:
    """Require one second of fresh, stationary pose samples and fresh 30-D state."""

    def __init__(self, frame=BASE_FRAME, duration_s=1.0):
        self.frame = frame
        self.duration_s = duration_s
        self.samples = deque()
        self.state_at = None
        self.detail = "awaiting measured TCP and state"

    def on_state(self, values, received_at):
        if len(values) != 30 or not all(math.isfinite(value) for value in values):
            raise ValueError("SDK state must contain exactly 30 finite values")
        self.state_at = received_at

    def on_pose(self, position, quaternion, frame, source_at, ros_now, received_at):
        if frame != self.frame:
            raise ValueError(f"TCP frame {frame!r} differs from expected {self.frame!r}")
        position, quaternion = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
        if (position.shape != (3,) or quaternion.shape != (4,)
                or not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion))
                or abs(np.linalg.norm(quaternion) - 1.0) > 1e-3):
            raise ValueError("TCP position/quaternion is invalid")
        if not all(math.isfinite(value) for value in (source_at, ros_now, received_at)):
            raise ValueError("TCP timestamps are invalid")
        age = ros_now - source_at
        if source_at <= 0 or not -0.1 <= age <= 0.5:
            raise ValueError(f"TCP source timestamp is not live: age={age:.3f}s")
        rotation = quaternion_matrix(quaternion)
        if self.samples:
            last = self.samples[-1]
            if source_at <= last[1]:
                # Repeated/restamped cached data cannot satisfy the stationary window.
                self.detail = "awaiting increasing TCP source timestamps"
                self.samples.clear()
                return
            if received_at - last[0] > 0.25 or source_at - last[1] > 0.25:
                self.samples.clear()
        while len(self.samples) > 1 and received_at - self.samples[1][0] >= self.duration_s:
            self.samples.popleft()
        # Compare with every sample, bounding the full spread rather than only
        # adjacent steps (which could conceal continuous slow movement).
        for sample in self.samples:
            translation = np.linalg.norm(position - sample[2])
            angle = np.arccos(np.clip((np.trace(rotation @ sample[4].T) - 1.0) / 2.0, -1.0, 1.0))
            if translation > 0.001 or angle > 0.01:
                self.samples.clear()
                self.detail = "TCP is moving; waiting for 1 s within 1 mm / 0.01 rad"
                break
        self.samples.append((received_at, source_at, position.copy(), quaternion.copy(), rotation))

    def result(self, now, ros_now):
        if self.state_at is None or not 0 <= now - self.state_at <= 0.5:
            self.detail = "awaiting fresh SDK state"
            return None
        if len(self.samples) < 2:
            return None
        first, last = self.samples[0], self.samples[-1]
        if not 0 <= now - last[0] <= 0.5 or not -0.1 <= ros_now - last[1] <= 0.5:
            self.detail = "awaiting fresh measured TCP"
            return None
        if last[0] - first[0] < self.duration_s or last[1] - first[1] < self.duration_s:
            self.detail = "collecting 1 s of stationary measured TCP"
            return None
        return {
            "schema_version": 1,
            "frame_id": self.frame,
            "tcp_pose": np.concatenate((last[2], matrix_rpy(last[4]))).tolist(),
            "tcp_quaternion_xyzw": last[3].tolist(),
            "source": "measured_tcp_pose",
            "captured_at": datetime.now(timezone.utc).isoformat(),
            "source_stamp_s": last[1],
        }


def write_home(path, document, overwrite=False):
    """Publish a complete JSON file atomically; refuse replacement by default."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        if overwrite:
            os.replace(temporary, path)
        else:
            # link is atomic and fails if another process created path meanwhile.
            os.link(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def check_publishers(node):
    """ROS discovery is asynchronous: unknown/ambiguous identity means wait."""
    identities = []
    for suffix in ("tcp_pose", "state"):
        publishers = node.get_publishers_info_by_topic(PREFIX + "/" + suffix)
        if not publishers:
            return False, f"awaiting publisher discovery on {suffix}"
        if len(publishers) != 1 or publishers[0].node_name != "cartesian_impedance_node":
            names = [f"{entry.node_namespace}/{entry.node_name}" for entry in publishers]
            return False, f"awaiting one cartesian_impedance_node publisher on {suffix}; observed {names}"
        identities.append((publishers[0].node_name, publishers[0].node_namespace))
    if identities[0] != identities[1]:
        return False, "TCP and state publishers do not share the same node identity"
    return True, "one matching cartesian_impedance_node discovered on TCP and state"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--overwrite", action="store_true", help="explicitly replace an existing home file")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 1.0:
        parser.error("--timeout must be finite and greater than 1 second")
    if args.output.exists() and not args.overwrite:
        parser.error(f"home already exists: {args.output}; use --overwrite only to redefine it")

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float64MultiArray

    rclpy.init(args=[])
    node = Node("ar3_capture_home", enable_rosout=False, start_parameter_services=False)
    capture = StationaryHomeCapture()

    def receive_pose(message):
        p, q = message.pose.position, message.pose.orientation
        capture.on_pose(
            [p.x, p.y, p.z], [q.x, q.y, q.z, q.w], message.header.frame_id,
            message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            node.get_clock().now().nanoseconds * 1e-9, time.monotonic(),
        )

    node.create_subscription(PoseStamped, PREFIX + "/tcp_pose", receive_pose, qos_profile_sensor_data)
    node.create_subscription(
        Float64MultiArray, PREFIX + "/state",
        lambda message: capture.on_state(message.data, time.monotonic()), qos_profile_sensor_data,
    )
    try:
        deadline = time.monotonic() + args.timeout
        identity_ready_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            ready, identity_detail = check_publishers(node)
            now = time.monotonic()
            if not ready:
                identity_ready_since = None
                capture.samples.clear()
                capture.state_at = None
                capture.detail = identity_detail
                continue
            if identity_ready_since is None:
                identity_ready_since = now
            if now - identity_ready_since < 0.3:
                # Do not accept samples collected before source discovery has
                # stabilized; an unknown endpoint can appear briefly at startup.
                capture.samples.clear()
                capture.state_at = None
                capture.detail = "waiting for stable TCP/state publisher identity"
                continue
            document = capture.result(now, node.get_clock().now().nanoseconds * 1e-9)
            if document is not None:
                write_home(args.output, document, args.overwrite)
                print(f"Saved measured home: {args.output.resolve()}", flush=True)
                print(json.dumps(document, ensure_ascii=False), flush=True)
                return 0
        raise RuntimeError(f"No stationary live TCP within {args.timeout:g}s: {capture.detail}")
    except (ValueError, RuntimeError, OSError) as error:
        print(f"Home capture failed: {error}", flush=True)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

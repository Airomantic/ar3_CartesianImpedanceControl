#!/usr/bin/env python3
"""Read-only startup wait: real RT state, controller status and RT metadata."""

import argparse
import json
import math
from pathlib import Path
import re
import sys
import time


class CartesianStateWait:
    def __init__(self, timeout_s, dependency_exit_paths=(), clock=time.monotonic):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout must be finite and positive")
        self.clock = clock
        self.deadline = clock() + timeout_s
        self.dependency_exit_paths = tuple(Path(path) for path in dependency_exit_paths)
        self.state_received = False
        self.state_at = None
        self.metadata_received = False
        self.status = ""
        self.failure = ""

    def fail(self, message):
        if not self.failure:
            self.failure = message

    def on_state(self, values):
        if len(values) != 30 or not all(math.isfinite(value) for value in values):
            self.fail("SDK state 无效：需要 30 个有限数值（pose6,wrench6,K6,D6,M6）")
            return
        if self.status.startswith(("connecting:", "stopping:")):
            return
        self.state_received = True
        self.state_at = self.clock()

    def on_metadata(self, text):
        try:
            metadata = json.loads(text)
            if (not isinstance(metadata, dict) or metadata.get("motion_backend") != "rt"
                    or metadata.get("control_mode") != "cartesianImpedance"):
                raise ValueError("需要 rt / cartesianImpedance")
        except (ValueError, TypeError) as error:
            self.fail(f"SDK metadata 无效：{error}")
            return
        self.metadata_received = True

    def on_status(self, text):
        self.status = text.strip()
        if (self.status.startswith(("fault:", "stop failed:", "offline:"))
                or "RT monitoring restoration failed" in self.status
                or "monitoring unavailable" in self.status):
            self.fail(f"机械臂节点未就绪：{self.status}；请查看任务 impedance 标签页")
        if self.status.startswith(("connecting:", "stopping:")):
            # A prior sample cannot prove that monitoring recovered after this
            # transition. Wait for a new sample after a usable SDK status.
            self.state_received = False
            self.state_at = None

    def poll(self):
        if self.failure:
            return 2, self.failure
        for path in self.dependency_exit_paths:
            if path.exists():
                return 2, f"依赖任务 {path.stem} 已退出，请查看对应标签页"
        current_time = self.clock()
        fresh_state = self.state_at is not None and 0 <= current_time - self.state_at <= 0.5
        if current_time >= self.deadline:
            missing = []
            if not fresh_state:
                missing.append("最近 0.5 秒内的实际 state 数据")
            if not self.metadata_received:
                missing.append("RT metadata")
            missing.append(f"当前 status={self.status or '尚未收到'}")
            return 124, "等待机械臂状态超时：" + "；".join(missing)
        # A status arriving on its own cannot stand in for real state samples.
        # Likewise a stale state publisher must not hide a terminal SDK status.
        if (fresh_state and self.metadata_received
                and self.status.startswith(("monitoring:", "armed:", "initializing:", "active:", "stopped:"))):
            return 0, "已收到有效 RT state、metadata 和非故障 SDK status"
        return None


def make_wait_node(waiter):
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import Float64MultiArray, String

    node = Node("ar3_wait_cartesian_state", enable_rosout=False, start_parameter_services=False)
    prefix = "/ar3/cartesian_impedance"
    durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Float64MultiArray, prefix + "/state",
                             lambda message: waiter.on_state(message.data), qos_profile_sensor_data)
    node.create_subscription(String, prefix + "/status",
                             lambda message: waiter.on_status(message.data), durable)
    node.create_subscription(String, prefix + "/metadata",
                             lambda message: waiter.on_metadata(message.data), durable)
    return node


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--dependency", action="append", default=[])
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    if args.dependency and (args.run_dir is None or any(
            re.fullmatch(r"[a-z][a-z0-9_]*", name) is None for name in args.dependency)):
        parser.error("--dependency requires --run-dir and a task name")
    exits = [args.run_dir / f"{name}.exit" for name in args.dependency]
    waiter = CartesianStateWait(args.timeout, exits)
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=[])
    node = None
    try:
        node = make_wait_node(waiter)
        while rclpy.ok():
            result = waiter.poll()
            if result is not None:
                code, message = result
                print(f"[oneClick] {message}", file=sys.stdout if code == 0 else sys.stderr, flush=True)
                return code
            rclpy.spin_once(node, timeout_sec=0.1)
        return 130
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

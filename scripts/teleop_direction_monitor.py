#!/usr/bin/env python3
"""Read-only comparison of Cartesian targets, IK previews and real feedback."""

import math
import time

import numpy as np
import pinocchio as pin
import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


def delta_text(anchor, pose):
    delta_mm = (pose.translation - anchor.translation) * 1000.0
    # Spatial rotation vector: signs refer to the same base axes as dXYZ.
    rotation_degrees = np.degrees(pin.log3(pose.rotation @ anchor.rotation.T))
    xyz = ', '.join(f'{value:+.1f}' for value in delta_mm)
    rotation = ', '.join(f'{value:+.1f}' for value in rotation_degrees)
    return f'dXYZ=({xyz})mm dRotXYZ=({rotation})deg'


def ar3r_wrist_bounds(q6_deg, q7_deg):
    """Right-arm polygon, AR3 hardware manual 2026-06-08, section 4.1.4.

    J6/J7 vertices (degrees): (-60,+/-46), (-19,+/-90),
    (+10,+/-90), (+60,+/-46). This is diagnostic, not a command limiter.
    """
    q7_bound = min(90.0, 90.0 + 44.0 / 41.0 * (q6_deg + 19.0),
                   90.0 - 44.0 / 50.0 * (q6_deg - 10.0))
    margin = min(60.0 - abs(q6_deg), q7_bound - abs(q7_deg))
    return q7_bound, margin


class DirectionMonitor(Node):
    def __init__(self):
        super().__init__('ar3r_teleop_direction_monitor')
        self.base = 'AR3-3_07R-W4C5C5_base'
        self.ee = 'AR3-3_07R-W4C5C5_tcp'
        self.names = [f'joint{i}' for i in range(1, 8)]
        self.model = None
        self.clutch = False
        self.anchors = {}
        self.latest = {}
        self.joint_samples = {}
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        description_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.description, description_qos)
        self.create_subscription(Bool, '/clutch/right', self.clutch_changed, 10)
        self.create_subscription(PoseStamped, '/target_robot/right_ee/target', self.target, state_qos)
        self.create_subscription(
            PoseStamped, '/ar3r_retargeting/mapped_target',
            lambda msg: self.target(msg, 'MAPPED'), state_qos,
        )
        self.create_subscription(JointState, '/ar3r_retargeting/preview_joint_states', self.preview, state_qos)
        self.create_subscription(
            JointTrajectoryControllerState,
            '/position_joint_trajectory_controller/controller_state',
            self.feedback, state_qos,
        )
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            '只读方向诊断；不发布运动指令。请先确认遥操 dry_run=true。'
            '每次按下离合后，以各路第一帧为零点；TARGET 保留输入分量，'
            'MAPPED/IK/ACTUAL 按机器人基坐标解释。'
            '本机右臂侧装：基座 +X 朝前、+Y 朝上、+Z 朝右。'
        )

    def description(self, msg):
        if self.model is not None:
            return
        try:
            model = pin.buildModelFromXML(msg.data)
            if model.nq != 7 or any(not model.existJointName(n) for n in self.names):
                raise ValueError('需要包含 joint1..joint7 的固定基座七轴模型')
            if not model.existFrame(self.base) or not model.existFrame(self.ee):
                raise ValueError('模型缺少 AR3R base/tcp')
            self.indices = [model.joints[model.getJointId(n)].idx_q for n in self.names]
            self.data = model.createData()
            self.base_id, self.ee_id = model.getFrameId(self.base), model.getFrameId(self.ee)
            self.model = model
            self.get_logger().info(f'已加载模型；末端为 {self.ee}')
        except Exception as exc:
            self.get_logger().error(f'模型加载失败：{exc}')

    def clutch_changed(self, msg):
        if msg.data and not self.clutch:
            self.anchors.clear()
            self.latest.clear()
            self.joint_samples.clear()
            self.get_logger().info('新一轮：保持手柄朝向，先沿一个方向平移，再松开离合。')
        self.clutch = msg.data

    def remember(self, key, pose):
        if not self.clutch:
            return
        self.latest[key] = (pose.copy(), time.monotonic())
        self.anchors.setdefault(key, pose.copy())

    def target(self, msg, key='TARGET'):
        if msg.header.frame_id.lstrip('/') != self.base:
            return
        p, o = msg.pose.position, msg.pose.orientation
        xyz = np.array([p.x, p.y, p.z])
        quat = np.array([o.w, o.x, o.y, o.z])
        norm = np.linalg.norm(quat)
        if not np.isfinite(xyz).all() or not np.isfinite(quat).all() or norm < 1e-9:
            return
        rotation = pin.Quaternion(*(quat / norm)).matrix()
        self.remember(key, pin.SE3(rotation, xyz))

    def joint_pose(self, names, positions):
        if self.model is None or len(names) != len(positions):
            return None
        values = dict(zip(names, positions))
        if any(n not in values or not math.isfinite(values[n]) for n in self.names):
            return None
        q = pin.neutral(self.model)
        q[self.indices] = [values[n] for n in self.names]
        pin.framesForwardKinematics(self.model, self.data, q)
        pose = self.data.oMf[self.base_id].inverse() * self.data.oMf[self.ee_id]
        return q, pose

    def preview(self, msg):
        result = self.joint_pose(msg.name, msg.position)
        if result is not None and self.clutch:
            q, pose = result
            self.joint_samples['IK'] = q[self.indices].copy()
            self.remember('IK', pose)

    def feedback(self, msg):
        result = self.joint_pose(msg.joint_names, msg.feedback.positions)
        if result is not None and self.clutch:
            self.joint_samples['ACTUAL'] = result[0][self.indices].copy()
            self.remember('ACTUAL', result[1])

    def report(self):
        if not self.clutch:
            return
        now = time.monotonic()
        parts = []
        for key in ('TARGET', 'MAPPED', 'IK', 'ACTUAL'):
            if key not in self.latest:
                parts.append(f'{key}: waiting')
                continue
            pose, received = self.latest[key]
            age = now - received
            freshness = f' STALE({age:.2f}s)' if age > 0.3 else ''
            parts.append(f'{key}: {delta_text(self.anchors[key], pose)}{freshness}')
        for key, values in self.joint_samples.items():
            q3, q6, q7 = np.degrees(values[[2, 5, 6]])
            bound, margin = ar3r_wrist_bounds(q6, q7)
            flag = ' OUTSIDE' if margin < 0 else ''
            parts.append(f'{key} q3/q6/q7=({q3:+.2f},{q6:+.2f},{q7:+.2f})deg '
                         f'wrist |q7|<={bound:.2f}deg margin={margin:.2f}deg{flag}')
        self.get_logger().info(' | '.join(parts))


def main():
    rclpy.init()
    node = DirectionMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

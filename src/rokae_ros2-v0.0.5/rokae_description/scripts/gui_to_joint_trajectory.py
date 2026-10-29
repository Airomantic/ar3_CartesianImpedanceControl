#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration, Time


class GuiToTrajectory(Node):
    def __init__(self):
        super().__init__('gui_to_joint_trajectory')

        self.declare_parameter(
            'joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
        )
        self.declare_parameter(
            'trajectory_topic',
            '/position_joint_trajectory_controller/joint_trajectory',
        )
        self.declare_parameter(
            'follow_joint_trajectory_action',
            '/position_joint_trajectory_controller/follow_joint_trajectory',
        )
        self.declare_parameter('use_follow_joint_trajectory_action', False)
        self.declare_parameter('trajectory_duration', 0.25)
        self.declare_parameter('publish_period', 0.1)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self._trajectory_topic = str(self.get_parameter('trajectory_topic').value)
        self._action_name = str(self.get_parameter('follow_joint_trajectory_action').value)
        self._use_action = bool(self.get_parameter('use_follow_joint_trajectory_action').value)
        self._traj_duration = float(self.get_parameter('trajectory_duration').value)
        self._publish_period = float(self.get_parameter('publish_period').value)

        self.sub = self.create_subscription(
            JointState,
            '/joint_states_gui',
            self.callback,
            10,
        )

        self._pub = None
        self._action_client = None
        if self._use_action:
            self._action_client = ActionClient(
                self, FollowJointTrajectory, self._action_name
            )
            self.get_logger().info(
                f'GUI->轨迹: FollowJointTrajectory ({self._action_name})'
            )
        else:
            self._pub = self.create_publisher(
                JointTrajectory, self._trajectory_topic, 10
            )
            self.get_logger().info(
                f'GUI->轨迹: 话题 {self._trajectory_topic} '
                f'(duration={self._traj_duration:.2f}s)'
            )

        # Wall-clock throttle: independent of /clock / use_sim_time.
        self._last_pub_wall = 0.0
        self._warned_no_server = False
        self._goal_handle = None

    def _build_trajectory(self, positions):
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)
        # stamp=0 → execute immediately (avoid stale-vs-/clock rejection)
        traj.header.stamp = Time(sec=0, nanosec=0)
        point = JointTrajectoryPoint()
        point.positions = positions
        n = len(self.joint_names)
        point.velocities = [0.0] * n
        dur_ns = int(max(self._traj_duration, 0.05) * 1e9)
        point.time_from_start = Duration(
            sec=dur_ns // 1_000_000_000,
            nanosec=dur_ns % 1_000_000_000,
        )
        traj.points = [point]
        return traj

    def callback(self, msg: JointState):
        try:
            now_wall = time.monotonic()
            if now_wall - self._last_pub_wall < self._publish_period:
                return
            self._last_pub_wall = now_wall

            positions = [0.0] * len(self.joint_names)
            name_to_index = {name: idx for idx, name in enumerate(self.joint_names)}
            matched = 0
            for i, name in enumerate(msg.name):
                if i >= len(msg.position):
                    continue
                idx = name_to_index.get(name)
                if idx is not None:
                    positions[idx] = float(msg.position[i])
                    matched += 1
            if matched == 0:
                return

            traj = self._build_trajectory(positions)

            if self._action_client is not None:
                if not self._action_client.wait_for_server(timeout_sec=0.0):
                    if not self._warned_no_server:
                        self.get_logger().warn(
                            f'FollowJointTrajectory 未就绪: {self._action_name}'
                            '（等待 joint_trajectory_controller 激活）'
                        )
                        self._warned_no_server = True
                    return
                self._warned_no_server = False
                # Cancel previous in-flight goal so slider updates are not queued behind 1s trajs.
                if self._goal_handle is not None:
                    try:
                        self._goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                    self._goal_handle = None
                goal = FollowJointTrajectory.Goal()
                goal.trajectory = traj
                send_future = self._action_client.send_goal_async(goal)
                send_future.add_done_callback(self._on_goal_sent)
            elif self._pub is not None:
                self._pub.publish(traj)
        except Exception as e:
            self.get_logger().error(f'gui_to_joint_trajectory: {e}')

    def _on_goal_sent(self, future):
        try:
            goal_handle = future.result()
            if goal_handle is None:
                return
            if not goal_handle.accepted:
                self.get_logger().warning(
                    'FollowJointTrajectory 目标被拒绝（检查控制器是否为 active）'
                )
                return
            self._goal_handle = goal_handle
        except Exception as e:
            self.get_logger().error(f'发送 FollowJointTrajectory 失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GuiToTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

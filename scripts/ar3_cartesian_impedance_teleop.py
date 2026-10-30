#!/usr/bin/env python3
"""Daimon pose/clutch -> latest targets for the SDK's 1 ms Cartesian impedance loop."""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from cartesian_teleop_math import limit_pose, map_pose, matrix_rpy, quaternion_matrix, rpy_matrix


class CartesianTeleop(Node):
    def __init__(self):
        super().__init__("ar3_cartesian_impedance_teleop")
        self.declare_parameter("dry_run", True)
        defaults = {
            "base_frame": "AR3-3_07R-W4C5C5_base",
            "target_topic": "/target_robot/right_ee/target",
            "robot_state_topic": "/target_robot/right_ee/state",
            "clutch_topic": "/clutch/right",
            "target_direction_rpy_deg": [-90., 0., 0.],
            "rate_hz": 100., "target_timeout_s": 0.25, "feedback_timeout_s": 0.5,
            # MessageBridge forwards right_teleop_en when present; its protocol
            # does not promise a periodic clutch heartbeat. Zero means latched.
            "clutch_timeout_s": 0., "max_linear_speed_m_s": 0.04,
            "service_timeout_s": 2.0,
            "max_angular_speed_rad_s": 0.15,
            "max_translation_lead_m": 0.04, "max_rotation_lead_rad": 0.05,
            # Command shaping is intentionally tighter than the existing SDK
            # tracking stop limits. Feedback can move between ROS samples;
            # touching a shaping boundary must not end a healthy RT hold.
            "max_tracking_error_m": 0.05, "max_tracking_error_rad": 0.15,
            "direction_diagnostics_hz": 1.0,
            "desired_wrench": [0.] * 6,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.p = lambda key: self.get_parameter(key).value
        for key in ("rate_hz", "target_timeout_s", "feedback_timeout_s",
                    "max_linear_speed_m_s", "max_angular_speed_rad_s", "service_timeout_s",
                    "max_translation_lead_m", "max_rotation_lead_rad",
                    "max_tracking_error_m", "max_tracking_error_rad"):
            if not np.isfinite(self.p(key)) or self.p(key) <= 0:
                raise ValueError(f"{key} must be finite and positive")
        self.validate_tracking_limits()
        if not np.isfinite(self.p("clutch_timeout_s")) or self.p("clutch_timeout_s") < 0:
            raise ValueError("clutch_timeout_s must be finite and nonnegative")
        if not np.isfinite(self.p("direction_diagnostics_hz")) or not 0 <= self.p("direction_diagnostics_hz") <= 10:
            raise ValueError("direction_diagnostics_hz must be between 0 and 10")
        for key in ("target_direction_rpy_deg", "desired_wrench"):
            values = self.p(key)
            if len(values) != (3 if key == "target_direction_rpy_deg" else 6) or not np.all(np.isfinite(values)):
                raise ValueError(f"invalid {key}")
        self.correction = rpy_matrix(np.deg2rad(self.p("target_direction_rpy_deg")))
        self.state = None
        self.pose = None
        self.target = None
        self.state_at = self.pose_at = self.target_at = self.clutch_at = 0.
        self.clutch = False
        self.active = False
        self.armed = False
        self.sdk_active = False
        self.holding = self.resuming = False
        self.last_values = self.session_parameters = None
        self.pending = None
        self.stop_waiting = False
        self.stop_failed_reason = ""
        self.stop_retry_at = 0.
        self.must_release = True
        self.source_anchor = self.robot_anchor = self.last_command = None
        self.takeover_at = 0.
        self.direction_report_at = 0.
        self.last_tick = time.monotonic()
        prefix = "/ar3/cartesian_impedance"
        self.command_pub = self.create_publisher(Float64MultiArray, prefix+"/command", 1)
        self.preview_pub = self.create_publisher(Float64MultiArray, prefix+"/preview_command", 1)
        self.pose_pub = self.create_publisher(PoseStamped, self.p("robot_state_topic"), 1)
        self.create_subscription(Float64MultiArray, prefix+"/state", self.on_state, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, prefix+"/tcp_pose", self.on_pose, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self.p("target_topic"), self.on_target, qos_profile_sensor_data)
        # Match MessageBridge's reliable publisher and retain a quick release /
        # press pair instead of overwriting the release with queue depth one.
        self.create_subscription(Bool, self.p("clutch_topic"), self.on_clutch, 10)
        self.create_subscription(String, prefix+"/status", self.on_status,
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.enable_client = self.create_client(SetBool, prefix+"/enable")
        self.stop_client = self.create_client(Trigger, prefix+"/stop")
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(1./self.p("rate_hz"), self.tick)
        self.get_logger().warning(
            f"RT Cartesian teleop: release then press clutch; dry_run={self.p('dry_run')}; "
            f"clutch_timeout_s={self.p('clutch_timeout_s')} (0=latched); "
            "SDK state/TCP and target watchdogs remain enabled")
        axes = "; ".join(f"source +{axis} -> base {np.round(self.correction[:, i], 4).tolist()}"
                         for i, axis in enumerate("XYZ"))
        self.get_logger().info(f"Direction mapping RPY(deg)={self.p('target_direction_rpy_deg')}: {axes}")
        self.get_logger().info(
            f"Latest pose targets at {self.p('rate_hz'):.0f} Hz; SDK interpolates at 1000 Hz; "
            f"translation limit={self.p('max_linear_speed_m_s'):.3f} m/s; "
            "K/desired wrench fixed during each continuous RT session; D/M unspecified and not applied")
        self.get_logger().info(
            "After SDK active, clutch release holds the last TCP with zero desired wrench; "
            "press again to re-anchor and resume; explicit stop, faults and watchdogs still stop RT")
        self.get_logger().info(
            f"Command lead limits={self.p('max_translation_lead_m')*1000:.1f} mm/"
            f"{self.p('max_rotation_lead_rad'):.3f} rad; tracking stop limits="
            f"{self.p('max_tracking_error_m')*1000:.1f} mm/"
            f"{self.p('max_tracking_error_rad'):.3f} rad")

    def validate_tracking_limits(self):
        for lead, tracking in (("max_translation_lead_m", "max_tracking_error_m"),
                               ("max_rotation_lead_rad", "max_tracking_error_rad")):
            if self.p(lead) > self.p(tracking):
                raise ValueError(f"{lead} must not exceed {tracking}")

    def on_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == "dry_run":
                if type(parameter.value) is not bool:
                    return SetParametersResult(successful=False, reason="dry_run must be boolean")
                self.halt("dry_run changed; release clutch before takeover")
                self.get_logger().info(f"dry_run={parameter.value}; release then press clutch")
        return SetParametersResult(successful=True)

    def halt(self, reason):
        repeated_release = (reason == "Clutch released" and self.pending is not None
                            and self.pending[0] == "stop")
        if self.active or self.holding or (self.armed and not self.stop_waiting and not repeated_release):
            self.get_logger().warning(reason)
        self.active = False
        self.sdk_active = self.holding = self.resuming = False
        self.last_values = self.session_parameters = None
        self.must_release = True
        self.source_anchor = None
        # Serialize stop after any outstanding enable; never re-arm on a stale response.
        if (not self.stop_failed_reason and self.pending is None and self.armed
                and time.monotonic() >= self.stop_retry_at
                and self.stop_client.service_is_ready()):
            self.pending = ("stop", self.stop_client.call_async(Trigger.Request()), time.monotonic())

    def latch_stop_failure(self, reason):
        # An explicit controller failure is terminal, unlike a lost RPC reply.
        # Preserve the diagnostic and wait for confirmed recovery; repeating stop
        # requests must not conceal the error or make another takeover possible.
        if reason != self.stop_failed_reason:
            self.get_logger().error(
                f"SDK {reason}; automatic stop retries suspended; takeover blocked until stopped confirmation")
        self.stop_failed_reason = reason
        self.active = False
        self.sdk_active = self.holding = self.resuming = False
        self.last_values = self.session_parameters = None
        self.armed = self.stop_waiting = True
        self.must_release = True
        self.source_anchor = None
        if self.pending is not None:
            self.pending[1].cancel()
            self.pending = None

    def on_status(self, msg):
        # A controller-side watchdog may stop while measurements keep streaming.
        # Halt the local session as well; a stop RPC confirms cleanup completion.
        if msg.data.startswith("stop failed:"):
            self.latch_stop_failure(msg.data)
            return
        if msg.data.startswith("active:") and self.armed and (self.active or self.holding):
            self.sdk_active = True
        if (msg.data.startswith(("fault:", "offline:")) or
                "RT monitoring restoration failed" in msg.data or "monitoring unavailable" in msg.data):
            self.halt(f"SDK {msg.data}; release clutch")
            return
        if self.sdk_active and msg.data.startswith(("connecting:", "monitoring:", "armed:", "initializing:")):
            self.halt(f"SDK left active RT session: {msg.data}; release clutch")
            return
        if self.stop_failed_reason:
            if msg.data.startswith("stopped:"):
                self.stop_failed_reason = ""
                self.active = self.armed = self.stop_waiting = False
                self.must_release = True
                self.source_anchor = None
                self.get_logger().info("SDK stop confirmed after failure; release then press clutch to take over")
            return
        if msg.data.startswith(("stopping:", "stopped:")) and (self.active or self.armed):
            self.halt(f"SDK {msg.data}; release clutch")

    def on_state(self, msg):
        values = np.asarray(msg.data, dtype=float)
        if values.shape != (30,) or not np.all(np.isfinite(values)):
            self.halt("Invalid SDK state")
            return
        self.state, self.state_at = values.copy(), time.monotonic()

    def decode_pose(self, msg):
        if msg.header.frame_id.lstrip("/") != self.p("base_frame"):
            raise ValueError("pose frame differs from robot base")
        p, q = msg.pose.position, msg.pose.orientation
        position = np.array([p.x, p.y, p.z])
        if not np.all(np.isfinite(position)):
            raise ValueError("invalid pose position")
        return position, quaternion_matrix([q.x, q.y, q.z, q.w])

    def on_pose(self, msg):
        try:
            self.pose = self.decode_pose(msg)
        except ValueError as error:
            self.halt(str(error))
            return
        self.pose_at = time.monotonic()
        self.pose_pub.publish(msg)  # Actual SDK TCP pose for Daimon's feedback loop.

    def on_target(self, msg):
        # The released hand cannot change a held robot target. Some hand
        # publishers reset their target while disabled; a new valid message is
        # required after the next press before following resumes.
        if self.holding and not self.clutch:
            return
        try:
            self.target = self.decode_pose(msg)
        except ValueError as error:
            self.halt(str(error))
            return
        self.target_at = time.monotonic()

    def on_clutch(self, msg):
        self.clutch, self.clutch_at = bool(msg.data), time.monotonic()
        if not self.clutch:
            if self.holding:
                return  # Repeated release packets cannot move the held target.
            if (self.active and self.armed and self.sdk_active and self.pending is None
                    and not self.stop_waiting and not self.stop_failed_reason
                    and not self.p("dry_run") and self.last_values is not None
                    and np.all(self.last_values[6:12] == 0.)):
                self.active = self.resuming = False
                self.holding = True
                self.must_release = False
                self.source_anchor = None
                self.get_logger().info(
                    "Clutch released: holding the last commanded TCP in the active RT session; "
                    "zero desired wrench, feedback and force guards remain active")
                return
            self.halt("Clutch released")
            self.must_release = False

    def validate_tracking_error(self, command):
        p, r = command
        translation_error = np.linalg.norm(p-self.pose[0])
        rotation_lead = np.arccos(np.clip((np.trace(r @ self.pose[1].T)-1.)/2., -1., 1.))
        if (translation_error > self.p("max_tracking_error_m") + 1e-9 or
                rotation_lead > self.p("max_tracking_error_rad") + 1e-9):
            mode = "hold" if self.holding or self.resuming else "follow"
            raise ValueError(
                f"Measured TCP exceeded tracking limits ({mode}): "
                f"translation={translation_error*1000:.3f} mm "
                f"(limit={self.p('max_tracking_error_m')*1000:.3f} mm), "
                f"rotation={rotation_lead:.6f} rad "
                f"(limit={self.p('max_tracking_error_rad'):.6f} rad); "
                "release clutch to re-anchor")

    def publish_command(self, values):
        msg = Float64MultiArray(data=values.tolist())
        self.preview_pub.publish(msg)
        if not self.p("dry_run"):
            self.command_pub.publish(msg)
            self.last_values = values.copy()

    def publish_hold(self):
        try:
            self.validate_tracking_error(self.last_command)
            self.publish_command(self.last_values)
        except ValueError as error:
            self.halt(str(error))

    def report_direction(self, now, mapped, command):
        rate = self.p("direction_diagnostics_hz")
        if rate == 0 or now-self.direction_report_at < 1./rate:
            return
        self.direction_report_at = now
        vectors = (
            ("SOURCE", self.target[0]-self.source_anchor[0]),
            ("MAPPED", mapped[0]-self.robot_anchor[0]),
            ("COMMAND", command[0]-self.robot_anchor[0]),
            ("ACTUAL", self.pose[0]-self.robot_anchor[0]),
        )
        text = " | ".join(f"{name} dXYZ(mm)=[{', '.join(f'{v:+.1f}' for v in delta*1000.)}]"
                          for name, delta in vectors)
        wrench = ', '.join(f'{v:+.2f}' for v in self.state[6:12])
        self.get_logger().info(
            f"direction: {text}; MAPPED/COMMAND/ACTUAL use SDK base axes; "
            f"RT_EXTERNAL_BASE[SDK6]=[{wrench}]")

    def tick(self):
        now = time.monotonic()
        dt = min(now-self.last_tick, 1./self.p("rate_hz"))
        self.last_tick = now
        if self.stop_failed_reason:
            return
        if self.pending is not None:
            kind, future, submitted_at = self.pending
            if not future.done():
                if now-submitted_at <= self.p("service_timeout_s"):
                    return
                future.cancel()
                self.pending = None
                # A lost enable reply does not prove the robot remained disarmed.
                self.armed = True
                self.halt(f"SDK {kind} service timed out; requesting stop")
                return
            self.pending = None
            try:
                response = future.result()
                if not response.success:
                    if kind == "stop" and response.message.startswith("stop failed:"):
                        self.latch_stop_failure(response.message)
                        return
                    raise RuntimeError(response.message)
                if kind == "stop":
                    # Stop is asynchronous in the SDK node. An accepted request
                    # does not prove the RT loop ended and the robot is idle.
                    if response.message.startswith("stopped"):
                        self.armed = self.stop_waiting = False
                        self.active = False
                        self.must_release = True
                        self.source_anchor = None
                    elif response.message.startswith("stopping:"):
                        self.armed = self.stop_waiting = True
                        self.stop_retry_at = now + 0.2
                        return
                    else:
                        raise RuntimeError(f"unrecognized stop result: {response.message}")
                else:
                    self.armed = True
            except Exception as error:
                self.get_logger().error(f"SDK {kind} failed: {error}")
                self.armed = True  # Treat uncertain RPC outcomes conservatively.
                if kind == "stop":
                    self.stop_waiting = True
                    self.stop_retry_at = now + 0.2
                self.halt("SDK service failure")
                return
        if not self.active and self.armed and not self.holding:
            self.halt("Stopping inactive session")
            return
        if self.state is None or self.pose is None:
            return
        stale = []
        for name, received_at, timeout in (
            ("SDK state", self.state_at, self.p("feedback_timeout_s")),
            ("SDK TCP pose", self.pose_at, self.p("feedback_timeout_s")),
            ("Daimon clutch heartbeat", self.clutch_at, self.p("clutch_timeout_s")),
        ):
            if timeout > 0 and now-received_at > timeout:
                stale.append(f"{name} age={now-received_at:.3f}s > {timeout:.3f}s")
        if stale:
            self.halt("Feedback stale: " + "; ".join(stale) + "; release clutch")
            return
        if self.holding:
            if self.clutch and not self.must_release:
                self.holding = False
                self.resuming = self.active = True
                self.takeover_at = now
                self.robot_anchor = (self.pose[0].copy(), self.pose[1].copy())
                self.source_anchor = None
                # Keep last_command as the slew origin. The new hand anchor
                # uses measured TCP without jumping the existing RT target.
                self.get_logger().info(
                    "Clutch resume: same RT session, anchored to measured TCP; waiting for fresh target")
            self.publish_hold()
            return
        if not self.clutch or self.must_release:
            return
        if not self.active:
            self.active = True
            self.takeover_at = now
            self.robot_anchor = (self.pose[0].copy(), self.pose[1].copy())
            self.last_command = self.robot_anchor
            self.source_anchor = None
            self.get_logger().info("Clutch takeover anchored to measured TCP; waiting for fresh target")
            return
        if self.target_at <= self.takeover_at:
            if now-self.takeover_at > self.p("target_timeout_s"):
                self.halt("No fresh target after clutch takeover")
            elif self.resuming:
                self.publish_hold()
            return
        if now-self.target_at > self.p("target_timeout_s"):
            self.halt(f"Daimon target stale: age={now-self.target_at:.3f}s; release clutch")
            return
        if self.source_anchor is None:
            self.source_anchor = (self.target[0].copy(), self.target[1].copy())
        self.resuming = False
        if not self.p("dry_run") and not self.armed:
            if self.enable_client.service_is_ready():
                self.armed = True  # An outstanding enable may already have reached the controller.
                self.pending = ("enable", self.enable_client.call_async(SetBool.Request(data=True)), now)
            return
        try:
            # Test displacement from the already-issued target before shaping
            # a new one; a slew step must not conceal a tracking-limit breach.
            self.validate_tracking_error(self.last_command)
            mapped = map_pose(self.target, self.source_anchor, self.robot_anchor, self.correction)
            # Bound tracking error even while the RT session initializes or an
            # external contact holds the robot. The SDK consumes the latest
            # target continuously; this is not a queue of motion segments.
            bounded_target = limit_pose(self.pose, mapped, self.p("max_translation_lead_m"),
                                        self.p("max_rotation_lead_rad"))
            command = limit_pose(self.last_command, bounded_target,
                                 self.p("max_linear_speed_m_s")*dt,
                                 self.p("max_angular_speed_rad_s")*dt)
            p, r = command
            # Feedback motion can temporarily put the previous command beyond
            # the soft lead cap. Preserve slew continuity while returning to
            # that cap; the separate tracking guard still bounds displacement.
            self.validate_tracking_error(command)
            self.last_command = command
            self.report_direction(now, mapped, command)
            if self.session_parameters is None:
                self.session_parameters = np.concatenate((self.p("desired_wrench"), self.state[12:]))
            values = np.concatenate((p, matrix_rpy(r), self.session_parameters))
            self.publish_command(values)
        except ValueError as error:
            self.halt(str(error))


def main():
    rclpy.init()
    node = CartesianTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.active = node.sdk_active = node.holding = node.resuming = False
        node.last_values = node.session_parameters = None
        node.must_release = True
        if rclpy.ok() and not node.stop_failed_reason and node.stop_client.service_is_ready():
            future = node.stop_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=1.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

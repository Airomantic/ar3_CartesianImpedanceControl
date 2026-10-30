#!/usr/bin/env python3
"""Daimon pose/clutch -> latest targets for the SDK's 1 ms Cartesian impedance loop."""

import json
import time
import uuid
from pathlib import Path

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
from cartesian_return_home import ReturnHomePath, load_home, pose_error


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
            "return_home_file": str(Path(__file__).resolve().parents[1] / "config/cartesian_home.json"),
            "return_home_linear_speed_m_s": 0.02,
            "return_home_angular_speed_rad_s": 0.1,
            "return_home_timeout_s": 60.,
            "return_home_workspace_radius_m": 0.25,
            "return_home_position_tolerance_m": 0.02,
            "return_home_rotation_tolerance_rad": 0.04,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.p = lambda key: self.get_parameter(key).value
        for key in ("rate_hz", "target_timeout_s", "feedback_timeout_s",
                    "max_linear_speed_m_s", "max_angular_speed_rad_s", "service_timeout_s",
                    "max_translation_lead_m", "max_rotation_lead_rad",
                    "max_tracking_error_m", "max_tracking_error_rad",
                    "return_home_linear_speed_m_s", "return_home_angular_speed_rad_s",
                    "return_home_timeout_s", "return_home_workspace_radius_m",
                    "return_home_position_tolerance_m", "return_home_rotation_tolerance_rad"):
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
        self.sdk_status = ""
        self.holding = self.resuming = False
        self.last_values = self.session_parameters = None
        self.pending = None
        self.stop_waiting = False
        self.stop_failed_reason = ""
        self.stop_retry_at = 0.
        self.must_release = True
        self.source_anchor = self.robot_anchor = self.last_command = None
        self.session_anchor = None
        self.home_values = self.home_pose = None
        self.home_error = ""
        self.home_path = None
        self.home_started_at = self.home_settled_at = None
        self.home_request_id = ""
        self.home_cancel_service = None
        self.home_cancel_service_name = ""
        self.home_status = "idle"
        self.home_phase = ""
        self.home_message = "No return requested"
        self.home_progress = None
        self.home_report_at = 0.
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
        self.home_status_pub = self.create_publisher(
            String, prefix+"/teleop/return_home_status",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, prefix+"/teleop/return_home", self.on_return_home)
        self.create_service(Trigger, prefix+"/teleop/cancel_return_home", self.on_cancel_return_home)
        self.try_load_home()
        self.publish_home_status()
        self.create_timer(1., self.publish_home_status)
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
        if self.home_values is not None:
            self.get_logger().info(
                f"Saved return-home TCP={self.home_values.tolist()} in {self.p('base_frame')}; "
                f"measured arrival tolerance={self.p('return_home_position_tolerance_m')*1000.:.1f} mm/"
                f"{self.p('return_home_rotation_tolerance_rad'):.3f} rad; "
                "requested return requires released clutch; idle SDK may start a new RT hold")
        else:
            self.get_logger().info(f"Return-home origin unavailable; manual teleop remains available: {self.home_error}")

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
        self.fail_return_home(reason)
        repeated_release = (reason == "Clutch released" and self.pending is not None
                            and self.pending[0] == "stop")
        if self.active or self.holding or (self.armed and not self.stop_waiting and not repeated_release):
            self.get_logger().warning(reason)
        self.active = False
        self.sdk_active = self.holding = self.resuming = False
        self.last_values = self.session_parameters = None
        self.must_release = True
        self.source_anchor = None
        self.session_anchor = None
        # Serialize stop after any outstanding enable; never re-arm on a stale response.
        if (not self.stop_failed_reason and self.pending is None and self.armed
                and time.monotonic() >= self.stop_retry_at
                and self.stop_client.service_is_ready()):
            self.pending = ("stop", self.stop_client.call_async(Trigger.Request()), time.monotonic())

    def latch_stop_failure(self, reason):
        self.fail_return_home(reason)
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
        self.session_anchor = None
        if self.pending is not None:
            self.pending[1].cancel()
            self.pending = None

    def on_status(self, msg):
        self.sdk_status = msg.data
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
        previous_clutch = self.clutch
        self.clutch, self.clutch_at = bool(msg.data), time.monotonic()
        if self.clutch != previous_clutch:
            self.publish_home_status()
        if self.home_path is not None and self.clutch:
            self.halt("Return home cancelled by clutch press; release then press again after stop")
            return
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

    def try_load_home(self):
        # Once loaded, the origin stays fixed for this teleop process. Missing
        # configuration must never prevent ordinary manual teleoperation.
        if self.home_pose is None:
            try:
                self.home_values, self.home_pose = load_home(self.p("return_home_file"), self.p("base_frame"))
                self.home_error = ""
            except (OSError, ValueError, TypeError) as error:
                self.home_error = str(error)

    def publish_home_status(self):
        if self.home_pose is None:
            self.try_load_home()
        self.update_home_progress()
        data = {
            "request_id": self.home_request_id, "state": self.home_status,
            "message": self.home_message, "home_configured": self.home_pose is not None,
            "phase": self.home_phase, "startup_home_supported": True,
            "home_pose": None if self.home_values is None else self.home_values.tolist(),
            "frame_id": self.p("base_frame"), "clutch_pressed": self.clutch,
            "holding": self.holding, "sdk_active": self.sdk_active, "dry_run": self.p("dry_run"),
            "home_error": self.home_error,
            "id_scoped_cancel": True, "cancel_service": self.home_cancel_service_name,
            "progress": self.home_progress,
        }
        self.home_status_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def fail_return_home(self, reason):
        if self.home_path is not None:
            self.update_home_progress()
            self.home_path = None
            self.home_phase = ""
            self.home_status, self.home_message = "failed", reason
            self.publish_home_status()

    def update_home_progress(self):
        """Keep the final in-motion errors after a stop changes the measured pose."""
        if self.home_path is None or self.pose is None or self.last_command is None:
            return
        path = self.home_path
        candidate, _ = path.candidate(1./self.p("rate_hz"))
        lead_m, lead_rad = pose_error(candidate, self.pose)
        actual_m, actual_rad = pose_error(self.pose, self.home_pose)
        command_m, command_rad = pose_error(self.last_command, self.home_pose)
        tracking_m, tracking_rad = pose_error(self.last_command, self.pose)
        if not self.sdk_active:
            gate = "waiting_for_rt_active"
        elif path.elapsed >= path.duration:
            gate = "settling_at_home"
        else:
            blocked = []
            if lead_m > self.p("max_translation_lead_m") + 1e-9:
                blocked.append("translation_lead")
            if lead_rad > self.p("max_rotation_lead_rad") + 1e-9:
                blocked.append("rotation_lead")
            gate = "paused_" + "_and_".join(blocked) if blocked else "moving"
        self.home_progress = {
            "gate": gate,
            "wall_elapsed_s": round(time.monotonic()-self.home_started_at, 3),
            "path_elapsed_s": round(path.elapsed, 3),
            "path_duration_s": round(path.duration, 3),
            "actual_to_home_mm_rad": [round(actual_m*1000., 3), round(actual_rad, 6)],
            "command_to_home_mm_rad": [round(command_m*1000., 3), round(command_rad, 6)],
            "tracking_mm_rad": [round(tracking_m*1000., 3), round(tracking_rad, 6)],
            "lead_limits_mm_rad": [self.p("max_translation_lead_m")*1000., self.p("max_rotation_lead_rad")],
            "actual_tcp": np.concatenate((self.pose[0], matrix_rpy(self.pose[1]))).tolist(),
            "command_tcp": np.concatenate((self.last_command[0], matrix_rpy(self.last_command[1]))).tolist(),
            "measured_feedback_sdk6": None if self.state is None else self.state[6:12].tolist(),
        }

    def home_progress_text(self):
        self.update_home_progress()
        progress = self.home_progress
        if progress is None:
            return "progress unavailable"
        return (f"gate={progress['gate']}; path={progress['path_elapsed_s']:.2f}/"
                f"{progress['path_duration_s']:.2f}s, wall={progress['wall_elapsed_s']:.1f}s; "
                f"actual_to_home={progress['actual_to_home_mm_rad']} mm/rad; "
                f"command_to_home={progress['command_to_home_mm_rad']} mm/rad; "
                f"tracking={progress['tracking_mm_rad']} mm/rad; "
                f"lead_limits={progress['lead_limits_mm_rad']} mm/rad")

    def on_return_home(self, request, response):
        del request
        try:
            if self.home_path is not None:
                raise ValueError("A return-home request is already running")
            if self.clutch:
                raise ValueError("Release the Daimon clutch before saving/returning home")
            if (self.p("dry_run") or self.active or self.resuming or self.pending is not None
                    or self.stop_waiting or self.stop_failed_reason):
                raise ValueError("Return home requires dry_run=false and a healthy inactive teleop")
            existing_hold = self.armed and self.sdk_active and self.holding and not self.must_release
            idle_start = (not self.armed and not self.sdk_active and not self.holding
                          and self.sdk_status.startswith(("monitoring:", "stopped:")))
            if not existing_hold and not idle_start:
                raise ValueError("Return home requires an owned RT hold or an idle monitoring/stopped SDK")
            if idle_start and self.clutch_at <= 0.:
                raise ValueError("Clutch state is unknown; release the Daimon clutch before startup return")
            now = time.monotonic()
            if (self.state is None or self.pose is None
                    or now-self.state_at > self.p("feedback_timeout_s")
                    or now-self.pose_at > self.p("feedback_timeout_s")):
                raise ValueError("SDK pose/state feedback is missing or stale")
            if existing_hold and (self.last_values is None or self.session_parameters is None
                    or not np.all(self.last_values[6:12] == 0.)
                    or not np.all(self.session_parameters[:6] == 0.)):
                raise ValueError("Return home requires zero desired wrench in the existing RT session")
            if idle_start and not np.all(np.asarray(self.p("desired_wrench")) == 0.):
                raise ValueError("Startup return requires zero desired wrench")
            if idle_start and not self.enable_client.service_is_ready():
                raise ValueError("SDK enable service is unavailable")
            if (self.p("clutch_timeout_s") > 0.
                    and now-self.clutch_at > self.p("clutch_timeout_s")):
                raise ValueError("Daimon clutch feedback is stale")
            self.try_load_home()
            if self.home_pose is None:
                raise ValueError(f"No valid saved home: {self.home_error}")
            start = self.pose if idle_start else self.last_command
            anchor = self.pose if idle_start else self.session_anchor
            self.validate_tracking_error(start)
            if anchor is None:
                raise ValueError("Original RT session anchor is unavailable; re-start teleop session")
            radius = self.p("return_home_workspace_radius_m")
            for name, pose in (("saved home", self.home_pose), ("last target", start),
                               ("measured TCP", self.pose)):
                if np.linalg.norm(pose[0]-anchor[0]) > radius:
                    raise ValueError(f"{name} is outside {radius:g} m of the original RT session anchor")
            path = ReturnHomePath(
                start, self.home_pose,
                min(self.p("return_home_linear_speed_m_s"), self.p("max_linear_speed_m_s")),
                min(self.p("return_home_angular_speed_rad_s"), self.p("max_angular_speed_rad_s")))
            if path.duration + 1. > self.p("return_home_timeout_s"):
                raise ValueError("Return path duration exceeds configured timeout")
            request_id = "r_" + uuid.uuid4().hex
            cancel_service_name = "/ar3/cartesian_impedance/teleop/cancel_return_home/" + request_id
            try:
                cancel_service = self.create_service(
                    Trigger, cancel_service_name,
                    lambda request, response, identity=request_id:
                        self.on_cancel_return_home_scoped(request, response, identity))
            except Exception as error:
                raise ValueError(f"Cannot create return-home cancel service: {error}") from error
            # Keep the scoped endpoint until the next accepted request. A late
            # callback must still check its identity before touching that job.
            if self.home_cancel_service is not None:
                self.destroy_service(self.home_cancel_service)
            self.home_cancel_service = cancel_service
            self.home_cancel_service_name = cancel_service_name
            self.home_request_id = request_id
            self.home_progress = None
            self.home_report_at = now
            self.home_path = path
            self.home_started_at, self.home_settled_at = now, None
            self.home_phase = "initializing" if idle_start else "returning"
            self.home_status, self.home_message = "running", "Returning to saved home in the existing RT session"
            if idle_start:
                # An explicit recorder request owns this startup. Keep the
                # measured pose fixed until the SDK confirms the RT loop is
                # active; no hand target or path waypoint starts the session.
                self.session_anchor = tuple(value.copy() for value in self.pose)
                self.robot_anchor = tuple(value.copy() for value in self.pose)
                self.last_command = tuple(value.copy() for value in self.pose)
                self.session_parameters = np.concatenate((np.zeros(6), self.state[12:]))
                self.last_values = np.concatenate((self.pose[0], matrix_rpy(self.pose[1]),
                                                   self.session_parameters))
                self.source_anchor = None
                self.must_release = False
                self.holding = self.armed = True
                self.home_message = "Starting RT hold at measured TCP before returning to saved home"
                try:
                    self.pending = ("home_enable", self.enable_client.call_async(
                        SetBool.Request(data=True)), now)
                except Exception as error:
                    # A transport exception may occur after the server acted.
                    self.halt(f"Startup return enable failed: {error}; requesting stop")
                    raise ValueError(self.home_message) from error
            self.publish_home_status()
            self.get_logger().info(self.home_message + f"; request_id={self.home_request_id}")
            response.success = True
            response.message = json.dumps({"request_id": self.home_request_id, "state": "running",
                                           "cancel_service": cancel_service_name})
        except (ValueError, TypeError) as error:
            response.success = False
            response.message = json.dumps({"request_id": "", "state": "rejected", "message": str(error)})
            self.get_logger().warning(f"Return home rejected: {error}")
        return response

    def on_cancel_return_home_scoped(self, request, response, request_id):
        del request
        if self.home_path is None or request_id != self.home_request_id:
            response.success = False
            response.message = "Return-home request is no longer current; no stop sent"
        else:
            self.halt("Return home cancelled by recorder; protective stop requested")
            response.success = True
            response.message = "Matching return-home request cancelled; protective stop requested"
        return response

    def on_cancel_return_home(self, request, response):
        del request
        if self.home_path is not None:
            self.halt("Return home cancelled by recorder; protective stop requested")
            response.message = "Return home cancelled; protective stop requested"
        else:
            response.message = "No return-home request is running"
        response.success = True
        return response

    def tick_return_home(self, now, dt):
        try:
            if now-self.home_started_at > self.p("return_home_timeout_s"):
                raise ValueError("Return home timed out before measured TCP reached the saved home; "
                                 + self.home_progress_text())
            if now-self.home_report_at >= 2.:
                self.home_report_at = now
                self.get_logger().info("Return home progress: " + self.home_progress_text())
            if not self.sdk_active:
                self.publish_hold()
                return
            if self.home_phase == "initializing":
                self.home_phase = "returning"
                self.home_message = "RT hold active; returning to saved home"
                self.publish_home_status()
            self.validate_tracking_error(self.last_command)
            command, elapsed = self.home_path.candidate(dt)
            position_lead, rotation_lead = pose_error(command, self.pose)
            # A slow or obstructed robot pauses the trajectory clock. Never
            # advance along the recording clock or accumulate targets ahead.
            if (position_lead <= self.p("max_translation_lead_m") + 1e-9
                    and rotation_lead <= self.p("max_rotation_lead_rad") + 1e-9):
                self.validate_tracking_error(command)
                self.last_command = command
                self.home_path.elapsed = elapsed
                self.last_values = np.concatenate((command[0], matrix_rpy(command[1]), self.session_parameters))
            self.publish_command(self.last_values)
            position_error, rotation_error = pose_error(self.pose, self.home_pose)
            reached = (self.home_path.elapsed >= self.home_path.duration
                       and position_error <= self.p("return_home_position_tolerance_m")
                       and rotation_error <= self.p("return_home_rotation_tolerance_rad"))
            if not reached:
                self.home_settled_at = None
            elif self.home_settled_at is None:
                self.home_settled_at = now
            elif now-self.home_settled_at >= 0.3:
                self.update_home_progress()
                self.home_path = None
                self.home_phase = ""
                self.home_status = "succeeded"
                self.home_message = (
                    "Measured TCP within home tolerance "
                    f"({self.p('return_home_position_tolerance_m')*1000.:.1f} mm/"
                    f"{self.p('return_home_rotation_tolerance_rad'):.3f} rad); active RT pose hold continues")
                self.publish_home_status()
                self.get_logger().info(self.home_message)
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
                if kind == "home_enable" and response.success and "existing session retained" in response.message:
                    # Another client armed after the idle check. This request
                    # sent no command, so it must not take over or stop that
                    # client's session, including after a pending cancellation.
                    self.armed = self.active = self.holding = self.sdk_active = False
                    self.last_values = self.session_parameters = None
                    self.session_anchor = None
                    self.must_release = True
                    self.fail_return_home("Startup return refused: SDK session belongs to another controller")
                    self.get_logger().warning("Startup return refused existing SDK session; no command or stop sent")
                    return
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
        if self.home_path is not None:
            self.tick_return_home(now, dt)
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
            self.session_anchor = tuple(value.copy() for value in self.robot_anchor)
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
        if rclpy.ok():
            node.fail_return_home("Teleop program is exiting; protective stop requested")
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

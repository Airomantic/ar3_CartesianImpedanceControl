"""ROS-independent validation and bounded pose path for a saved Cartesian home."""

import json
from pathlib import Path

import numpy as np

from cartesian_teleop_math import limit_pose, rpy_matrix


def pose_error(first, second):
    return (float(np.linalg.norm(first[0] - second[0])),
            float(np.arccos(np.clip((np.trace(first[1] @ second[1].T)-1.)/2., -1., 1.))))


def load_home(path, base_frame):
    path = Path(path).expanduser()
    if path.stat().st_size > 16384:
        raise ValueError("home file is larger than 16 KiB")
    data = json.loads(path.read_text())
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise ValueError("home file requires schema_version=1")
    if data.get("frame_id") != base_frame:
        raise ValueError("home frame differs from the SDK base frame")
    if data.get("source") != "measured_tcp_pose":
        raise ValueError("home must come from measured_tcp_pose")
    values = np.asarray(data.get("tcp_pose"), dtype=float)
    if values.shape != (6,) or not np.all(np.isfinite(values)):
        raise ValueError("home tcp_pose must contain six finite xyz[m]/RPY[rad] values")
    return values.copy(), (values[:3].copy(), rpy_matrix(values[3:]))


class ReturnHomePath:
    """Quintic path with zero end velocities; feedback may pause its progress.

    This is a direct Cartesian path, not an obstacle-avoiding planner. The owner
    retains all state, wrench, tracking and workspace guards while following it.
    """

    def __init__(self, start, goal, linear_speed, angular_speed):
        self.start = tuple(value.copy() for value in start)
        self.goal = tuple(value.copy() for value in goal)
        self.distance, self.angle = pose_error(start, goal)
        if self.angle > np.pi - 1e-5:
            raise ValueError("return-home orientation differs by pi; choose a closer starting pose")
        # max d(10u^3-15u^4+6u^5)/du = 1.875.
        self.duration = max(2., 1.875*self.distance/linear_speed, 1.875*self.angle/angular_speed)
        self.elapsed = 0.

    def candidate(self, dt):
        elapsed = min(self.duration, self.elapsed + max(0., dt))
        if elapsed >= self.duration:
            return self.goal, elapsed
        u = elapsed/self.duration
        progress = u*u*u*(10. + u*(-15. + 6.*u))
        return limit_pose(self.start, self.goal, self.distance*progress, self.angle*progress), elapsed

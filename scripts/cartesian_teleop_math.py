"""ROS-independent pose mapping for the AR3 native Cartesian bridge.

Pose vectors are xyz metres + roll/pitch/yaw radians (Rz @ Ry @ Rx).
"""

import numpy as np


def rpy_matrix(rpy):
    r, p, y = rpy
    sr, sp, sy = np.sin([r, p, y])
    cr, cp, cy = np.cos([r, p, y])
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def matrix_rpy(rotation):
    pitch = np.arctan2(-rotation[2, 0], np.hypot(rotation[0, 0], rotation[1, 0]))
    if np.hypot(rotation[0, 0], rotation[1, 0]) < 1e-8:
        return np.array([0., pitch, np.arctan2(-rotation[0, 1], rotation[1, 1])])
    return np.array([np.arctan2(rotation[2, 1], rotation[2, 2]), pitch,
                     np.arctan2(rotation[1, 0], rotation[0, 0])])


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.all(np.isfinite(q)) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def map_pose(target, source_anchor, robot_anchor, correction):
    """Map clutch-relative spatial increments, including mounted-base rotation."""
    p, r = target
    sp, sr = source_anchor
    rp, rr = robot_anchor
    return rp + correction @ (p-sp), correction @ r @ sr.T @ correction.T @ rr


def limit_pose(previous, target, max_translation, max_rotation):
    """Bound each incremental command on SE(3), without Euler wrap jumps."""
    p0, r0 = previous
    p1, r1 = target
    delta = p1-p0
    distance = np.linalg.norm(delta)
    position = p0 + delta * min(1., max_translation / max(distance, 1e-12))
    rotation_delta = r1 @ r0.T
    angle = np.arccos(np.clip((np.trace(rotation_delta)-1.)/2., -1., 1.))
    if angle <= max_rotation:
        return position, r1.copy()
    # A discontinuous half-turn is not a valid teleop increment.
    if np.pi-angle < 1e-5:
        raise ValueError("target orientation jumped by pi")
    axis = np.array([rotation_delta[2, 1]-rotation_delta[1, 2],
                     rotation_delta[0, 2]-rotation_delta[2, 0],
                     rotation_delta[1, 0]-rotation_delta[0, 1]]) / (2*np.sin(angle))
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    increment = np.eye(3) + np.sin(max_rotation)*skew + (1-np.cos(max_rotation))*(skew@skew)
    return position, increment @ r0

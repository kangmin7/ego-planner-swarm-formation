"""NED/FRD <-> ENU/FLU conversions shared by the px4_bridge nodes.

PX4 (px4_msgs) uses NED for world position/velocity and FRD for body
orientation; ego_planner (this repo) assumes a single right-handed "world"
frame with no tf tree, which we define as ENU/FLU. These are the standard
fixed-rotation conversions (the same ones used by px4_ros_com's
frame_transforms.cpp) re-derived here in plain Python/numpy to avoid adding
a new dependency.
"""
import math

import numpy as np

# Fixed quaternions [w, x, y, z], Hamiltonian convention.
_Q_NED_ENU = (0.0, 0.70710678, 0.70710678, 0.0)  # 180 deg about (East+North)/sqrt(2)
_Q_FRD_FLU = (0.0, 1.0, 0.0, 0.0)                # 180 deg about body X (roll)


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def ned_to_enu_position(x_ned: float, y_ned: float, z_ned: float):
    return y_ned, x_ned, -z_ned


def enu_to_ned_position(x_enu: float, y_enu: float, z_enu: float):
    return y_enu, x_enu, -z_enu


# Velocity transforms are the same linear map as position.
ned_to_enu_velocity = ned_to_enu_position
enu_to_ned_velocity = enu_to_ned_position


def frd_ned_quat_to_flu_enu_quat(q_frd_ned):
    """q_frd_ned: [w, x, y, z] body(FRD) -> world(NED). Returns body(FLU) -> world(ENU)."""
    return _quat_mul(_quat_mul(_Q_NED_ENU, tuple(q_frd_ned)), _Q_FRD_FLU)


def quat_to_rotation_matrix(q):
    """[w, x, y, z] -> 3x3 rotation matrix (body -> world)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def enu_yaw_to_ned_yaw(yaw_enu: float) -> float:
    """ENU yaw (math angle, 0 = East, CCW+) -> PX4/NED heading (0 = North, CW+)."""
    yaw = math.pi / 2.0 - yaw_enu
    return math.atan2(math.sin(yaw), math.cos(yaw))

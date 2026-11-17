"""Small ROS helpers shared by the nodes."""
from __future__ import annotations

import dataclasses
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw: float):
    from geometry_msgs.msg import Quaternion
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def make_path(xs, ys, yaws, frame_id: str, stamp) -> Path:
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for x, y, th in zip(xs, ys, yaws):
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.orientation = quat_from_yaw(float(th))
        path.poses.append(ps)
    return path


def declare_dataclass_params(node, prefix: str, cls) -> dict:
    """Declare ``prefix.<field>`` for every scalar dataclass field; return the values.

    Fields whose default is ``None`` or non-scalar are skipped (they keep their
    dataclass default).
    """
    values = {}
    inst = cls()
    for f in dataclasses.fields(cls):
        default = getattr(inst, f.name)
        if isinstance(default, (bool, int, float, str)):
            name = f"{prefix}.{f.name}"
            node.declare_parameter(name, default)
            values[f.name] = node.get_parameter(name).value
    return values

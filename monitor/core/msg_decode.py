"""
Message → dict conversion shared by the transport backends.

Every backend decodes into the same plain-dict shape so ``DataModel`` and the
UI stay transport-agnostic::

    {"time_stamp": int, "frame_id": str, "sequence": int, "joints": [...]}
    {"time_stamp": int, ..., "position_x": float, "roll": float, ...}

Topic and message type (must match what the robot publishes)::

    /geniarm/joint_state    pb:aimrt.protocols.sensor.JointStateArray
    /geniarm/joint_cmd      pb:aimrt.protocols.sensor.JointCommandArray
    /geniarm/pose_current   pb:genisys.protocols.PoseStamped
    /geniarm/pose_expected  pb:genisys.protocols.PoseStamped

The robot publishes ``PoseStamped`` (aliased to ``Pose`` in ControlModule),
not ``Pose``.
"""

from __future__ import annotations

import json
import math
from typing import Optional

from monitor.proto.joint_command_pb2 import JointCommandArray
from monitor.proto.joint_state_pb2 import JointStateArray
from monitor.proto.pose_pb2 import PoseStamped

# Protobuf field names per message type.
JOINT_STATE_FIELDS = ("position", "velocity", "effort")
JOINT_CMD_FIELDS = ("position", "velocity", "effort", "stiffness", "damping")

# Scalar pose fields exposed to the data model (joints-less messages).
POSE_SCALAR_FIELDS = (
    "position_x", "position_y", "position_z",
    "orientation_x", "orientation_y", "orientation_z", "orientation_w",
    "roll", "pitch", "yaw",
)

# source name (AIMRT config topics key) → protobuf message class.
# ``genisys.protocols.PoseStamped`` lives in monitor/proto/pose_pb2.py.
MSGTYPE_BY_SOURCE = {
    "joint_state": JointStateArray,
    "joint_cmd": JointCommandArray,
    "pose_current": PoseStamped,
    "pose_expected": PoseStamped,
}


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """Quaternion → ZYX Euler angles (roll, pitch, yaw) in radians."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _joints_to_dicts(msg, fields) -> list[dict]:
    return [
        {"name": j.name, **{name: getattr(j, name) for name in fields}}
        for j in msg.joints
    ]


def joint_state_to_dict(msg: JointStateArray) -> dict:
    return {
        "time_stamp": msg.header.time_stamp,
        "frame_id": msg.header.frame_id,
        "sequence": msg.header.sequence_num,
        "joints": _joints_to_dicts(msg, JOINT_STATE_FIELDS),
    }


def joint_cmd_to_dict(msg: JointCommandArray) -> dict:
    return {
        "time_stamp": msg.header.time_stamp,
        "frame_id": msg.header.frame_id,
        "sequence": msg.header.sequence_num,
        "joints": _joints_to_dicts(msg, JOINT_CMD_FIELDS),
    }


def pose_to_dict(msg: PoseStamped, source: str) -> dict:
    qx = msg.pose.orientation.x
    qy = msg.pose.orientation.y
    qz = msg.pose.orientation.z
    qw = msg.pose.orientation.w
    roll, pitch, yaw = quat_to_rpy(qx, qy, qz, qw)
    return {
        "time_stamp": msg.header.time_stamp,
        "frame_id": msg.header.frame_id,
        "sequence": msg.header.sequence_num,
        "source": source,
        "position_x": msg.pose.position.x,
        "position_y": msg.pose.position.y,
        "position_z": msg.pose.position.z,
        "orientation_x": qx,
        "orientation_y": qy,
        "orientation_z": qz,
        "orientation_w": qw,
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
    }


def message_to_dict(source: str, msg) -> Optional[dict]:
    """Convert an already-parsed protobuf message to the transport-neutral dict."""
    if source == "joint_state":
        return joint_state_to_dict(msg)
    if source == "joint_cmd":
        return joint_cmd_to_dict(msg)
    if source in ("pose_current", "pose_expected"):
        return pose_to_dict(msg, source)
    return None


# ---------------------------------------------------------------------------
# Byte-oriented helpers — kept for backends that receive raw payloads
# (the legacy Zenoh subscriber).
# ---------------------------------------------------------------------------

def decode_pb(source: str, pb_bytes: bytes) -> Optional[dict]:
    """Decode protobuf bytes according to *source* type."""
    cls = MSGTYPE_BY_SOURCE.get(source)
    if cls is None:
        return None
    msg = cls()
    msg.ParseFromString(pb_bytes)
    return message_to_dict(source, msg)


def decode_json(source: str, json_bytes: bytes) -> Optional[dict]:
    """Decode a JSON-encoded AIMRT payload according to *source* type.

    Tolerates both camelCase (protobuf JSON mapping) and snake_case keys.
    """
    obj = json.loads(json_bytes)
    header = obj.get("header", {})

    def _hdr() -> dict:
        return {
            "time_stamp": header.get("timeStamp", header.get("time_stamp", 0)),
            "frame_id": header.get("frameId", header.get("frame_id", "")),
            "sequence": header.get("sequenceNum", header.get("sequence_num", 0)),
        }

    if source == "joint_state":
        joints_data = [
            {"name": j.get("name", ""),
             **{name: j.get(name, 0.0) for name in JOINT_STATE_FIELDS}}
            for j in obj.get("joints", [])
        ]
        return {**_hdr(), "joints": joints_data}

    if source == "joint_cmd":
        joints_data = [
            {"name": j.get("name", ""),
             **{name: j.get(name, 0.0) for name in JOINT_CMD_FIELDS}}
            for j in obj.get("joints", [])
        ]
        return {**_hdr(), "joints": joints_data}

    if source in ("pose_current", "pose_expected"):
        pose = obj.get("pose", {})
        pos = pose.get("position", {})
        orient = pose.get("orientation", {})
        qx = orient.get("x", 0.0)
        qy = orient.get("y", 0.0)
        qz = orient.get("z", 0.0)
        qw = orient.get("w", 0.0)
        roll, pitch, yaw = quat_to_rpy(qx, qy, qz, qw)
        return {
            **_hdr(),
            "source": source,
            "position_x": pos.get("x", 0.0),
            "position_y": pos.get("y", 0.0),
            "position_z": pos.get("z", 0.0),
            "orientation_x": qx,
            "orientation_y": qy,
            "orientation_z": qz,
            "orientation_w": qw,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }

    return None

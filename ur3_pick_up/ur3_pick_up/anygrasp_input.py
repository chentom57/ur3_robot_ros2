"""Read AnyGrasp/GraspNet outputs without importing the inference SDK or ROS."""

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def rigid_transform(value, name="transform"):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-4)):
        raise ValueError(f"{name} must contain a proper rotation and homogeneous last row")
    return matrix


@dataclass(frozen=True)
class Candidate:
    score: float
    width: float
    height: float
    depth: float
    rotation: np.ndarray
    translation: np.ndarray

    def tool_pose(self, base_from_cloud, tip_from_tool):
        """T_base_tool = T_base_cloud T_cloud_tip T_tip_tool.

        The tip reference is translation + depth * approach (GraspNet +X).
        Its orientation is the GraspNet orientation, not the robot tool axes.
        """
        cloud_from_tip = np.eye(4)
        cloud_from_tip[:3, :3] = self.rotation
        cloud_from_tip[:3, 3] = self.translation + self.depth * self.rotation[:, 0]
        result = (rigid_transform(base_from_cloud, "T_base_cloud")
                  @ cloud_from_tip @ rigid_transform(tip_from_tool, "T_tip_tool"))
        if np.any(np.abs(result[:3, 3]) > 2.0):
            raise ValueError("Converted tool target exceeds +/-2 meters; check units and TCP")
        return rigid_transform(result, "T_base_tool")


def candidate_from_fields(score, width, height, depth, rotation, translation):
    scalars = np.asarray([score, width, height, depth], dtype=float)
    translation = np.asarray(translation, dtype=float)
    rotation = np.asarray(rotation, dtype=float)
    if not np.isfinite(scalars).all() or translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError("Candidate fields must be finite; translation must have shape (3,)")
    if width <= 0 or width > 0.1 or height <= 0 or depth < 0 or depth > 0.2:
        raise ValueError("Invalid candidate dimensions in meters (width: (0,0.1], depth: [0,0.2])")
    transform = np.eye(4)
    if rotation.shape != (3, 3):
        raise ValueError("rotation_matrix must have shape (3,3)")
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    rigid_transform(transform, "candidate")
    return Candidate(float(score), float(width), float(height), float(depth), rotation, translation)


def load_candidates(filename):
    """Return candidates, optional frame_id and ROS stamp (sec, nanosec)."""
    path = Path(filename).expanduser()
    frame, stamp = None, None
    if path.suffix.lower() == ".npy":
        array = np.load(path, allow_pickle=False)
        if array.ndim != 2 or array.shape[1] != 17 or not (np.issubdtype(array.dtype, np.integer) or np.issubdtype(array.dtype, np.floating)):
            raise ValueError("Expected GraspGroup N x 17 numeric .npy array")
        candidates = [candidate_from_fields(*row[:4], row[4:13].reshape(3, 3), row[13:16])
                      for row in array]
    elif path.suffix.lower() == ".json":
        data = json.loads(path.read_text())
        if not isinstance(data, dict) or not isinstance(data.get("candidates"), list):
            raise ValueError("JSON requires an object containing a candidates list")
        if not all(isinstance(c, dict) for c in data["candidates"]):
            raise ValueError("Each JSON candidate must be an object")
        frame = data.get("frame_id")
        if not isinstance(frame, str) or not frame or frame.startswith("/"):
            raise ValueError("JSON requires a nonempty frame_id without leading slash")
        stamp_data = data.get("stamp")
        if stamp_data is not None:
            sec, nanosec = stamp_data["sec"], stamp_data["nanosec"]
            if (type(sec) is not int or type(nanosec) is not int or sec < 0
                    or not 0 <= nanosec < 1_000_000_000 or sec + nanosec == 0):
                raise ValueError("stamp must be a nonzero ROS time: sec >= 0, 0 <= nanosec < 1e9")
            stamp = (sec, nanosec)
        candidates = [candidate_from_fields(
            c["score"], c["width"], c["height"], c["depth"],
            c["rotation_matrix"], c["translation"]
        ) for c in data["candidates"]]
    else:
        raise ValueError("AnyGrasp input must be .npy or .json")
    if not candidates:
        raise ValueError("No AnyGrasp candidates; no motion will be planned")
    return candidates, frame, stamp


def load_tool_config(filename, tool_link, tcp_offset=0.174):
    data = json.loads(Path(filename).expanduser().read_text())
    if data["tool_link"] != tool_link:
        raise ValueError("Tool configuration tool_link does not match the planning link")
    if data.get("use_pose_tcp") is True:
        if "T_tip_tool" in data:
            raise ValueError("Use either use_pose_tcp or T_tip_tool, not both")
        if tool_link != "tool0":
            raise ValueError("use_pose_tcp requires tool_link=tool0")
        offset = float(tcp_offset)
        if not np.isfinite(offset) or not 0 <= offset <= 0.5:
            raise ValueError("tcp_offset must be finite and within [0, 0.5] meters")
        # Grasp axes to physical TCP axes: R_tool = R_grasp @ mapping.
        # tool0 origin is behind TCP along the rotated tool0 +Z axis.
        transform = np.eye(4)
        mapping = np.asarray(data["grasp_to_tcp_rotation"], dtype=float)
        if mapping.shape != (3, 3):
            raise ValueError("grasp_to_tcp_rotation must be 3x3")
        transform[:3, :3] = mapping
        rigid_transform(transform, "grasp_to_tcp_rotation")
        transform[:3, 3] = -mapping @ np.array([0.0, 0.0, offset])
    else:
        transform = rigid_transform(data["T_tip_tool"], "T_tip_tool")
    opening = float(data["max_opening_m"])
    if not np.isfinite(opening) or not 0 < opening <= 0.2:
        raise ValueError("max_opening_m must be the measured gripper opening in meters")
    return transform, opening


def quaternion_from_matrix(matrix):
    """Return normalized xyzw, including robust handling of 180-degree rotations."""
    rotation = rigid_transform(matrix)[:3, :3]
    trace = np.trace(rotation)
    if trace > 0:
        scale = np.sqrt(trace + 1.0) * 2
        q = np.array([(rotation[2, 1] - rotation[1, 2]) / scale,
                      (rotation[0, 2] - rotation[2, 0]) / scale,
                      (rotation[1, 0] - rotation[0, 1]) / scale, scale / 4])
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        scale = np.sqrt(1 + rotation[i, i] - rotation[j, j] - rotation[k, k]) * 2
        q = np.zeros(4)
        q[i] = scale / 4
        q[j] = (rotation[j, i] + rotation[i, j]) / scale
        q[k] = (rotation[k, i] + rotation[i, k]) / scale
        q[3] = (rotation[k, j] - rotation[j, k]) / scale
    return q / np.linalg.norm(q)


def matrix_from_transform(transform):
    q = transform.rotation
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(values).all() or not np.isclose(np.linalg.norm(values), 1.0, atol=1e-3):
        raise ValueError("TF contains an invalid quaternion")
    x, y, z, w = values / np.linalg.norm(values)
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    matrix[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
    return rigid_transform(matrix, "TF")


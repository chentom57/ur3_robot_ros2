"""Pure geometry and PointCloud2 decoding for a fixed eye-on-base camera."""
import re

import numpy as np
from .anygrasp_input import rigid_transform
from ur3_moveit_scene.workspace import Workspace



def validate_settings(s):
    Workspace(s['workspace_min'], s['workspace_max'])
    for key in ('cloud_timeout', 'cloud_max_age', 'voxel_size', 'max_camera_depth',
                'inference_timeout', 'gripper_height'):
        if not np.isfinite(s[key]) or s[key] <= 0:
            raise ValueError(f'{key} must be positive and finite')
    for key in ('max_points', 'min_roi_points'):
        if type(s[key]) is not int or s[key] < 1:
            raise ValueError(f'{key} must be a positive integer')
    if s['max_points'] < s['min_roi_points']:
        raise ValueError('max_points must be >= min_roi_points')
    for key in ('max_velocity_scaling_factor', 'max_acceleration_scaling_factor'):
        if not np.isfinite(s[key]) or not 0 < s[key] <= 1:
            raise ValueError(f'{key} must be in (0, 1]')
    if not np.isfinite(s['tcp_offset']) or not 0 <= s['tcp_offset'] <= .5:
        raise ValueError('tcp_offset must be in [0, 0.5] m')
    if s['gripper_backend'] not in {'tcp', 'none'} or not 1 <= s['gripper_port'] <= 65535:
        raise ValueError('Invalid gripper backend or port')
    if s['tool_link'] != 'tool0':
        raise ValueError('The configured 0.174 m hardware TCP is defined from tool0')
    if not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_.-]*', s['docker_container']):
        raise ValueError('Invalid Docker container name')
    for key in ('base_frame', 'pointcloud_topic', 'exchange_dir', 'grasp_tool_config'):
        if not s[key]:
            raise ValueError(f'{key} cannot be empty')
    for key in ('docker_sdk_dir', 'checkpoint_path'):
        if not s[key].startswith('/'):
            raise ValueError(f'{key} must be an absolute container path')


def decode_xyz(msg):
    """Decode XYZ respecting endianness, field offsets and organized row padding."""
    if msg.width < 1 or msg.height < 1 or msg.point_step < 1:
        raise ValueError('Empty PointCloud2')
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.row_step * msg.height:
        raise ValueError('Truncated PointCloud2 data or invalid row_step')
    fields = {f.name: f for f in msg.fields}
    columns = []
    for name in 'xyz':
        f = fields.get(name)
        if f is None or f.count != 1 or f.datatype not in (7, 8):
            raise ValueError('PointCloud2 requires scalar FLOAT32/FLOAT64 x/y/z fields')
        dtype = np.dtype(('>' if msg.is_bigendian else '<') + ('f4' if f.datatype == 7 else 'f8'))
        if f.offset < 0 or f.offset + dtype.itemsize > msg.point_step:
            raise ValueError('PointCloud2 field outside point_step')
        column = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                            offset=f.offset, strides=(msg.row_step, msg.point_step))
        columns.append(column.reshape(-1))
    return np.column_stack(columns).astype(np.float32)


def transform_points(points, transform):
    t = rigid_transform(transform)
    return np.asarray(points) @ t[:3, :3].T + t[:3, 3]


def prepare_cloud(points, base_from_camera, workspace, *, voxel_size=.005,
                  max_points=120000, max_depth=2., min_roi_points=50):
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('Expected N x 3 camera points')
    valid = np.isfinite(points).all(axis=1) & (points[:, 2] > 0) & (points[:, 2] <= max_depth)
    points = points[valid]
    if len(points):
        _, ids = np.unique(np.floor(points / voxel_size).astype(np.int64), axis=0, return_index=True)
        points = points[np.sort(ids)]
    # Cap the full scene (not just the ROI) to bound GPU memory. Deterministic
    # sampling preserves reproducibility; the ROI is rechecked after sampling.
    if len(points) > max_points:
        points = points[np.linspace(0, len(points)-1, max_points, dtype=int)]
    mask = workspace.contains(transform_points(points, base_from_camera))
    if np.count_nonzero(mask) < min_roi_points:
        raise ValueError(f'Only {np.count_nonzero(mask)} workspace points; require {min_roi_points}. '
                         'Check TF, camera view and workspace bounds. No inference requested.')
    return np.ascontiguousarray(points, dtype=np.float32), mask


def ranked_candidates(candidates, base_from_camera, workspace, max_opening):
    """Return valid pose indices/candidates in descending score order."""
    valid = []
    for i, c in enumerate(candidates):
        # Both the raw grasp origin and insertion-depth-adjusted TCP must be
        # inside. The compensated tool0 origin is allowed outside this box.
        endpoints = transform_points(np.array([c.translation,
            c.translation + c.depth * c.rotation[:, 0]]), base_from_camera)
        if c.width <= max_opening and workspace.contains(endpoints).all():
            valid.append((i, c))
    if not valid:
        raise ValueError('No candidate has a valid opening and grasp/TCP inside the workspace')
    return sorted(valid, key=lambda pair: pair[1].score, reverse=True)


def verify_fixed_camera(captured, latest, translation_tolerance=.003, angle_tolerance=np.deg2rad(.5)):
    a, b = rigid_transform(captured), rigid_transform(latest)
    shift = np.linalg.norm(a[:3, 3] - b[:3, 3])
    angle = np.arccos(np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2, -1, 1))
    if shift > translation_tolerance or angle > angle_tolerance:
        raise ValueError(f'Camera TF changed since capture ({shift:.4f} m, {np.rad2deg(angle):.3f} deg); '
                         'capture and plan again')

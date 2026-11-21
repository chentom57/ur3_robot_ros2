"""Shared workspace limits and RViz geometry; the volume is not an obstacle."""
from dataclasses import dataclass
from itertools import product
import numpy as np

DEFAULT_WORKSPACE_MIN = (-0.30, 0.0, 0.02)
DEFAULT_WORKSPACE_MAX = (0.30, 0.60, 0.60)


@dataclass(frozen=True)
class Workspace:
    lower: object
    upper: object

    def __post_init__(self):
        lo, hi = np.asarray(self.lower, dtype=float), np.asarray(self.upper, dtype=float)
        if (lo.shape != (3,) or hi.shape != (3,) or not np.isfinite([lo, hi]).all()
                or np.any(lo >= hi)):
            raise ValueError("workspace_min/max require three finite, strictly ordered XYZ bounds")
        object.__setattr__(self, 'lower', lo)
        object.__setattr__(self, 'upper', hi)

    def contains(self, points):
        points = np.asarray(points)
        return np.isfinite(points).all(axis=-1) & ((points >= self.lower) & (points <= self.upper)).all(axis=-1)


def workspace_markers(workspace, frame):
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray
    def marker(identifier, kind):
        m = Marker()
        m.header.frame_id = frame
        m.ns = 'anygrasp_workspace'
        m.id = identifier
        m.type, m.action = kind, Marker.ADD
        m.pose.orientation.w = 1.
        return m
    center, size = (workspace.lower + workspace.upper) / 2, workspace.upper - workspace.lower
    box = marker(100, Marker.CUBE)
    box.pose.position.x, box.pose.position.y, box.pose.position.z = map(float, center)
    box.scale.x, box.scale.y, box.scale.z = map(float, size)
    box.color.r, box.color.g, box.color.b, box.color.a = .1, .85, 1., .07
    edges = marker(101, Marker.LINE_LIST)
    edges.scale.x = .004
    edges.color.r, edges.color.g, edges.color.b, edges.color.a = .1, .85, 1., .9
    corners = list(product((0, 1), repeat=3))
    for a in corners:
        for axis in range(3):
            if a[axis] != 0:
                continue
            b = list(a); b[axis] = 1
            for c in (a, b):
                xyz = workspace.lower + size * c
                edges.points.append(Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])))
    label = marker(102, Marker.TEXT_VIEW_FACING)
    label.pose.position.x, label.pose.position.y = map(float, center[:2])
    label.pose.position.z = float(workspace.upper[2] + .04)
    label.scale.z = .025
    label.color.r = label.color.g = label.color.b = label.color.a = 1.
    lo, hi = workspace.lower, workspace.upper
    label.text = (f'AnyGrasp ROI ({frame}, m)\n'
                  f'X [{lo[0]:.2f}, {hi[0]:.2f}]  Y [{lo[1]:.2f}, {hi[1]:.2f}]\n'
                  f'Z [{lo[2]:.2f}, {hi[2]:.2f}]')
    return MarkerArray(markers=[box, edges, label])

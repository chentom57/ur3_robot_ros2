#!/usr/bin/env python3
"""Display the workspace during bringup, yielding to an active task's markers."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import MarkerArray

from .workspace import Workspace, DEFAULT_WORKSPACE_MIN, DEFAULT_WORKSPACE_MAX, workspace_markers

WORKSPACE_TOPIC = '/ur3_pick_up/workspace'
TARGET_TOPIC = '/ur3_pick_up/target_and_path_markers'


class WorkspacePreview(Node):
    def __init__(self):
        super().__init__('ur3_workspace_preview')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('workspace_min', list(DEFAULT_WORKSPACE_MIN))
        self.declare_parameter('workspace_max', list(DEFAULT_WORKSPACE_MAX))
        self.workspace = Workspace(self.get_parameter('workspace_min').value,
                                   self.get_parameter('workspace_max').value)
        self.frame = self.get_parameter('base_frame').value
        self.marker_publishers = []
        self.refresh()
        self.timer = self.create_timer(1., self.refresh)

    def other_publisher_present(self):
        # Release our writers, including their transient-local samples, when
        # the task owns the display. This prevents default bounds from replacing
        # a task's explicitly configured bounds, including for late subscribers.
        return any(info.node_name != self.get_name() or info.node_namespace != self.get_namespace()
                   for topic in (WORKSPACE_TOPIC, TARGET_TOPIC)
                   for info in self.get_publishers_info_by_topic(topic))

    def refresh(self):
        if self.other_publisher_present():
            for publisher in self.marker_publishers:
                self.destroy_publisher(publisher)
            self.marker_publishers = []
            return
        if not self.marker_publishers:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.marker_publishers = [self.create_publisher(MarkerArray, topic, qos)
                                      for topic in (WORKSPACE_TOPIC, TARGET_TOPIC)]
        message = workspace_markers(self.workspace, self.frame)
        for publisher in self.marker_publishers:
            publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WorkspacePreview()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

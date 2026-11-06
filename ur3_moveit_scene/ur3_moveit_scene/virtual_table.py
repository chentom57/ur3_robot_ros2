#!/usr/bin/env python3
"""Publish a large shared horizontal table slab to the MoveIt scene."""

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker


class VirtualTablePublisher(Node):
    def __init__(self):
        super().__init__("ur3_virtual_table")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("table_z", -0.01)
        self.declare_parameter("table_extent", 4.0)
        self.declare_parameter("table_thickness", 0.02)
        self.declare_parameter("scene_publish_retries", 5)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.table_z = float(self.get_parameter("table_z").value)
        self.table_extent = float(self.get_parameter("table_extent").value)
        self.table_thickness = float(self.get_parameter("table_thickness").value)
        self.scene_publish_retries = int(
            self.get_parameter("scene_publish_retries").value
        )
        if self.table_extent <= 0.0:
            raise ValueError("table_extent must be greater than zero")
        if self.table_thickness <= 0.0:
            raise ValueError("table_thickness must be greater than zero")
        if self.scene_publish_retries < 1:
            raise ValueError("scene_publish_retries must be at least one")

        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.marker_pub = self.create_publisher(
            Marker, "/moveit_virtual_table/marker", 10
        )

        self.scene_diff = self._make_scene_diff()
        self.marker = self._make_marker()
        self.scene_applied = False
        self.scene_future = None
        self.remaining_scene_publishes = 0
        self.last_scene_subscriber_count = 0
        self.timer = self.create_timer(0.5, self._publish_updates)
        self.get_logger().info(
            f"Publishing shared table slab in {self.base_frame}: "
            f"{self.table_extent:.1f} m square, top at z={self.table_z:.3f} m, "
            f"thickness={self.table_thickness:.3f} m."
        )

    def _make_scene_diff(self):
        slab = SolidPrimitive()
        slab.type = SolidPrimitive.BOX
        slab.dimensions = [
            self.table_extent,
            self.table_extent,
            self.table_thickness,
        ]

        collision = CollisionObject()
        collision.header.frame_id = self.base_frame
        collision.id = "ur3_virtual_table_collision"
        collision.pose.orientation.w = 1.0
        collision.primitives = [slab]
        slab_pose = Pose()
        slab_pose.position.z = self.table_z - self.table_thickness / 2.0
        slab_pose.orientation.w = 1.0
        collision.primitive_poses = [slab_pose]
        collision.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision)
        return scene

    def _make_marker(self):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.ns = "virtual_table"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.z = self.table_z - self.table_thickness / 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.table_extent
        marker.scale.y = self.table_extent
        marker.scale.z = self.table_thickness
        marker.color.r = 0.1
        marker.color.g = 0.55
        marker.color.b = 0.95
        marker.color.a = 0.30
        marker.lifetime.sec = 0
        return marker

    def _publish_updates(self):
        # Apply the object through move_group's synchronous planning-scene API
        # first. This makes the shared scene authoritative before we announce
        # the same diff to any other PlanningSceneMonitors.
        if not self.scene_applied and self.scene_future is None:
            if self.apply_scene_client.service_is_ready():
                request = ApplyPlanningScene.Request()
                request.scene = self.scene_diff
                self.scene_future = self.apply_scene_client.call_async(request)
                self.scene_future.add_done_callback(self._on_scene_applied)

        subscriber_count = self.scene_pub.get_subscription_count()
        if subscriber_count != self.last_scene_subscriber_count:
            self.last_scene_subscriber_count = subscriber_count
            if self.scene_applied and subscriber_count > 0:
                # Late planning clients need the table after they subscribe.
                self.remaining_scene_publishes = self.scene_publish_retries

        # Repeat the diff briefly for each newly connected scene monitor.
        if self.scene_applied and self.remaining_scene_publishes > 0:
            if subscriber_count > 0:
                self.scene_pub.publish(self.scene_diff)
                self.remaining_scene_publishes -= 1
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self.marker)

    def _on_scene_applied(self, future):
        try:
            response = future.result()
        except Exception as exc:  # Keep retrying if the service call failed.
            self.get_logger().error(f"Planning scene apply call failed: {exc}")
            self.scene_future = None
            return

        self.scene_future = None
        if not response.success:
            self.get_logger().error(
                "MoveIt rejected the virtual table collision slab; will retry."
            )
            return

        self.scene_applied = True
        self.remaining_scene_publishes = self.scene_publish_retries
        self.get_logger().info(
            f"MoveIt accepted collision object 'ur3_virtual_table_collision'; "
            f"its top face is at {self.base_frame} z={self.table_z:.3f} m."
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VirtualTablePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

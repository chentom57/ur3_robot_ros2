
#!/usr/bin/env python3
"""Visualize the MoveIt handover object in RViz. No robot commands."""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from visualization_msgs.msg import Marker


OBJECT_ID = "handover_demo_object"


def multiply(a, b):
    return (
        a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
        a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
        a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
        a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2],
    )


def quaternion(p):
    q = p.orientation
    return (q.x, q.y, q.z, q.w)


def compose(a, b):
    """Compose CollisionObject pose and primitive-local pose."""
    import math
    from geometry_msgs.msg import Pose

    q = quaternion(a)
    v = (b.position.x, b.position.y, b.position.z)
    u = q[:3]

    uv = (
        u[1]*v[2] - u[2]*v[1],
        u[2]*v[0] - u[0]*v[2],
        u[0]*v[1] - u[1]*v[0],
    )
    uuv = (
        u[1]*uv[2] - u[2]*uv[1],
        u[2]*uv[0] - u[0]*uv[2],
        u[0]*uv[1] - u[1]*uv[0],
    )

    result = Pose()
    result.position.x = a.position.x + v[0] + 2*(q[3]*uv[0] + uuv[0])
    result.position.y = a.position.y + v[1] + 2*(q[3]*uv[1] + uuv[1])
    result.position.z = a.position.z + v[2] + 2*(q[3]*uv[2] + uuv[2])

    x, y, z, w = multiply(q, quaternion(b))
    result.orientation.x = x
    result.orientation.y = y
    result.orientation.z = z
    result.orientation.w = w
    return result


class ObjectDisplay(Node):
    def __init__(self):
        super().__init__("handover_object_display")

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(
            Marker, "/ur3_handover/object_marker", qos
        )
        self.client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self.last_marker = None
        self.last_mode = None

    def refresh(self):
        if not self.client.service_is_ready():
            return

        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=2.0
        )

        if not future.done() or future.result() is None:
            return

        scene = future.result().scene
        obj = None
        mode = None

        for attached in scene.robot_state.attached_collision_objects:
            if attached.object.id == OBJECT_ID:
                obj = attached.object
                mode = "ATTACHED"
                break

        if obj is None:
            for world_obj in scene.world.collision_objects:
                if world_obj.id == OBJECT_ID:
                    obj = world_obj
                    mode = "RELEASED"
                    break

        # Keep the previous marker during brief scene-sync gaps.
        if obj is None or not obj.primitives:
            return

        primitive = obj.primitives[0]
        if primitive.type != 1 or len(primitive.dimensions) < 3:
            self.get_logger().warn("Expected a box collision object")
            return

        marker = Marker()
        marker.header.frame_id = obj.header.frame_id
        if not marker.header.frame_id:
            marker.header.frame_id = (
                "tool0" if mode == "ATTACHED" else "world"
            )

        marker.ns = "handover_object"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = compose(
            obj.pose, obj.primitive_poses[0]
        )
        marker.scale.x = primitive.dimensions[0]
        marker.scale.y = primitive.dimensions[1]
        marker.scale.z = primitive.dimensions[2]

        marker.color.r = 0.65
        marker.color.g = 0.25
        marker.color.b = 0.90
        marker.color.a = 1.0

        self.last_marker = marker

        if mode != self.last_mode:
            self.get_logger().info(
                f"Object mode: {mode}, frame: {marker.header.frame_id}"
            )
            self.last_mode = mode

    def publish_marker(self):
        if self.last_marker is not None:
            self.last_marker.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.last_marker)


def main():
    rclpy.init()
    node = ObjectDisplay()

    try:
        while rclpy.ok():
            node.refresh()
            for _ in range(5):
                node.publish_marker()
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

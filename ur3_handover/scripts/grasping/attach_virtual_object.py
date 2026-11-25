
#!/usr/bin/env python3
"""Attach a demonstration object to UR3 tool0 in MoveIt."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    CollisionObject,
    AttachedCollisionObject,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene


def main():
    rclpy.init()
    node = Node("attach_virtual_handover_object")

    try:
        client = node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

        if not client.wait_for_service(timeout_sec=10.0):
            print("ERROR: Planning scene service unavailable.")
            return

        # Remove the old stationary object from the world.
        remove = CollisionObject()
        remove.header.frame_id = "base_link"
        remove.id = "handover_demo_object"
        remove.operation = CollisionObject.REMOVE

        # Represent the same object as attached to tool0.
        obj = CollisionObject()
        obj.header.frame_id = "tool0"
        obj.id = "handover_demo_object"
        obj.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.12]

        pose = Pose()
        pose.position.z = 0.12
        pose.orientation.w = 1.0

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)

        attached = AttachedCollisionObject()
        attached.link_name = "tool0"
        attached.object = obj
        attached.touch_links = ["tool0"]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects.append(remove)
        scene.robot_state.attached_collision_objects.append(
            attached
        )

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=10.0
        )

        if not future.done() or future.result() is None:
            print("ERROR: Request timed out.")
        elif future.result().success:
            print("SUCCESS: Virtual object attached to tool0.")
        else:
            print("ERROR: MoveIt rejected the attachment.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

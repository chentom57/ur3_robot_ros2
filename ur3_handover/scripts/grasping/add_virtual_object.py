
#!/usr/bin/env python3
"""Add one virtual box to the MoveIt planning scene."""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene


def main():
    rclpy.init()
    node = Node("add_virtual_handover_object")

    try:
        client = node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

        if not client.wait_for_service(timeout_sec=10.0):
            print("ERROR: MoveIt planning scene service unavailable.")
            return

        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = "handover_demo_object"
        obj.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.12]

        pose = Pose()
        pose.position.x = -0.25
        pose.position.y = 0.35
        pose.position.z = 0.20
        pose.orientation.w = 1.0

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects.append(obj)

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=10.0
        )

        if future.done() and future.result() is not None:
            if future.result().success:
                print("SUCCESS: Virtual object added to MoveIt.")
            else:
                print("ERROR: MoveIt rejected the object.")
        else:
            print("ERROR: Planning scene request timed out.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

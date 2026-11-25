
#!/usr/bin/env python3
"""Read the virtual object's status from MoveIt."""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents

OBJECT_ID = "handover_demo_object"


def main():
    rclpy.init()
    node = Node("check_handover_object_state")

    try:
        client = node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )

        if not client.wait_for_service(timeout_sec=10.0):
            print("ERROR: Planning scene service unavailable.")
            return

        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )

        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=10.0
        )

        if not future.done() or future.result() is None:
            print("ERROR: Request timed out.")
            return

        scene = future.result().scene

        world_objects = [
            obj for obj in scene.world.collision_objects
            if obj.id == OBJECT_ID
        ]

        attached_objects = [
            obj for obj in scene.robot_state.attached_collision_objects
            if obj.object.id == OBJECT_ID
        ]

        print("===== OBJECT STATUS =====")
        print("World object count:", len(world_objects))
        print("Attached object count:", len(attached_objects))

        for obj in world_objects:
            print("World frame:", obj.header.frame_id)

        for obj in attached_objects:
            print("Attached link:", obj.link_name)
            print("Object frame:", obj.object.header.frame_id)
            print("Touch links:", list(obj.touch_links))

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

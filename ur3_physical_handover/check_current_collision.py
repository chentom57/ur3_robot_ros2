#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningSceneComponents


def call(node, service_type, name, request):
    client = node.create_client(service_type, name)

    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError(f"Service unavailable: {name}")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)

    if not future.done() or future.result() is None:
        raise RuntimeError(f"Service timed out: {name}")

    return future.result()


def main():
    rclpy.init()
    node = Node("ur3_physical_collision_check")

    try:
        scene_request = GetPlanningScene.Request()
        scene_request.components.components = (
            PlanningSceneComponents.ROBOT_STATE |
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )

        scene = call(
            node,
            GetPlanningScene,
            "/get_planning_scene",
            scene_request
        ).scene

        objects = [
            obj.id for obj in scene.world.collision_objects
        ]

        print("World objects:", objects)

        if "ur3_virtual_table_collision" not in objects:
            raise RuntimeError("Virtual table is missing")

        state_request = GetStateValidity.Request()
        state_request.robot_state = scene.robot_state
        state_request.group_name = "ur_manipulator"

        result = call(
            node,
            GetStateValidity,
            "/check_state_validity",
            state_request
        )

        print("Current robot state valid:", result.valid)

        if not result.valid:
            print("Collision contacts:")
            for contact in result.contacts[:10]:
                print(
                    contact.contact_body_1,
                    "<->",
                    contact.contact_body_2
                )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

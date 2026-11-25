#!/usr/bin/env python3
"""Plan return from released-object handover state. MOCK ONLY."""

import time
import rclpy
from moveit.core.robot_state import RobotState

from ur3_mock_handover_cycle import (
    BaseCoordinatePick,
    confirm_mock,
    current_scene,
    check_current_state,
    OBJECT_ID,
)

START_JOINTS = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]


def main():
    confirm_mock()
    rclpy.init()
    app = None

    try:
        app = BaseCoordinatePick()

        if not app.scene_ready:
            raise RuntimeError("Virtual table not available")

        # Wait for the authoritative MoveIt scene to synchronize.
        release_verified = False

        for attempt in range(20):
            scene = current_scene(app.node)

            world_ids = [
                obj.id for obj in scene.world.collision_objects
            ]
            attached_ids = [
                obj.object.id
                for obj in scene.robot_state.attached_collision_objects
            ]

            print(
                f"Scene check {attempt + 1}/20: "
                f"world={world_ids}, attached={attached_ids}",
                flush=True,
            )

            if (
                OBJECT_ID in world_ids
                and OBJECT_ID not in attached_ids
                and "ur3_virtual_table_collision" in world_ids
            ):
                release_verified = True
                break

            time.sleep(0.5)

        if not release_verified:
            raise RuntimeError(
                "Release scene did not synchronize; "
                "refusing return planning"
            )

        check_current_state(app.node)

        saved = RobotState(app.robot.get_robot_model())
        saved.set_to_default_values()
        saved.set_joint_group_positions(app.group, START_JOINTS)
        saved.update()

        app.planning.set_start_state_to_current_state()
        app.planning.set_goal_state(robot_state=saved)

        print("Planning return to recorded starting joints...")
        plan = app.planning.plan()

        if plan is None:
            print("FAILED: No valid return path found.")
            return

        print("SUCCESS: Mock return trajectory planned.", flush=True)

        answer = input(
            "Execute return on verified MOCK UR3? Type MOCK_RETURN: "
        ).strip()

        if answer != "MOCK_RETURN":
            print("Cancelled. No motion executed.")
            return

        confirm_mock()
        check_current_state(app.node)

        print("Executing MOCK return...", flush=True)

        if not app.execute_plan(plan, "Mock return to start"):
            raise RuntimeError("Mock return execution failed")

        app.planning.set_start_state_to_current_state()
        actual = app.planning.get_start_state().get_joint_group_positions(
            app.group
        )

        import numpy as np
        error = np.max(
            np.abs(np.asarray(actual) - np.asarray(START_JOINTS))
        )

        print("Final group joints:", actual, flush=True)
        print("Maximum joint error (rad):", error, flush=True)

        if error > 0.03:
            raise RuntimeError(
                "Return executed but final joints differ from start"
            )

        print(
            "SUCCESS: Mock UR3 returned to starting joints.",
            flush=True
        )

    finally:
        if app is not None:
            app.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""MOCK ONLY: resume UR3 at handover pose, release attached virtual item, return to saved original joints.

Does NOT move a real robot or actuate Robotiq fingers.
Requires the existing ur3_mock_handover_cycle.py in the SAME directory.
"""
import time
import numpy as np
import rclpy
from moveit.core.robot_state import RobotState
from ur3_mock_handover_cycle import (
    BaseCoordinatePick,
    OBJECT_ID,
    attached_object,
    check_current_state,
    confirm_mock,
    current_scene,
    simulate_release,
)

# Recorded before the successful carry (MoveIt ur_manipulator group order).
ORIGINAL_GROUP_JOINTS = np.array([
    0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0
], dtype=float)


def wait_for_attached(node, timeout_s=10.0):
    deadline = time.monotonic() + timeout_s
    while True:
        scene = current_scene(node)
        attached = attached_object(scene)
        if attached is not None:
            return attached
        if time.monotonic() >= deadline:
            raise RuntimeError(f"ABORT: {OBJECT_ID} not attached in /get_planning_scene")
        time.sleep(0.5)


def wait_for_released(node, timeout_s=10.0):
    deadline = time.monotonic() + timeout_s
    while True:
        scene = current_scene(node)
        attached = attached_object(scene) is not None
        world = OBJECT_ID in [obj.id for obj in scene.world.collision_objects]
        if not attached and world:
            return
        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"ABORT: release not established (attached={attached}, world={world})"
            )
        time.sleep(0.5)


def main():
    confirm_mock()
    rclpy.init()
    app = None
    try:
        app = BaseCoordinatePick()
        if not app.scene_ready:
            raise RuntimeError("ABORT: virtual table collision object missing")
        wait_for_attached(app.node)
        check_current_state(app.node)

        app.planning.set_start_state_to_current_state()
        current_joints = np.asarray(
            app.planning.get_start_state().get_joint_group_positions(app.group),
            dtype=float,
        )
        print("Current group joints:", current_joints, flush=True)
        print("Recorded original joints:", ORIGINAL_GROUP_JOINTS, flush=True)
        if current_joints.shape != ORIGINAL_GROUP_JOINTS.shape:
            raise RuntimeError("ABORT: unexpected planning-group joint count")

        if input("Simulated human ready to receive object? Type RELEASE: ").strip() != "RELEASE":
            print("Cancelled. Object remains attached; no motion.")
            return
        confirm_mock()
        wait_for_attached(app.node)
        simulate_release(app)
        wait_for_released(app.node)
        print("Confirmed: object is in world, no longer attached.", flush=True)

        # Re-plan with released object represented as a collision obstacle.
        check_current_state(app.node)
        saved = RobotState(app.robot.get_robot_model())
        saved.set_to_default_values()
        saved.set_joint_group_positions(app.group, ORIGINAL_GROUP_JOINTS)
        saved.update()
        app.planning.set_start_state_to_current_state()
        app.planning.set_goal_state(robot_state=saved)
        print("Planning return from current pose to ORIGINAL joints...", flush=True)
        plan = app.planning.plan()
        if plan is None:
            raise RuntimeError("ABORT: return planning failed; no movement executed")
        print("Return plan generated. Check robot and object clearance in RViz.", flush=True)
        if input("Execute MOCK-only return? Type RETURN: ").strip() != "RETURN":
            print("Return cancelled; object remains released in world")
            return
        confirm_mock()
        wait_for_released(app.node)
        check_current_state(app.node)
        if not app.execute_plan(plan, "Mock return to original joints"):
            raise RuntimeError("ABORT: return controller execution failed")
        print("SIMULATED GRIPPER CLOSED (state indication only; no moving fingers)", flush=True)
        print("SUCCESS: mock object release and return executed", flush=True)
    finally:
        if app is not None:
            app.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

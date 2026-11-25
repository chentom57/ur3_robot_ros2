#!/usr/bin/env python3
"""UR3 mock-only carry -> simulated handover -> return to original joint state.

Requires an existing attached handover_demo_object. Does NOT simulate gripper fingers
or actual pickup. Will never connect to the Robotiq TCP controller.
"""
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from moveit.core.robot_state import RobotState
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject,
                             PlanningScene, PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity

PROJECT = Path.home() / "ur3_robot_ros2"
sys.path.insert(0, str(PROJECT / "ur3_base_pick"))
from ur3_base_pick.base_coordinate_pick import BaseCoordinatePick

OBJECT_ID = "handover_demo_object"
TARGET = (-0.25, 0.35, 0.20)


def confirm_mock():
    check = subprocess.run(["ros2", "control", "list_hardware_components"],
                           capture_output=True, text=True, timeout=15, check=True)
    if ("name: ur3" not in check.stdout or
            "plugin name: mock_components/GenericSystem" not in check.stdout):
        raise RuntimeError("ABORT: The active UR3 is not verified mock hardware")
    svc = subprocess.run(["ros2", "service", "list"], capture_output=True,
                         text=True, timeout=15, check=True)
    managers = [s for s in svc.stdout.splitlines()
                if s.endswith("/list_hardware_components")]
    if managers != ["/controller_manager/list_hardware_components"]:
        raise RuntimeError("ABORT: Unexpected controller-manager configuration")


def call(node, service_type, name, request):
    client = node.create_client(service_type, name)
    if not client.wait_for_service(timeout_sec=10):
        raise RuntimeError("Service unavailable: " + name)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    if not future.done() or future.result() is None:
        raise RuntimeError("Service failed or timed out: " + name)
    return future.result()


def current_scene(node):
    req = GetPlanningScene.Request()
    req.components.components = (
        PlanningSceneComponents.ROBOT_STATE |
        PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    )
    return call(node, GetPlanningScene, "/get_planning_scene", req).scene


def attached_object(scene):
    return next((a for a in scene.robot_state.attached_collision_objects
                 if a.object.id == OBJECT_ID), None)


def check_current_state(node):
    scene = current_scene(node)
    req = GetStateValidity.Request()
    req.robot_state = scene.robot_state
    req.group_name = "ur_manipulator"
    result = call(node, GetStateValidity, "/check_state_validity", req)
    if not result.valid:
        print("COLLISION/INVALID STATE detected:")
        for contact in result.contacts[:10]:
            print("  ", contact.contact_body_1, "<->", contact.contact_body_2)
        raise RuntimeError("ABORT: current robot state is invalid")
    print("Current scene state: valid")


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def rotate_vector(q, v):
    x, y, z, w = q
    t = np.cross([x, y, z], v) * 2
    return np.asarray(v) + w * t + np.cross([x, y, z], t)


def world_pose(app, obj):
    t = app.tf_buffer.lookup_transform("base_link", "tool0", Time(),
                                      timeout=Duration(seconds=3)).transform
    q = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    oq = (obj.pose.orientation.x, obj.pose.orientation.y,
          obj.pose.orientation.z, obj.pose.orientation.w)
    p = rotate_vector(q, [obj.pose.position.x, obj.pose.position.y,
                          obj.pose.position.z]) + np.array([
                              t.translation.x, t.translation.y, t.translation.z])
    qout = quaternion_multiply(q, oq)
    return p, qout


def simulate_release(app):
    scene_before = None
    attached = None

    for attempt in range(20):
        scene_before = current_scene(app.node)
        attached = attached_object(scene_before)

        if attached is not None:
            print("Attached object confirmed before release.", flush=True)
            break

        print(
            f"Waiting for attachment before release: {attempt + 1}/20",
            flush=True
        )
        time.sleep(0.5)

    if attached is None:
        raise RuntimeError(
            "ABORT: object not found in planning scene before release"
        )
    p, q = world_pose(app, attached.object)
    world = CollisionObject()
    world.header.frame_id = "base_link"
    world.id = OBJECT_ID
    world.operation = CollisionObject.ADD
    world.pose.position.x, world.pose.position.y, world.pose.position.z = map(float, p)
    (world.pose.orientation.x, world.pose.orientation.y,
     world.pose.orientation.z, world.pose.orientation.w) = map(float, q)
    world.primitives = list(attached.object.primitives)
    world.primitive_poses = list(attached.object.primitive_poses)

    removal = AttachedCollisionObject()
    removal.link_name = attached.link_name
    removal.object.id = OBJECT_ID
    removal.object.operation = CollisionObject.REMOVE
    delta = PlanningScene()
    delta.is_diff = True
    delta.robot_state.is_diff = True
    delta.robot_state.attached_collision_objects.append(removal)
    delta.world.collision_objects.append(world)
    req = ApplyPlanningScene.Request()
    req.scene = delta
    reply = call(app.node, ApplyPlanningScene, "/apply_planning_scene", req)
    # Wait for the planning scene to reflect the release.
    released = False
    for attempt in range(30):
        updated = current_scene(app.node)
        world_ids = {
            obj.id for obj in updated.world.collision_objects
        }
        attached_ids = {
            obj.object.id
            for obj in updated.robot_state.attached_collision_objects
        }

        released = (
            OBJECT_ID in world_ids
            and OBJECT_ID not in attached_ids
            and "ur3_virtual_table_collision" in world_ids
        )

        print(
            f"Release check {attempt + 1}/30: "
            f"world={sorted(world_ids)}, "
            f"attached={sorted(attached_ids)}",
            flush=True,
        )

        if released:
            break

        time.sleep(0.5)

    if not released:
        raise RuntimeError(
            "ABORT: Release was not verified within timeout"
        )

    print(
        "SUCCESS: Virtual object released into world scene.",
        flush=True,
    )

    if not reply.success:
        print(
            "NOTE: ApplyPlanningScene reported failure, "
            "but independent scene verification succeeded.",
            flush=True,
        )


def main():
    confirm_mock()
    rclpy.init()
    app = None
    try:
        app = BaseCoordinatePick()
        if not app.scene_ready:
            raise RuntimeError("ABORT: virtual table collision model missing")
        print("Waiting for attached object in MoveIt planning scene...", flush=True)

        for attempt in range(20):
            scene = current_scene(app.node)

            attached_ids = [
                a.object.id
                for a in scene.robot_state.attached_collision_objects
            ]

            world_ids = [
                o.id
                for o in scene.world.collision_objects
            ]

            print(
                f"Scene check {attempt + 1}/20: "
                f"attached={attached_ids}, world={world_ids}",
                flush=True
            )

            if attached_object(scene) is not None:
                print("Attached object confirmed.", flush=True)
                break

            time.sleep(0.5)
        else:
            raise RuntimeError(
                "ABORT: handover_demo_object was not found after scene synchronization"
            )
        if any(o.id == OBJECT_ID for o in scene.world.collision_objects):
            raise RuntimeError("ABORT: duplicate object in world and robot")
        check_current_state(app.node)

        app.planning.set_start_state_to_current_state()
        initial_joints = np.array(
            app.planning.get_start_state().get_joint_group_positions(app.group),
            dtype=float).copy()
        print("Saved original joints:", initial_joints)
        print("SIMULATED GRIPPER CLOSED: object already attached to tool0")
        
        from rclpy.time import Time
        from tf2_ros import TransformException

        print("Waiting for base_link -> tool0 TF...", flush=True)

        # Dedicated TF listener with its own executor thread.
        # Keep these references alive for the entire handover cycle.
        from tf2_ros import Buffer, TransformListener

        if not rclpy.ok():
            rclpy.init()

        handover_tf_node = rclpy.create_node(
            "ur3_handover_tf_listener"
        )
        handover_tf_buffer = Buffer()
        handover_tf_listener = TransformListener(
            handover_tf_buffer,
            handover_tf_node,
            spin_thread=True
        )

        # Also used by world_pose() when the object is released.
        app.tf_buffer = handover_tf_buffer

        tf_ready = False
        for attempt in range(40):
            try:
                app.tf_buffer.lookup_transform(
                    "base_link", "tool0", Time()
                )
                tf_ready = True
                print("TF connected successfully.", flush=True)
                break
            except TransformException as exc:
                print(f"TF attempt {attempt + 1}/40: {exc}", flush=True)
                time.sleep(0.25)

        if not tf_ready:
            raise RuntimeError(
                "ABORT: base_link -> tool0 TF unavailable"
            )

        print("Planning carry with attached object...", flush=True)
        outbound, _ = app.plan_pick_and_return(TARGET)
        if outbound is None:
            raise RuntimeError("ABORT: no carrying trajectory")
        print("Inspect the planned trajectory and clearance in RViz.")
        if input("Execute MOCK carry + simulated release + return? Type MOCK: ").strip() != "MOCK":
            print("Cancelled; no motion")
            return
        confirm_mock()
        check_current_state(app.node)
        if not app.execute_plan(outbound, "Mock carrying"):
            raise RuntimeError("Carry execution failed; keeping object attached")
        print("At handover zone; waiting 3 seconds...")
        time.sleep(3)

        answer = input(
            "Simulated receiver confirms secure support? Type RELEASE: "
        ).strip()

        if answer != "RELEASE":
            print("Receiver not confirmed. Object remains attached.")
            return

        confirm_mock()
        check_current_state(app.node)
        simulate_release(app)

        # Plan return *after* detaching, so the released object is an obstacle.
        check_current_state(app.node)
        saved = RobotState(app.robot.get_robot_model())
        saved.set_to_default_values()
        saved.set_joint_group_positions(app.group, initial_joints)
        saved.update()
        app.planning.set_start_state_to_current_state()
        app.planning.set_goal_state(robot_state=saved)
        return_plan = app.planning.plan()
        if return_plan is None:
            raise RuntimeError("ABORT: collision-free return to original pose unavailable")
        print("Return plan available; released object kept in planning scene")
        if input("Execute MOCK return to ORIGINAL joints? Type RETURN: ").strip() != "RETURN":
            print("Return cancelled; gripper remains simulated OPEN")
            return
        confirm_mock()
        if not app.execute_plan(return_plan, "Mock return"):
            raise RuntimeError("Return execution failed")
        print("SIMULATED GRIPPER CLOSED (no physical finger animation)")
        print("SUCCESS: Mock carry -> release -> original joint position completed")
    finally:
        if app is not None:
            app.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()

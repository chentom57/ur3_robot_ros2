#!/usr/bin/env python3
"""Add, inspect, or remove a measured box collision object in MoveIt's planning scene.

ROS 2 Jazzy / MoveIt 2. No motion/execution commands are ever issued.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


def positive(v):
    x = float(v)
    if not 0.0 < x <= 5.0:
        raise argparse.ArgumentTypeError('dimensions must be > 0 and <= 5 meters')
    return x


def request(node, client, req, timeout=10.):
    future = client.call_async(req)
    end = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        raise TimeoutError('Planning-scene service timed out')
    return future.result()


def objects(node, client):
    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    res = request(node, client, req)
    if res is None:
        raise RuntimeError('No response from /get_planning_scene')
    return res.scene.world.collision_objects


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action', required=True)
    add = sub.add_parser('add', help='Add/replace a measured box')
    for name in ('x', 'y', 'z'):
        add.add_argument('--' + name, required=True, type=float, help='Box center in base_link, meters')
    for name in ('sx', 'sy', 'sz'):
        add.add_argument('--' + name, required=True, type=positive, help='Box dimension, meters')
    add.add_argument('--id', default='ur3_test_obstacle')
    remove = sub.add_parser('remove', help='Remove a known obstacle by ID')
    remove.add_argument('--id', default='ur3_test_obstacle')
    sub.add_parser('list', help='List obstacles in the current MoveIt planning scene')
    args = p.parse_args()
    rclpy.init()
    node = Node('ur3_obstacle_scene_editor')
    try:
        get = node.create_client(GetPlanningScene, '/get_planning_scene')
        if not get.wait_for_service(timeout_sec=10):
            raise RuntimeError('/get_planning_scene not available (is move_group running?)')
        if args.action == 'list':
            found = objects(node, get)
            if found:
                for item in found:
                    print(f'{item.id}: frame={item.header.frame_id}, shapes={len(item.primitives)}')
            else:
                print('No world collision objects found')
            return
        apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not apply.wait_for_service(timeout_sec=10):
            raise RuntimeError('/apply_planning_scene not available')
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = args.id
        if args.action == 'add':
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [args.sx, args.sy, args.sz]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = args.x, args.y, args.z
            pose.orientation.w = 1.0
            obj.primitives = [primitive]
            obj.primitive_poses = [pose]
            obj.operation = CollisionObject.ADD
            print(f'Adding {obj.id}: center=({args.x:.3f},{args.y:.3f},{args.z:.3f}) m, size=({args.sx:.3f},{args.sy:.3f},{args.sz:.3f}) m in base_link')
        else:
            obj.operation = CollisionObject.REMOVE
            print(f'Removing {obj.id}')
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [obj]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        res = request(node, apply, req)
        if res is None or not res.success:
            raise RuntimeError('MoveIt rejected the planning-scene update')
        ids = [item.id for item in objects(node, get)]
        present = args.id in ids
        if present != (args.action == 'add'):
            raise RuntimeError('Service succeeded but obstacle presence was not verified')
        print(f'VERIFIED: {args.id} {"present in" if present else "removed from"} MoveIt planning scene')
        print('NO ROBOT MOVEMENT REQUESTS WERE SENT.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'ERROR: {e}', file=sys.stderr)
        sys.exit(1)

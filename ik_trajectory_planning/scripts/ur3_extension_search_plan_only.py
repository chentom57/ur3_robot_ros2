#!/usr/bin/env python3
"""UR3 MoveIt 2: find collision-aware reachable outward TCP targets.

PLAN ONLY. Never sends ExecuteTrajectory, FollowJointTrajectory, or plan_only=False.
Candidates are horizontal offsets radially outward from base_link, rotated -35/0/+35
in the XY plane. This is a reach-search heuristic, NOT a human-handover planner.
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (RobotState, Constraints, JointConstraint,
                             MotionPlanRequest, PlanningOptions)
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.action import MoveGroup
from tf2_ros import Buffer, TransformListener

JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


def wait_future(future, seconds):
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    if not event.wait(seconds):
        raise TimeoutError('ROS request timed out')
    return future.result()


def joint_map(msg):
    return dict(zip(msg.name, msg.position))


def max_joint_change(a, b):
    return max(abs(math.degrees(math.remainder(b[n] - a[n], 2*math.pi))) for n in JOINTS)


class Search(Node):
    def __init__(self):
        super().__init__('ur3_outward_reach_search_plan_only')
        self.joints = None
        self.last_seen = 0.0
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.move_client = ActionClient(self, MoveGroup, '/move_action')

    def on_joints(self, msg):
        if all(n in msg.name for n in JOINTS):
            self.joints = msg
            self.last_seen = time.monotonic()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distances-cm', type=float, nargs='+', default=[5, 10, 15, 20])
    parser.add_argument('--angles-deg', type=float, nargs='+', default=[0, -35, 35])
    args = parser.parse_args()
    if not args.distances_cm or any(not math.isfinite(x) or x <= 0 or x > 25 for x in args.distances_cm):
        parser.error('Distance candidates must be between 0 and 25 cm.')
    if not args.angles_deg or any(not math.isfinite(a) or abs(a) > 90 for a in args.angles_deg):
        parser.error('Candidate angles must be between -90 and +90 degrees.')

    rclpy.init()
    node = Search()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        until = time.monotonic() + 10
        while node.joints is None and time.monotonic() < until:
            time.sleep(.1)
        if node.joints is None or time.monotonic() - node.last_seen > 1.0:
            raise RuntimeError('No fresh /joint_states')
        if not node.ik_client.wait_for_service(timeout_sec=10):
            raise RuntimeError('MoveIt /compute_ik unavailable')
        if not node.move_client.wait_for_server(timeout_sec=10):
            raise RuntimeError('MoveIt /move_action unavailable')

        tf = node.tf_buffer.lookup_transform('base_link','tool0', rclpy.time.Time(),
                                             timeout=Duration(seconds=5))
        p = tf.transform.translation
        q = tf.transform.rotation
        r = math.hypot(p.x, p.y)
        if r < .03:
            raise RuntimeError('TCP too close to base_link XY origin to infer outward direction')
        angle0 = math.atan2(p.y, p.x)
        start = joint_map(node.joints)
        seed = RobotState()
        seed.joint_state = node.joints
        print(f'LIVE TCP A: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) m')
        print(f'Base-frame XY radius: {r*100:.1f} cm')
        print('PLAN ONLY: no motion commands; tool orientation fixed; 30-degree joint screen.')
        print('NOTE: outward means increasing base_link XY radius; it does not guarantee arm straightening.')
        print('Targets are candidate POSES, not a prevalidated safe route or execution command.\n')

        successes = []
        count = 0
        for dist in args.distances_cm:
            for turn in args.angles_deg:
                count += 1
                heading = angle0 + math.radians(turn)
                bx = p.x + (dist/100)*math.cos(heading)
                by = p.y + (dist/100)*math.sin(heading)
                bz = p.z
                target = PoseStamped()
                target.header.frame_id = 'base_link'
                target.header.stamp = node.get_clock().now().to_msg()
                target.pose.position.x = bx
                target.pose.position.y = by
                target.pose.position.z = bz
                target.pose.orientation = q
                print(f'[{count}] {dist:g} cm, direction {turn:+g} deg: B=({bx:.3f},{by:.3f},{bz:.3f})', flush=True)
                # Only request a plan from the original live joint configuration.
                ik_req = GetPositionIK.Request()
                ik_req.ik_request.group_name = 'ur_manipulator'
                ik_req.ik_request.ik_link_name = 'tool0'
                ik_req.ik_request.pose_stamped = target
                ik_req.ik_request.robot_state = seed
                ik_req.ik_request.avoid_collisions = True
                ik_req.ik_request.timeout.sec = 3
                try:
                    ik = wait_future(node.ik_client.call_async(ik_req), 8)
                    if ik.error_code.val != 1:
                        print(f'  FAIL IK code {ik.error_code.val}')
                        continue
                    target_joints = joint_map(ik.solution.joint_state)
                    deg = max_joint_change(start, target_joints)
                    if deg > 30:
                        print(f'  REJECT IK joint change {deg:.1f}° > 30°')
                        continue
                    constraints = Constraints()
                    for n in JOINTS:
                        c = JointConstraint()
                        c.joint_name = n
                        c.position = target_joints[n]
                        c.tolerance_above = .005
                        c.tolerance_below = .005
                        c.weight = 1.0
                        constraints.joint_constraints.append(c)
                    request = MotionPlanRequest()
                    request.group_name = 'ur_manipulator'
                    request.start_state = seed
                    request.goal_constraints = [constraints]
                    request.num_planning_attempts = 5
                    request.allowed_planning_time = 5.0
                    request.max_velocity_scaling_factor = .01
                    request.max_acceleration_scaling_factor = .01
                    goal = MoveGroup.Goal()
                    goal.request = request
                    goal.planning_options = PlanningOptions()
                    goal.planning_options.plan_only = True
                    goal.planning_options.replan = False
                    handle = wait_future(node.move_client.send_goal_async(goal), 12)
                    if handle is None or not handle.accepted:
                        print('  FAIL: planning goal rejected')
                        continue
                    result = wait_future(handle.get_result_async(), 25).result
                    jt = result.planned_trajectory.joint_trajectory
                    if result.error_code.val != 1 or not jt.points or not all(n in jt.joint_names for n in JOINTS):
                        print(f'  FAIL planning code={result.error_code.val}')
                        continue
                    if any(len(pt.positions) != len(jt.joint_names) or
                           not all(math.isfinite(x) for x in pt.positions) for pt in jt.points):
                        print('  REJECT invalid planned points')
                        continue
                    biggest = max(max_joint_change(start, dict(zip(jt.joint_names,pt.positions))) for pt in jt.points)
                    if biggest > 30:
                        print(f'  REJECT waypoint joint change {biggest:.1f}° > 30°')
                        continue
                    if max_joint_change(start, dict(zip(jt.joint_names,jt.points[0].positions))) > 1.0:
                        print('  REJECT plan start differs from measured start by >1°')
                        continue
                    reach_gain = 100*(math.hypot(bx,by)-r)
                    print(f'  PLAN OK: {len(jt.points)} points, XY reach gain {reach_gain:.1f} cm, largest joint deviation {biggest:.1f}°')
                    successes.append((reach_gain, dist, turn, bx, by, bz, biggest))
                except Exception as e:
                    print(f'  ERROR: {e}')

        print(f'\nRESULT: {len(successes)}/{count} candidates have acceptable plans (NOT executed).')
        for i, (gain,dist,turn,bx,by,bz,deg) in enumerate(sorted(successes, reverse=True), 1):
            print(f'  {i}. requested {dist:g}cm at {turn:+g}°, XY reach gain {gain:.1f}cm;'
                  f' B=({bx:.4f},{by:.4f},{bz:.4f}) m; max joint change {deg:.1f}°')
        if not successes:
            print('No candidate passed. Do NOT relax safety checks to force execution.')
        print('NO PHYSICAL ROBOT EXECUTION. Replan from live state before any future approved movement.')
    except Exception as e:
        print('STOP:', e, file=sys.stderr)
        sys.exit(1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""UR3 ROS2 Jazzy: guarded single short IK motion, plan-only by default.

NOT a safety-certified controller. Real motion is possible ONLY with --execute AND
an interactive approval after planning. It never runs the previous multi-route paths.
Requires MoveIt, UR driver, robot_program_running topic, and controllers.
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (RobotState, JointConstraint, Constraints,
                             MotionPlanRequest, PlanningOptions)
from moveit_msgs.srv import GetPositionIK
from tf2_ros import Buffer, TransformListener

JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


class Ur3(Node):
    def __init__(self):
        super().__init__('ur3_guarded_ik_execution')
        self.joint_msg = None
        self.joint_seen = 0.0
        self.program_running = None
        self.program_seen = 0.0
        self.create_subscription(JointState, '/joint_states', self.on_joint, 10)
        self.create_subscription(Bool, '/io_and_status_controller/robot_program_running',
                                 self.on_program, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.execution = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')

    def on_joint(self, msg):
        if all(x in msg.name for x in JOINTS):
            self.joint_msg = msg
            self.joint_seen = time.monotonic()

    def on_program(self, msg):
        self.program_running = bool(msg.data)
        self.program_seen = time.monotonic()


def await_result(future, timeout=30):
    evt = threading.Event()
    future.add_done_callback(lambda _: evt.set())
    if not evt.wait(timeout):
        raise RuntimeError('ROS operation timed out')
    return future.result()


def positions(msg):
    return dict(zip(msg.name, msg.position))


def delta_deg(a, b, name):
    return abs(math.degrees(math.remainder(b[name] - a[name], 2 * math.pi)))


def get_live(node):
    if node.joint_msg is None or time.monotonic()-node.joint_seen > 1.0:
        raise RuntimeError('No fresh /joint_states')
    return positions(node.joint_msg)



def await_fresh_program(node, timeout=15.0):
    """Wait for an observed, recent true state; never infer it from a prior run."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if (node.program_running is True and
                time.monotonic() - node.program_seen <= 3.0):
            return
        time.sleep(0.1)
    age = (time.monotonic() - node.program_seen) if node.program_seen else None
    raise RuntimeError(
        'No fresh robot_program_running=true message within 15 s '
        f'(last_state={node.program_running}, age_seconds={age}). '
        'Check the ROS topic and External Control program; do not bypass this check.'
    )

def check_hardware(node):
    if node.program_running is not True or time.monotonic()-node.program_seen > 3.0:
        raise RuntimeError('UR External Control program not confirmed RUNNING (fresh data required)')
    if not node.controllers.wait_for_service(timeout_sec=3):
        raise RuntimeError('Controller manager unavailable')
    result = await_result(node.controllers.call_async(ListControllers.Request()), 5)
    states = {x.name: x.state for x in result.controller}
    if states.get('scaled_joint_trajectory_controller') != 'active':
        raise RuntimeError('scaled_joint_trajectory_controller is not active')
    if states.get('joint_state_broadcaster') != 'active':
        raise RuntimeError('joint_state_broadcaster is not active')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dx-cm', type=float, default=2.0)
    parser.add_argument('--dy-cm', type=float, default=0.0)
    parser.add_argument('--dz-cm', type=float, default=0.0)
    parser.add_argument('--execute', action='store_true',
                        help='Allow physical execution AFTER interactive approval')
    args = parser.parse_args()
    dist = math.sqrt(args.dx_cm**2 + args.dy_cm**2 + args.dz_cm**2)
    if not 0.2 <= dist <= 3.0:
        parser.error('For the first physical test, displacement must be 0.2–3.0 cm')

    rclpy.init()
    node = Ur3()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        print('Waiting for fresh UR program and joint-state updates (up to 15 s)...')
        await_fresh_program(node)
        expiry = time.monotonic() + 5
        while node.joint_msg is None and time.monotonic() < expiry:
            time.sleep(0.1)
        check_hardware(node)
        current = get_live(node)
        if not node.ik.wait_for_service(timeout_sec=8):
            raise RuntimeError('/compute_ik unavailable')
        if not node.move.wait_for_server(timeout_sec=8):
            raise RuntimeError('/move_action unavailable')
        tf = node.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time(),
                                              timeout=Duration(seconds=5.0))
        p = tf.transform.translation
        q = tf.transform.rotation
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.header.stamp = node.get_clock().now().to_msg()
        target.pose.position.x = p.x + args.dx_cm / 100
        target.pose.position.y = p.y + args.dy_cm / 100
        target.pose.position.z = p.z + args.dz_cm / 100
        target.pose.orientation = q
        print('LIVE TCP A:', tuple(round(x,4) for x in (p.x,p.y,p.z)), 'm')
        print('GOAL TCP B:', tuple(round(x,4) for x in (target.pose.position.x,
                    target.pose.position.y,target.pose.position.z)), 'm')
        print('Orientation: kept fixed. Planning speed/acceleration scaling: 1%.')
        print('MODE:', 'PHYSICAL EXECUTION POSSIBLE AFTER CONFIRMATION' if args.execute else 'PLAN ONLY')

        start_state = RobotState()
        start_state.joint_state = node.joint_msg
        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = 'ur_manipulator'
        ik_req.ik_request.ik_link_name = 'tool0'
        ik_req.ik_request.pose_stamped = target
        ik_req.ik_request.robot_state = start_state
        ik_req.ik_request.avoid_collisions = True
        ik_req.ik_request.timeout.sec = 5
        ik_res = await_result(node.ik.call_async(ik_req), 12)
        if ik_res.error_code.val != 1:
            raise RuntimeError(f'Collision-aware IK rejected target: code {ik_res.error_code.val}')
        goal_joints = positions(ik_res.solution.joint_state)
        maximum = max(delta_deg(current,goal_joints,n) for n in JOINTS)
        print(f'IK max starting joint change: {maximum:.2f} degrees')
        if maximum > 30:
            raise RuntimeError('IK exceeds 30-degree joint screen')

        constraints = Constraints()
        for name in JOINTS:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = goal_joints[name]
            jc.tolerance_above = .005
            jc.tolerance_below = .005
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req = MotionPlanRequest()
        req.group_name = 'ur_manipulator'
        req.start_state = start_state
        req.goal_constraints = [constraints]
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.01
        req.max_acceleration_scaling_factor = 0.01
        move_goal = MoveGroup.Goal()
        move_goal.request = req
        move_goal.planning_options = PlanningOptions()
        move_goal.planning_options.plan_only = True
        move_goal.planning_options.replan = False
        print('Planning only...')
        handle = await_result(node.move.send_goal_async(move_goal), 15)
        if handle is None or not handle.accepted:
            raise RuntimeError('MoveGroup rejected planning request')
        wrapped = await_result(handle.get_result_async(), 45)
        result = wrapped.result
        points = result.planned_trajectory.joint_trajectory.points
        names = list(result.planned_trajectory.joint_trajectory.joint_names)
        if result.error_code.val != 1 or len(points) < 2 or not all(n in names for n in JOINTS):
            raise RuntimeError(f'Planning unsuccessful: code={result.error_code.val}, points={len(points)}')
        largest = 0
        for point in points:
            if not all(math.isfinite(v) for v in point.positions):
                raise RuntimeError('Non-finite planned joint position')
            j = dict(zip(names, point.positions))
            largest = max(largest, max(delta_deg(current,j,n) for n in JOINTS))
        if largest > 30.0:
            raise RuntimeError(f'Planned waypoint changes {largest:.2f}deg >30deg screen')
        first = dict(zip(names, points[0].positions))
        last = dict(zip(names, points[-1].positions))
        if max(delta_deg(current,first,n) for n in JOINTS) > 1.0:
            raise RuntimeError('Planned trajectory start differs from live start >1deg')
        if max(delta_deg(goal_joints,last,n) for n in JOINTS) > 1.0:
            raise RuntimeError('Planned trajectory endpoint differs from IK target >1deg')
        duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec / 1e9
        print(f'PLAN OK: {len(points)} points; estimated trajectory duration {duration:.1f}s; max joint deviation {largest:.2f}deg')
        print('Physical clearance and planning-scene fidelity MUST be independently reviewed.')
        if not args.execute:
            print('PLAN-ONLY COMPLETE. No commands sent to execution controller.')
            return
        if duration <= 0:
            raise RuntimeError('Invalid trajectory time parameterization')
        if not node.execution.wait_for_server(timeout_sec=5):
            raise RuntimeError('/execute_trajectory unavailable')
        print('\nFINAL PHYSICAL SAFETY CHECK:')
        print(' - Confirm YOUR move_group planning scene includes ALL nearby obstacles and attached tool.')
        print(' - Confirm teach pendant available, speed limited, workspace clear, no safety warnings.')
        print(' - Confirm full trajectory and START/END have been inspected by the responsible operator.')
        print(' - Execution will start IMMEDIATELY after entering the exact approval phrase.')
        approval = input('Type EXECUTE UR3 NOW to authorize ONE physical motion (anything else aborts): ').strip()
        if approval != 'EXECUTE UR3 NOW':
            print('Canceled; no physical execution requested.')
            return
        await_fresh_program(node)
        check_hardware(node)
        fresh = get_live(node)
        start_drift = max(delta_deg(current,fresh,n) for n in JOINTS)
        if start_drift > 0.5:
            raise RuntimeError(f'Robot moved {start_drift:.2f}deg since planning. Abort and replan.')
        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = result.planned_trajectory
        print('Sending ONE trajectory to MoveIt execution action...')
        execute_handle = await_result(node.execution.send_goal_async(execute_goal), 10)
        if execute_handle is None or not execute_handle.accepted:
            raise RuntimeError('Execution request rejected')
        executed = await_result(execute_handle.get_result_async(), max(30, duration * 4 + 20))
        print('Execution action status:', executed.status,
              '| MoveIt code:', executed.result.error_code.val)
        if executed.result.error_code.val != 1:
            raise RuntimeError('Execution did not report success: check teach pendant and move_group logs')
        time.sleep(0.5)
        after = get_live(node)
        final_diff = max(delta_deg(after,last,n) for n in JOINTS)
        print(f'Finished; maximum final joint deviation from planned goal: {final_diff:.2f}deg')
        print('Verify TCP endpoint separately using tf2_echo base_link tool0.')
    except (KeyboardInterrupt, EOFError):
        print('Aborted. If motion is active, use the teach pendant safety controls.')
    except Exception as exc:
        print('STOP:', exc, file=sys.stderr)
        sys.exit(1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Plan and visualize alternative A->waypoint->B UR3 routes. NEVER execute motion.

This is exploratory visualization, not a robot safety validation. Requires running
UR MoveIt 2 compute_ik, compute_fk, move_action and joint_states on ROS 2 Jazzy.
"""
import argparse
import copy
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions, RobotState, DisplayTrajectory, RobotTrajectory)
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

JOINTS = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
          'wrist_1_joint','wrist_2_joint','wrist_3_joint']
TOPIC = '/ur3_ik_tcp_path'
MAX_DEG = 30.0
COLORS = [(0.9,0.3,0.25),(0.1,0.8,0.4),(0.3,0.65,1.0),
          (0.95,0.75,0.08),(0.8,0.4,0.9),(0.1,0.8,0.85),
          (0.95,0.55,0.25),(0.55,0.9,0.2)]
SEEDS = [None, ('shoulder_lift_joint', 8), ('shoulder_lift_joint', -8),
         ('elbow_joint', 8), ('elbow_joint', -8),
         ('wrist_1_joint', 8), ('wrist_1_joint', -8)]

class Preview(Node):
    def __init__(self):
        super().__init__('ur3_fixed_target_alternative_routes_animation')
        self.joints = None
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.move = ActionClient(self, MoveGroup, '/move_action')
        qos = QoSProfile(depth=80, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Marker, TOPIC, qos)
        self.animation_pub = self.create_publisher(
            DisplayTrajectory, '/display_planned_path',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))

    def on_joints(self, msg):
        if all(j in msg.name for j in JOINTS):
            self.joints = msg

def wait(future, timeout=30):
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    if not event.wait(timeout):
        raise TimeoutError('ROS request timed out')
    return future.result()

def mapping(msg):
    return dict(zip(msg.name, msg.position))

def deviation(a, b):
    return max(abs(math.degrees(math.remainder(b[j]-a[j], 2*math.pi))) for j in JOINTS)

def state_from(positions):
    state = RobotState()
    state.joint_state.name = list(JOINTS)
    state.joint_state.position = [positions[j] for j in JOINTS]
    return state

def pose_at(start, orientation, xyz):
    ps = PoseStamped()
    ps.header.frame_id = 'base_link'
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xyz
    ps.pose.orientation = orientation
    return ps

def find_ik(node, target_pose, seed_state, live_start):
    found = []
    for variation in SEEDS:
        seed = copy.deepcopy(seed_state)
        if variation:
            name, deg = variation
            k = seed.joint_state.name.index(name)
            seed.joint_state.position[k] += math.radians(deg)
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'ur_manipulator'
        req.ik_request.ik_link_name = 'tool0'
        req.ik_request.pose_stamped = target_pose
        req.ik_request.robot_state = seed
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 2
        try:
            reply = wait(node.ik.call_async(req), 6)
        except TimeoutError:
            continue
        if reply.error_code.val != 1:
            continue
        result = mapping(reply.solution.joint_state)
        if not all(j in result for j in JOINTS):
            continue
        if deviation(live_start, result) > MAX_DEG:
            continue
        if not any(deviation(result, previous) < 0.5 for previous in found):
            found.append(result)
    found.sort(key=lambda x: deviation(live_start,x))
    return found

def plan(node, from_state, target, live_start):
    req = MotionPlanRequest()
    req.group_name = 'ur_manipulator'
    req.num_planning_attempts = 8
    req.allowed_planning_time = 8.0
    req.max_velocity_scaling_factor = 0.05
    req.max_acceleration_scaling_factor = 0.05
    req.start_state = from_state
    constraints = Constraints()
    for j in JOINTS:
        c = JointConstraint()
        c.joint_name = j
        c.position = target[j]
        c.tolerance_above = 0.01
        c.tolerance_below = 0.01
        c.weight = 1.0
        constraints.joint_constraints.append(c)
    req.goal_constraints = [constraints]
    goal = MoveGroup.Goal()
    goal.request = req
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = True
    handle = wait(node.move.send_goal_async(goal), 15)
    if not handle.accepted:
        return None
    reply = wait(handle.get_result_async(), 30).result
    trajectory = reply.planned_trajectory.joint_trajectory
    if reply.error_code.val != 1 or len(trajectory.points) < 2:
        return None
    if not all(j in trajectory.joint_names for j in JOINTS):
        return None
    coords = [dict(zip(trajectory.joint_names, pt.positions)) for pt in trajectory.points]
    if any(deviation(live_start, item) > MAX_DEG for item in coords):
        return None
    return coords

def fk_path(node, coords):
    results = []
    for joints in coords:
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['tool0']
        req.robot_state = state_from(joints)
        reply = wait(node.fk.call_async(req), 10)
        if reply.error_code.val != 1 or not reply.pose_stamped:
            return []
        position = reply.pose_stamped[0].pose.position
        results.append(Point(x=position.x,y=position.y,z=position.z))
    return results

def line_marker(idx, pts):
    m = Marker()
    m.header.frame_id = 'base_link'
    m.ns = 'alternative_routes'
    m.id = idx
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.0015
    m.color.r, m.color.g, m.color.b = COLORS[idx % len(COLORS)]
    m.color.a = 1.0
    m.points = pts
    return m

def target_marker(xyz, q):
    m = Marker()
    m.header.frame_id = 'base_link'
    m.ns = 'fixed_target_b'
    m.id = 999
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
    m.pose.orientation = q
    m.scale.x, m.scale.y, m.scale.z = .06, .006, .006
    m.color.r, m.color.g, m.color.b, m.color.a = 1., .1, .1, 1.
    return m

def path_length(pts):
    return sum(math.dist((a.x,a.y,a.z), (b.x,b.y,b.z)) for a,b in zip(pts,pts[1:]))

def is_distinct(candidate, earlier):
    # Compare sample points at normalized arc lengths to reject near-identical paths.
    def samples(points):
        if len(points) < 2: return []
        segment = [math.dist((a.x,a.y,a.z),(b.x,b.y,b.z)) for a,b in zip(points,points[1:])]
        total = sum(segment)
        if total < 1e-6: return []
        output = []
        for frac in (.2,.4,.6,.8):
            travel = frac*total
            for i, length in enumerate(segment):
                if travel <= length or i == len(segment)-1:
                    t = min(1.,travel/max(length,1e-9))
                    a,b = points[i],points[i+1]
                    output.append((a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)))
                    break
                travel -= length
        return output
    this = samples(candidate)
    for old in earlier:
        other = samples(old)
        if len(this)==len(other)==4 and max(math.dist(a,b) for a,b in zip(this,other)) < .015:
            return False
    return True

def publish_preview(node, start_state, joint_positions, seconds_per_point=0.12):
    """Publish a VISUALIZATION-ONLY animation; never send a controller goal."""
    msg = DisplayTrajectory()
    msg.trajectory_start = copy.deepcopy(start_state)
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(JOINTS)
    for index, positions in enumerate(joint_positions):
        pt = JointTrajectoryPoint()
        pt.positions = [float(positions[j]) for j in JOINTS]
        pt.time_from_start = Duration(seconds=index * seconds_per_point).to_msg()
        trajectory.joint_trajectory.points.append(pt)
    msg.trajectory = [trajectory]
    node.animation_pub.publish(msg)
    return max(3.0, len(joint_positions)*seconds_per_point + 1.5)

def main():
    parser = argparse.ArgumentParser(description='Several routes to ONE target B using alternative intermediate poses; planning only.')
    parser.add_argument('--dx-cm', type=float, default=18)
    parser.add_argument('--dy-cm', type=float, default=0)
    parser.add_argument('--dz-cm', type=float, default=0)
    parser.add_argument('--detour-cm', type=float, default=3, help='Magnitude of sideways/up/down waypoint detours')
    parser.add_argument('--no-animation', action='store_true', help='Show lines only, do not animate robot in RViz')
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.dx_cm,args.dy_cm,args.dz_cm,args.detour_cm)):
        parser.error('Inputs must be finite')
    if math.dist((0,0,0), (args.dx_cm,args.dy_cm,args.dz_cm)) > 35:
        parser.error('Target displacement cannot exceed 35 cm')
    if not 0 < args.detour_cm <= 8:
        parser.error('Detour must be between 0 and 8 cm')
    rclpy.init()
    node = Preview()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin,daemon=True)
    worker.start()
    try:
        end = time.monotonic()+12
        while node.joints is None and time.monotonic()<end: time.sleep(.1)
        if node.joints is None: raise RuntimeError('No live joint states')
        for client, label in [(node.ik,'/compute_ik'),(node.fk,'/compute_fk')]:
            if not client.wait_for_service(timeout_sec=10): raise RuntimeError(label+' missing')
        if not node.move.wait_for_server(timeout_sec=10): raise RuntimeError('/move_action missing')
        initial_msg = copy.deepcopy(node.joints)
        initial = mapping(initial_msg)
        if not all(math.isfinite(initial[j]) for j in JOINTS): raise RuntimeError('Bad joint states')
        start_state = state_from(initial)
        tf = node.tf.lookup_transform('base_link','tool0',rclpy.time.Time(),timeout=Duration(seconds=5))
        p,q = tf.transform.translation,tf.transform.rotation
        A = (p.x,p.y,p.z)
        B = (p.x+args.dx_cm/100,p.y+args.dy_cm/100,p.z+args.dz_cm/100)
        print(f'LIVE A: {A}; FIXED TARGET B: {B}')
        print('PLAN ONLY; no trajectory execution. All routes share the same A and B.')
        print(f'Joint deviation screen: {MAX_DEG:.0f} degrees from A at EVERY waypoint.')
        clear = Marker()
        clear.header.frame_id='base_link'
        clear.action=Marker.DELETEALL
        node.pub.publish(clear)
        time.sleep(.2)
        goal_mark = target_marker(B,q)
        node.pub.publish(goal_mark)
        drawn=[]
        all_xyz=[]
        route_joint_paths=[]
        route_labels=[]
        goal_pose = pose_at(A,q,B)
        goals = find_ik(node,goal_pose,start_state,initial)
        if not goals:
            print('Target B has no IK solution within 30 degrees. Try a shorter/smaller target offset.')
            return
        print(f'Target B: {len(goals)} acceptable IK solution(s)')
        # Baseline A->B.
        for target in goals[:2]:
            joint_path = plan(node,start_state,target,initial)
            if joint_path:
                xyz = fk_path(node,joint_path)
                if len(xyz)>1 and is_distinct(xyz,all_xyz):
                    all_xyz.append(xyz)
                    route_joint_paths.append(joint_path)
                    route_labels.append('DIRECT')
                    drawn.append(line_marker(len(drawn),xyz))
                    node.pub.publish(drawn[-1])
                    print(f'  DIRECT: TCP {path_length(xyz)*100:.1f} cm, {len(xyz)} points')
        # Waypoint at 35%, 50%, or 65% along A->B, displaced in base Y/Z.
        d=args.detour_cm/100
        candidates=[]
        for portion in (.35,.50,.65):
            mid=tuple(a+portion*(b-a) for a,b in zip(A,B))
            for axis,delta in [('Y+', (0,d,0)),('Y-',(0,-d,0)),
                               ('Z+',(0,0,d)),('Z-',(0,0,-d))]:
                candidates.append((f'{axis} at {portion:.0%}',tuple(mid[i]+delta[i] for i in range(3))))
        for i,(label,point) in enumerate(candidates,1):
            print(f'[{i}/{len(candidates)}] Via {label}: ',end='',flush=True)
            mid_pose=pose_at(A,q,point)
            middle_solutions=find_ik(node,mid_pose,start_state,initial)
            found=False
            for mid_solution in middle_solutions[:2]:
                first=plan(node,start_state,mid_solution,initial)
                if not first: continue
                mid_state=state_from(first[-1])
                # Recompute endpoint IK starting from the planned intermediate joint state.
                final_solutions=find_ik(node,goal_pose,mid_state,initial)
                for final in final_solutions[:2]:
                    second=plan(node,mid_state,final,initial)
                    if not second: continue
                    combined=first+second[1:]
                    xyz=fk_path(node,combined)
                    if len(xyz)<2: continue
                    if not is_distinct(xyz,all_xyz): continue
                    all_xyz.append(xyz)
                    route_joint_paths.append(combined)
                    route_labels.append('Via '+label)
                    m=line_marker(len(drawn),xyz)
                    drawn.append(m)
                    node.pub.publish(m)
                    print(f'OK, path {path_length(xyz)*100:.1f} cm; {len(xyz)} points')
                    found=True
                    break
                if found: break
            if not found: print('no distinct acceptable route')
        print(f'\nFINISHED: {len(drawn)} distinct routes to ONE fixed target B.')
        print('Marker topic:',TOPIC,'(1.5 mm lines). NO PHYSICAL EXECUTION.')
        print('Keeping markers visible for at least 120 seconds (Ctrl+C to stop).')
        print('Animation affects RViz virtual robot ONLY; physical execution disabled.')
        # Finish planning before publishing display animations, so move_group's own
        # planning displays do not immediately overwrite our preview.
        if not args.no_animation and route_joint_paths:
            print('RViz -> MotionPlanning -> Planned Path -> Show Robot Visual: ON')
            time.sleep(1.0)
            for idx, (label, positions) in enumerate(zip(route_labels, route_joint_paths), 1):
                print(f'ANIMATING {idx}/{len(route_joint_paths)}: {label} ({len(positions)} points)', flush=True)
                duration=publish_preview(node, start_state, positions)
                until=time.monotonic()+duration
                while time.monotonic()<until:
                    goal_mark.header.stamp=node.get_clock().now().to_msg()
                    node.pub.publish(goal_mark)
                    for m in drawn:
                        m.header.stamp=node.get_clock().now().to_msg()
                        node.pub.publish(m)
                    time.sleep(.5)
        end_time=time.monotonic()+120
        while time.monotonic()<end_time:
            goal_mark.header.stamp=node.get_clock().now().to_msg()
            node.pub.publish(goal_mark)
            for m in drawn:
                m.header.stamp=node.get_clock().now().to_msg()
                node.pub.publish(m)
            time.sleep(.5)
    except KeyboardInterrupt:
        print('\nStopped. No robot execution requested.')
    except Exception as e:
        print('ERROR:',e,file=sys.stderr)
        sys.exit(1)
    finally:
        executor.shutdown(timeout_sec=5)
        worker.join(timeout=5)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()

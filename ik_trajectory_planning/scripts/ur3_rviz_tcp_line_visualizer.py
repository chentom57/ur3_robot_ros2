#!/usr/bin/env python3
"""Visualize the TCP line of MoveIt's newest RViz-planned trajectory.

ROS 2 Jazzy / MoveIt 2. Visualization only: no action or execution client.
Requires /display_planned_path and /compute_fk from MoveIt.
"""
import threading
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from moveit_msgs.msg import DisplayTrajectory, RobotState
from moveit_msgs.srv import GetPositionFK
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

FK_LINK = 'tool0'       # actual UR3 TCP frame used in previous experiments
FRAME = 'base_link'
INPUT_TOPIC = '/display_planned_path'
OUTPUT_TOPIC = '/ur3_ik_tcp_path'
LINE_WIDTH = 0.002  # 2 mm
MAX_SAMPLES = 100


class TcpLines(Node):
    def __init__(self):
        super().__init__('ur3_rviz_tcp_line_visualizer')
        self.latest_generation = 0
        self.latest_lock = threading.Lock()
        self.live_joints = None
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        # Compatible with MoveIt's volatile and transient-local publishers.
        input_qos = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        marker_qos = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.line_pub = self.create_publisher(Marker, OUTPUT_TOPIC, marker_qos)
        self.create_subscription(DisplayTrajectory, INPUT_TOPIC,
                                 self.on_trajectory, input_qos)
        self.create_subscription(JointState, '/joint_states',
                                 self.on_joints, 10)
        self.get_logger().info('Listening for MoveIt paths on %s' % INPUT_TOPIC)
        self.get_logger().info('Will publish 2 mm TCP lines on %s; NEVER executes motion' % OUTPUT_TOPIC)

    def on_joints(self, msg):
        self.live_joints = msg

    def on_trajectory(self, msg):
        paths = [t.joint_trajectory for t in msg.trajectory
                 if t.joint_trajectory.points and t.joint_trajectory.joint_names]
        if not paths:
            return
        # DisplayTrajectory may contain several consecutive segments; combine their waypoints.
        with self.latest_lock:
            self.latest_generation += 1
            generation = self.latest_generation
        threading.Thread(target=self.make_line,
                         args=(msg.trajectory_start, paths, generation),
                         daemon=True).start()

    def still_latest(self, generation):
        with self.latest_lock:
            return generation == self.latest_generation

    def make_line(self, start_state, paths, generation):
        if not self.fk_client.wait_for_service(timeout_sec=5):
            self.get_logger().warn('/compute_fk unavailable: start MoveIt move_group')
            return
        # Prefer the trajectory start state; supplement with current joints if necessary.
        state_map = {}
        if self.live_joints is not None:
            state_map.update(zip(self.live_joints.name, self.live_joints.position))
        state_map.update(zip(start_state.joint_state.name,
                             start_state.joint_state.position))
        configs = []
        for trajectory in paths:
            names = list(trajectory.joint_names)
            for pt in trajectory.points:
                if len(pt.positions) != len(names):
                    continue
                state_map.update(zip(names, pt.positions))
                configs.append(dict(state_map))
        if not configs:
            return
        stride = max(1, math.ceil(len(configs) / MAX_SAMPLES))
        sampled = configs[::stride]
        if sampled[-1] is not configs[-1]:
            sampled.append(configs[-1])
        points = []
        for i, positions in enumerate(sampled):
            if not self.still_latest(generation):
                return
            req = GetPositionFK.Request()
            req.header.frame_id = FRAME
            req.fk_link_names = [FK_LINK]
            req.robot_state = RobotState()
            req.robot_state.joint_state.name = list(positions)
            req.robot_state.joint_state.position = [positions[n] for n in positions]
            future = self.fk_client.call_async(req)
            event = threading.Event()
            future.add_done_callback(lambda _: event.set())
            if not event.wait(timeout=5):
                self.get_logger().warn('Timed out waiting for /compute_fk')
                return
            try:
                response = future.result()
            except Exception as exc:
                self.get_logger().warn('FK service error: %s' % exc)
                return
            if response is None or response.error_code.val != 1 or not response.pose_stamped:
                self.get_logger().warn('FK failed at point %d/%d' % (i+1, len(sampled)))
                return
            xyz = response.pose_stamped[0].pose.position
            points.append(Point(x=xyz.x, y=xyz.y, z=xyz.z))
        if not self.still_latest(generation) or len(points) < 2:
            return
        marker = Marker()
        marker.header.frame_id = FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'latest_moveit_tcp_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = LINE_WIDTH
        marker.color.r = 0.05
        marker.color.g = 0.95
        marker.color.b = 0.95
        marker.color.a = 1.0
        marker.points = points
        self.line_pub.publish(marker)
        self.get_logger().info('Published TCP line (%d points, newest planned path)' % len(points))


def main():
    rclpy.init()
    node = TcpLines()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

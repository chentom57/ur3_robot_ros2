#!/usr/bin/env python3
"""Interactive UR3 pick prototype driven by a 3-D point in the camera frame.

The input layer is deliberately isolated so a future detector can call
``pick_camera_point`` with its estimated grasp point instead of using input().
"""

import math
import socket
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def transform_point(point, transform):
    """Transform a 3-D point using a geometry_msgs TransformStamped."""
    tr = transform.transform.translation
    rot = transform.transform.rotation
    q = (rot.x, rot.y, rot.z)
    twice_cross = tuple(2.0 * value for value in _cross(q, point))
    rotated = tuple(
        point[i] + rot.w * twice_cross[i] + _cross(q, twice_cross)[i]
        for i in range(3)
    )
    return (rotated[0] + tr.x, rotated[1] + tr.y, rotated[2] + tr.z)


class RobotiqTcpGripper:
    """Small TCP adapter for the existing Robotiq string-command interface."""

    def __init__(self, host, port, timeout=2.0):
        self.socket = socket.create_connection((host, port), timeout=timeout)
        self.socket.settimeout(timeout)

    def close(self):
        if self.socket:
            self.socket.close()
            self.socket = None

    def _set(self, *name_values):
        command = "SET " + " ".join(str(value) for value in name_values) + "\n"
        self.socket.sendall(command.encode("utf-8"))
        if self.socket.recv(1024) != b"ack":
            raise RuntimeError("Robotiq controller did not acknowledge the command")

    def _get(self, name):
        self.socket.sendall(f"GET {name}\n".encode("utf-8"))
        response = self.socket.recv(1024).decode("utf-8").split()
        if len(response) != 2 or response[0] != name:
            raise RuntimeError(f"Unexpected Robotiq response: {response}")
        return int(response[1])

    def activate(self):
        if self._get("STA") == 3:
            return
        self._set("ACT", 0, "ATR", 0)
        deadline = time.monotonic() + 5.0
        while self._get("STA") != 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        self._set("ACT", 1)
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if self._get("STA") == 3:
                return
            time.sleep(0.1)
        raise TimeoutError("Robotiq gripper did not become active")

    def move_and_wait(self, position, speed=80, force=80):
        self._set("POS", position, "SPE", speed, "FOR", force, "GTO", 1)
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if self._get("OBJ") != 0:
                return
            time.sleep(0.05)
        raise TimeoutError("Timed out waiting for Robotiq gripper movement")


class CoordinatePickDemo:
    def __init__(self):
        self.node = Node("ur3_coordinate_pick")
        self.node.declare_parameter("robot_ip", "192.168.86.7")
        self.node.declare_parameter("gripper_port", 63352)
        self.node.declare_parameter("gripper_backend", "tcp")
        self.node.declare_parameter("base_frame", "base_link")
        self.node.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.node.declare_parameter("tool_link", "tool0")
        self.node.declare_parameter("planning_group", "ur_manipulator")
        self.node.declare_parameter("approach_height", 0.10)
        self.node.declare_parameter("show_target_marker", True)
        self.robot_ip = self.node.get_parameter("robot_ip").value
        self.gripper_port = self.node.get_parameter("gripper_port").value
        self.gripper_backend = self.node.get_parameter("gripper_backend").value
        if self.gripper_backend not in {"tcp", "none"}:
            raise ValueError("gripper_backend must be 'tcp' or 'none'")
        self.base_frame = self.node.get_parameter("base_frame").value
        self.camera_frame = self.node.get_parameter("camera_frame").value
        self.tool_link = self.node.get_parameter("tool_link").value
        self.planning_group = self.node.get_parameter("planning_group").value
        self.approach_height = float(self.node.get_parameter("approach_height").value)
        self.show_target_marker = bool(self.node.get_parameter("show_target_marker").value)
        self.marker_pub = self.node.create_publisher(Marker, "~/target_markers", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=True)

        moveit_config = (
            MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
            .robot_description_semantic(
                file_path=Path("srdf") / "ur.srdf.xacro", mappings={"name": "ur3"}
            )
            .to_moveit_configs()
        )
        self.robot = MoveItPy(node_name="ur3_coordinate_pick_moveit", config_dict=moveit_config.to_dict())
        self.planning = self.robot.get_planning_component(self.planning_group)
        self.gripper = None

    def camera_point_to_base(self, point_camera):
        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.camera_frame,
            Time(),
            timeout=Duration(seconds=3.0),
        )
        return transform_point(point_camera, transform)

    def current_tool_orientation(self):
        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.tool_link,
            Time(),
            timeout=Duration(seconds=3.0),
        )
        return transform.transform.rotation

    def publish_target_markers(self, point_camera, point_base, approach):
        """Show the entered camera point (transformed into base_frame) in RViz."""
        if not self.show_target_marker:
            return
        stamp = self.node.get_clock().now().to_msg()
        for marker_id, position, kind, color, label in (
            (0, point_base, Marker.SPHERE, (1.0, 0.15, 0.05, 1.0), "GRASP"),
            (1, approach, Marker.SPHERE, (0.1, 0.8, 0.2, 1.0), "APPROACH"),
        ):
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = stamp
            marker.ns = "coordinate_pick"
            marker.id = marker_id
            marker.type = kind
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.045
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            marker.lifetime.sec = 0
            self.marker_pub.publish(marker)

            text = Marker()
            text.header.frame_id = self.base_frame
            text.header.stamp = stamp
            text.ns = "coordinate_pick_labels"
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = position[0]
            text.pose.position.y = position[1]
            text.pose.position.z = position[2] + 0.06
            text.pose.orientation.w = 1.0
            text.scale.z = 0.04
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f"{label}  camera=({point_camera[0]:.3f}, {point_camera[1]:.3f}, {point_camera[2]:.3f}) m"
            text.lifetime.sec = 0
            self.marker_pub.publish(text)

    def plan_and_execute(self, position_base, orientation, label):
        goal = PoseStamped()
        goal.header.frame_id = self.base_frame
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = position_base
        goal.pose.orientation = orientation

        self.planning.set_start_state_to_current_state()
        self.planning.set_goal_state(pose_stamped_msg=goal, pose_link=self.tool_link)
        self.node.get_logger().info(f"Planning {label} pose")
        result = self.planning.plan()
        if not result:
            self.node.get_logger().error(f"MoveIt could not plan the {label} motion")
            return False

        print(f"\n{label} pose in {self.base_frame}: {position_base}")
        if input("Execute after checking the trajectory in RViz and the space around the robot? Enter y: ").strip().lower() != "y":
            print("This motion was canceled.")
            return False
        self.robot.execute(result.trajectory, controllers=[])
        return True

    def pick_camera_point(self, point_camera):
        point_base = self.camera_point_to_base(point_camera)
        orientation = self.current_tool_orientation()
        approach = (point_base[0], point_base[1], point_base[2] + self.approach_height)

        self.publish_target_markers(point_camera, point_base, approach)

        print(f"Camera coordinates {point_camera} m -> base coordinates {point_base} m")
        print("The input point is used as the tool0 target position; it is not an object center or pixel coordinate.")
        if input("Confirm the target coordinates and start picking? Enter y: ").strip().lower() != "y":
            return False

        if self.gripper_backend == "tcp":
            self.gripper = RobotiqTcpGripper(self.robot_ip, self.gripper_port)
        else:
            print("gripper_backend=none: skipping physical gripper commands.")

        try:
            if self.gripper:
                self.gripper.activate()
                self.gripper.move_and_wait(0)
            if not self.plan_and_execute(approach, orientation, "approach"):
                return False
            if not self.plan_and_execute(point_base, orientation, "grasp"):
                return False
            if self.gripper:
                self.gripper.move_and_wait(255)
            return self.plan_and_execute(approach, orientation, "lift")
        finally:
            if self.gripper:
                self.gripper.close()

    def run(self):
        print("Enter camera_frame coordinates x y z in meters; enter q to quit.")
        while rclpy.ok():
            raw = input("Camera x y z> ").strip()
            if raw.lower() in {"q", "quit", "exit"}:
                break
            try:
                values = tuple(float(value) for value in raw.replace(",", " ").split())
                if len(values) != 3 or not all(math.isfinite(value) for value in values):
                    raise ValueError("Enter three finite numeric values for x y z")
                if any(abs(value) > 2.0 for value in values):
                    raise ValueError("Coordinate magnitudes must be at most 2 meters; check the units and frame")
            except ValueError as exc:
                print(f"Invalid input: {exc}")
                continue
            try:
                self.pick_camera_point(values)
            except TransformException as exc:
                self.node.get_logger().error(f"Calibration or robot TF not found: {exc}")
            except (OSError, RuntimeError, TimeoutError) as exc:
                self.node.get_logger().error(f"Pick sequence failed: {exc}")

    def shutdown(self):
        if self.gripper:
            self.gripper.close()
        self.robot.shutdown()
        self.tf_listener.unregister()
        self.node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    app = None
    try:
        app = CoordinatePickDemo()
        app.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Startup failed: {exc}")
        raise
    finally:
        if app:
            app.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

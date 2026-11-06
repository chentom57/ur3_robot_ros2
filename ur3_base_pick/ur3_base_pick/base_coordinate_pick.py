#!/usr/bin/env python3
"""Repeatedly move the UR3 tool to XYZ coordinates expressed in base_link."""

import math
import socket
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def rotate_vector(q, v):
    qv = (q.x, q.y, q.z)
    cross = (
        qv[1] * v[2] - qv[2] * v[1],
        qv[2] * v[0] - qv[0] * v[2],
        qv[0] * v[1] - qv[1] * v[0],
    )
    twice = tuple(2.0 * x for x in cross)
    second = (
        qv[1] * twice[2] - qv[2] * twice[1],
        qv[2] * twice[0] - qv[0] * twice[2],
        qv[0] * twice[1] - qv[1] * twice[0],
    )
    return tuple(v[i] + q.w * twice[i] + second[i] for i in range(3))


def multiply_quaternions(a, b):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
    q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
    q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    return q


class RobotiqGripper:
    """Minimal client for the Robotiq TCP string-command server."""

    def __init__(self, host, port, timeout=2.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None

    def set(self, *items):
        self.sock.sendall(("SET " + " ".join(map(str, items)) + "\n").encode())
        if self.sock.recv(1024) != b"ack":
            raise RuntimeError("Robotiq controller did not acknowledge command")

    def get(self, name):
        self.sock.sendall(f"GET {name}\n".encode())
        reply = self.sock.recv(1024).decode().split()
        if len(reply) != 2 or reply[0] != name:
            raise RuntimeError(f"Unexpected Robotiq response: {reply}")
        return int(reply[1])

    def activate(self):
        if self.get("STA") == 3:
            return
        self.set("ACT", 0, "ATR", 0)
        deadline = time.monotonic() + 5.0
        while self.get("STA") != 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        self.set("ACT", 1)
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if self.get("STA") == 3:
                return
            time.sleep(0.1)
        raise TimeoutError("Robotiq gripper did not become active")

    def move_and_wait(self, position, speed=80, force=80):
        self.set("POS", position, "SPE", speed, "FOR", force, "GTO", 1)
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if self.get("OBJ") != 0:
                return
            time.sleep(0.05)
        raise TimeoutError("Timed out waiting for Robotiq gripper movement")


class BaseCoordinatePick:
    def __init__(self):
        self.node = Node("ur3_base_coordinate_pick")
        self.node.declare_parameter("base_frame", "base_link")
        self.node.declare_parameter("tool_link", "tool0")
        self.node.declare_parameter("planning_group", "ur_manipulator")
        self.node.declare_parameter("robot_ip", "192.168.86.7")
        self.node.declare_parameter("gripper_port", 63352)
        self.node.declare_parameter("gripper_backend", "tcp")
        self.node.declare_parameter("max_velocity_scaling_factor", 0.1)
        self.node.declare_parameter("max_acceleration_scaling_factor", 0.1)
        self.node.declare_parameter("workspace_map_min", -0.60)
        self.node.declare_parameter("workspace_map_max", 0.60)
        self.node.declare_parameter("workspace_map_step", 0.05)
        self.base_frame = self.node.get_parameter("base_frame").value
        self.tool_link = self.node.get_parameter("tool_link").value
        self.group = self.node.get_parameter("planning_group").value
        self.robot_ip = self.node.get_parameter("robot_ip").value
        self.gripper_port = int(self.node.get_parameter("gripper_port").value)
        self.gripper_backend = self.node.get_parameter("gripper_backend").value
        self.velocity_scaling = float(
            self.node.get_parameter("max_velocity_scaling_factor").value
        )
        self.acceleration_scaling = float(
            self.node.get_parameter("max_acceleration_scaling_factor").value
        )
        self.map_min = float(self.node.get_parameter("workspace_map_min").value)
        self.map_max = float(self.node.get_parameter("workspace_map_max").value)
        self.map_step = float(self.node.get_parameter("workspace_map_step").value)
        if self.map_step <= 0.0 or self.map_max <= self.map_min:
            raise ValueError("workspace map range and step must be positive and ordered")
        if self.gripper_backend not in {"tcp", "none"}:
            raise ValueError("gripper_backend must be 'tcp' or 'none'")
        if not 0.0 < self.velocity_scaling <= 1.0:
            raise ValueError("max_velocity_scaling_factor must be in the range (0, 1]")
        if not 0.0 < self.acceleration_scaling <= 1.0:
            raise ValueError("max_acceleration_scaling_factor must be in the range (0, 1]")

        config = (
            MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
            .robot_description_semantic(
                file_path=Path("srdf") / "ur.srdf.xacro", mappings={"name": "ur3"}
            )
            .to_moveit_configs()
        )
        moveit_parameters = config.to_dict()
        # MoveItPy requires this mapping even when MoveItConfigsBuilder returns
        # the list format used by move_group launch configurations.
        moveit_parameters["planning_pipelines"] = {"pipeline_names": ["ompl"]}
        ompl_parameters = moveit_parameters.setdefault("ompl", {})
        ompl_parameters[self.group] = {"planner_configs": ["RRTConnect"]}
        moveit_parameters["plan_request_params"] = {
            "planning_pipeline": "ompl",
            "planner_id": "RRTConnect",
            "planning_time": 5.0,
            "planning_attempts": 5,
            "max_velocity_scaling_factor": self.velocity_scaling,
            "max_acceleration_scaling_factor": self.acceleration_scaling,
        }
        self.robot = MoveItPy(
            node_name="ur3_base_coordinate_pick_moveit", config_dict=moveit_parameters
        )
        self.scene_monitor = self.robot.get_planning_scene_monitor()
        # MoveItPy listens to /planning_scene. The table publisher resends its
        # collision diff whenever this late planning client subscribes.
        self.scene_ready = False
        for _ in range(40):
            with self.scene_monitor.read_only() as scene:
                scene_object_ids = {
                    obj.id for obj in scene.planning_scene_message.world.collision_objects
                }
            if "ur3_virtual_table_collision" in scene_object_ids:
                self.scene_ready = True
                break
            time.sleep(0.2)
        if self.scene_ready:
            self.node.get_logger().info(
                "Received the shared virtual table collision slab on /planning_scene."
            )
        else:
            self.node.get_logger().error(
                "The shared MoveIt scene has no 'ur3_virtual_table_collision'. "
                "Start bringup with the virtual table enabled before planning."
            )
        self.planning = self.robot.get_planning_component(self.group)
        self.node.get_logger().info(
            "Trajectory limits: velocity scaling="
            f"{self.velocity_scaling:.2f}, acceleration scaling={self.acceleration_scaling:.2f}."
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=True)
        self.map_pub = self.node.create_publisher(
            MarkerArray, "~/workspace_map", 10
        )
        self.target_path_pub = self.node.create_publisher(
            MarkerArray, "~/target_and_path_markers", 10
        )
        self.gripper = None

    def show_workspace_slice(self, z):
        """Publish an XY IK map at z for the current tool orientation."""
        model = self.robot.get_robot_model()
        joint_group = model.get_joint_model_group(self.group)
        if joint_group is None:
            raise RuntimeError(f"MoveIt has no joint group named {self.group}")
        model_frame = model.model_frame
        if model_frame == self.base_frame:
            base_to_model = None
        else:
            base_to_model = self.tf_buffer.lookup_transform(
                model_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=3.0),
            ).transform
        tool_tf = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.tool_link,
            Time(),
            timeout=Duration(seconds=3.0),
        ).transform

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        step_count = int(round((self.map_max - self.map_min) / self.map_step))
        reachable = 0
        total = 0
        reachable_rows = []
        for ix in range(step_count + 1):
            x = self.map_min + ix * self.map_step
            row = []
            for iy in range(step_count + 1):
                y = self.map_min + iy * self.map_step
                pose = PoseStamped().pose
                pose.position.x, pose.position.y, pose.position.z = x, y, z
                pose.orientation = tool_tf.rotation
                if base_to_model is not None:
                    rotated = rotate_vector(base_to_model.rotation, (x, y, z))
                    pose.position.x = rotated[0] + base_to_model.translation.x
                    pose.position.y = rotated[1] + base_to_model.translation.y
                    pose.position.z = rotated[2] + base_to_model.translation.z
                    pose.orientation = multiply_quaternions(
                        base_to_model.rotation, pose.orientation
                    )

                found_ik = False
                for attempt in range(2):
                    state = RobotState(model)
                    state.set_to_default_values()
                    if attempt:
                        state.set_to_random_positions(joint_group)
                    try:
                        found_ik = bool(
                            state.set_from_ik(self.group, pose, self.tool_link, 0.01)
                        )
                    except RuntimeError:
                        found_ik = False
                    if found_ik:
                        break

                marker = Marker()
                marker.header.frame_id = self.base_frame
                marker.header.stamp = self.node.get_clock().now().to_msg()
                marker.ns = "workspace_ik_map"
                marker.id = total
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = self.map_step * 0.8
                marker.scale.z = 0.012
                marker.color.r = 0.1 if found_ik else 0.95
                marker.color.g = 0.8 if found_ik else 0.1
                marker.color.b = 0.15
                marker.color.a = 0.85
                markers.markers.append(marker)
                reachable += int(found_ik)
                total += 1
                row.append(found_ik)
            reachable_rows.append((x, row))

        self.map_pub.publish(markers)
        print(
            f"Published XY IK slice at z={z:.3f} m: {reachable} cells have an IK solution (green), "
            f"and {total - reachable} currently have no IK solution (red)."
        )
        print(f"Sampled reachable base_link XY intervals (z={z:.3f} m; grid step {self.map_step:.2f} m):")
        found_any = False
        for x, row in reachable_rows:
            intervals = []
            iy = 0
            while iy < len(row):
                if not row[iy]:
                    iy += 1
                    continue
                start = iy
                while iy + 1 < len(row) and row[iy + 1]:
                    iy += 1
                end = iy
                y_start = self.map_min + start * self.map_step
                y_end = self.map_min + end * self.map_step
                intervals.append((y_start, y_end))
                iy += 1
            if intervals:
                found_any = True
                text = ", ".join(f"[{lo:+.2f}, {hi:+.2f}]" for lo, hi in intervals)
                print(f"  x={x:+.2f} m -> y approximately in {text} m")
        if not found_any:
            print("  No IK solutions were found in the scanned area at this height and current tool0 orientation.")
        print(
            "In RViz, add a MarkerArray display and set its topic to "
            "/ur3_base_coordinate_pick/workspace_map; set Fixed Frame to "
            f"{self.base_frame}. This map holds the current tool0 orientation fixed and does not indicate collision-free paths."
        )

    def plan_pick_and_return(self, xyz):
        if "up" not in self.planning.named_target_states:
            print("Outbound motion skipped: named state 'up' was not found in the MoveIt SRDF.")
            return None, None

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
        # Keep the tool's current orientation; only XYZ is specified by the user.
        tool_tf = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.tool_link,
            Time(),
            timeout=Duration(seconds=3.0),
        ).transform
        current_xyz = (
            tool_tf.translation.x,
            tool_tf.translation.y,
            tool_tf.translation.z,
        )
        distance = math.sqrt(sum((xyz[i] - current_xyz[i]) ** 2 for i in range(3)))
        print(f"Current tool0 position ({self.base_frame}, m): {current_xyz}")
        print(f"Straight-line distance from current position to target: {distance:.3f} m")
        pose.pose.orientation = tool_tf.rotation

        self.planning.set_start_state_to_current_state()
        self.planning.set_goal_state(pose_stamped_msg=pose, pose_link=self.tool_link)
        self.node.get_logger().info(f"Planning outbound path to tool0 target {xyz}")
        target_plan = self.planning.plan()
        if not target_plan:
            print(f"Outbound path unavailable: no feasible path to tool0 target {xyz} m; the robot will not move.")
            return None, None
        self.publish_target_and_paths(xyz, target_plan, None)

        # Preflight the return from the planned target joint state, before moving.
        target_trajectory = target_plan.trajectory
        if len(target_trajectory) == 0:
            print("The outbound plan contains no trajectory points; the robot will not move.")
            return None, None
        target_end_state = target_trajectory[len(target_trajectory) - 1]
        self.planning.set_start_state(robot_state=target_end_state)
        self.planning.set_goal_state(configuration_name="up")
        self.node.get_logger().info("Planning return path from target to named state 'up'")
        return_plan = self.planning.plan()
        if not return_plan:
            print("Return path unavailable: no path from the target to named state 'up'; the robot will not move.")
            return target_plan, None

        print(f"Outbound path is available for tool0 target {xyz} m.")
        print("Return path is available: target -> named state 'up'.")
        self.publish_target_and_paths(xyz, target_plan, return_plan)
        return target_plan, return_plan

    def trajectory_tool_points(self, plan):
        points = []
        trajectory = plan.trajectory
        for index in range(len(trajectory)):
            state = trajectory[index]
            base_transform = np.asarray(
                state.get_global_link_transform(self.base_frame), dtype=float
            )
            tool_transform = np.asarray(
                state.get_global_link_transform(self.tool_link), dtype=float
            )
            base_to_tool = np.linalg.inv(base_transform) @ tool_transform
            point = Point()
            point.x = float(base_to_tool[0, 3])
            point.y = float(base_to_tool[1, 3])
            point.z = float(base_to_tool[2, 3])
            points.append(point)
        return points

    def publish_target_and_paths(self, xyz, target_plan, return_plan):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        stamp = self.node.get_clock().now().to_msg()
        target = Marker()
        target.header.frame_id = self.base_frame
        target.header.stamp = stamp
        target.ns = "pick_target"
        target.id = 0
        target.type = Marker.SPHERE
        target.action = Marker.ADD
        target.pose.position.x, target.pose.position.y, target.pose.position.z = xyz
        target.pose.orientation.w = 1.0
        target.scale.x = target.scale.y = target.scale.z = 0.07
        target.color.r, target.color.g, target.color.b, target.color.a = (1.0, 0.15, 0.05, 1.0)
        markers.markers.append(target)

        label = Marker()
        label.header.frame_id = self.base_frame
        label.header.stamp = stamp
        label.ns = "pick_target_label"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = xyz[0]
        label.pose.position.y = xyz[1]
        label.pose.position.z = xyz[2] + 0.08
        label.pose.orientation.w = 1.0
        label.scale.z = 0.04
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f"tool0 target ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) m"
        markers.markers.append(label)

        for marker_id, plan, marker_name, color in (
            (2, target_plan, "outbound_path", (0.1, 0.75, 1.0, 1.0)),
            (3, return_plan, "return_path_to_up", (1.0, 0.75, 0.1, 1.0)),
        ):
            if plan is None:
                continue
            line = Marker()
            line.header.frame_id = self.base_frame
            line.header.stamp = stamp
            line.ns = marker_name
            line.id = marker_id
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.008
            line.color.r, line.color.g, line.color.b, line.color.a = color
            line.points = self.trajectory_tool_points(plan)
            markers.markers.append(line)

        self.target_path_pub.publish(markers)
        print(
            "Published target and planned paths to the RViz MarkerArray: "
            "/ur3_base_coordinate_pick/target_and_path_markers"
        )

    def execute_plan(self, plan, label):
        result = self.robot.execute(plan.trajectory, controllers=[])
        error_code = getattr(result, "val", None)
        if result is False or (error_code is not None and error_code != 1):
            print(f"{label} execution failed (MoveIt error code: {error_code}).")
            return False
        return True

    def pick_return_home(self, target_plan, return_plan):
        if target_plan is None or return_plan is None:
            return
        print("Outbound and return paths to 'up' are planned. Check the paths in RViz and the surrounding space before proceeding.")
        if input("Confirm and start the automatic pick sequence? Enter y: ").strip().lower() != "y":
            print("Canceled; the robot arm and gripper will not move.")
            return
        if self.gripper_backend == "tcp":
            self.gripper = RobotiqGripper(self.robot_ip, self.gripper_port)
        try:
            if self.gripper_backend == "tcp":
                self.gripper.activate()
                # Start the sequence with the gripper closed.
                self.gripper.move_and_wait(255)
                print("Gripper closed before outbound motion.")

            print("Both outbound and return paths were planned in advance. Starting the automatic pick sequence.")
            if not self.execute_plan(target_plan, "Outbound"):
                print("Outbound motion did not complete; skipping grasp and return.")
                return

            if self.gripper_backend == "tcp":
                print("Target reached. Opening the gripper.")
                self.gripper.move_and_wait(0)
                print("Gripper open. Waiting 0.5 seconds before closing.")
                time.sleep(0.5)
                self.gripper.move_and_wait(255)
                print("Gripper closed. Waiting 0.5 seconds before returning to 'up'.")
                time.sleep(0.5)
            else:
                print("gripper_backend=none: skipping all gripper commands.")

            if not self.execute_plan(return_plan, "Return"):
                print("Failed to return to 'up'; the gripper remains closed. Check the robot state.")
                return

            if self.gripper_backend == "tcp":
                print("Returned to 'up'; gripper remains closed.")
            else:
                print("Returned to 'up'; gripper_backend=none, gripper commands were skipped.")
        finally:
            if self.gripper:
                self.gripper.close()
                self.gripper = None

    def print_current_tool_position(self):
        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.tool_link,
            Time(),
            timeout=Duration(seconds=3.0),
        ).transform
        print(
            f"Current {self.tool_link} position ({self.base_frame}, m): "
            f"({transform.translation.x:.4f}, {transform.translation.y:.4f}, "
            f"{transform.translation.z:.4f})"
        )
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "current_tool"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = transform.translation.x
        marker.pose.position.y = transform.translation.y
        marker.pose.position.z = transform.translation.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.06
        marker.color.r = 0.1
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker_array.markers.append(marker)

        label = Marker()
        label.header = marker.header
        label.ns = "current_tool_label"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = transform.translation.x
        label.pose.position.y = transform.translation.y
        label.pose.position.z = transform.translation.z + 0.07
        label.pose.orientation.w = 1.0
        label.scale.z = 0.04
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = (
            f"tool0 ({transform.translation.x:.3f}, "
            f"{transform.translation.y:.3f}, {transform.translation.z:.3f}) m"
        )
        marker_array.markers.append(label)
        self.map_pub.publish(marker_array)

    def run(self):
        if not self.scene_ready:
            print("Pick program stopped: MoveIt has no shared virtual table collision slab.")
            return
        print(
            f"Enter {self.base_frame} coordinates x y z in meters to pick and return to 'up'; "
            "enter r to print the current tool0 position; enter m to show the workspace slice at a selected height; "
            "enter q to quit."
        )
        while rclpy.ok():
            try:
                self.print_current_tool_position()
            except TransformException as exc:
                print(f"Could not get the TF for the current {self.tool_link} position: {exc}")
            raw = input(f"{self.base_frame} x y z> ").strip()
            if raw.lower() in {"q", "quit", "exit"}:
                break
            if raw.lower() == "r":
                try:
                    self.print_current_tool_position()
                except TransformException as exc:
                    print(f"Could not get the current position: {exc}")
                continue
            if raw.lower() == "m":
                try:
                    z = float(input("Slice height z (base_link, meters)> ").strip())
                    if not math.isfinite(z) or abs(z) > 2.0:
                        raise ValueError("z must be a finite value between -2 and 2 meters")
                    self.show_workspace_slice(z)
                except Exception as exc:
                    print(f"Could not generate the workspace slice: {exc}")
                continue
            try:
                xyz = tuple(float(value) for value in raw.replace(",", " ").split())
                if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
                    raise ValueError("Enter three finite numeric values for x y z")
                if any(abs(v) > 2.0 for v in xyz):
                    raise ValueError("Coordinate magnitudes must be at most 2 meters; check the units and coordinate frame")
            except ValueError as exc:
                print(f"Invalid input: {exc}")
                continue

            try:
                target_plan, return_plan = self.plan_pick_and_return(xyz)
                self.pick_return_home(target_plan, return_plan)
            except (OSError, RuntimeError, TimeoutError, TransformException) as exc:
                self.node.get_logger().error(f"Operation failed: {exc}")

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
        app = BaseCoordinatePick()
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        if app:
            app.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

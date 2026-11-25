#!/usr/bin/env python3
"""Capture a live camera cloud and execute a confirmed highest-score AnyGrasp task."""

import socket
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory


DISPLAY_FRAME = "camera_color_optical_frame"


def optical_xyz(app, xyz, source_frame=None):
    """Convert displayed positions, independently of the planner/input frame."""
    source_frame = source_frame or app.base_frame
    if source_frame == DISPLAY_FRAME:
        return tuple(float(v) for v in xyz)
    tf = app.tf_buffer.lookup_transform(
        DISPLAY_FRAME, source_frame, Time(), timeout=Duration(seconds=3.0)
    ).transform
    rotated = rotate_vector(tf.rotation, xyz)
    return tuple(float(rotated[i] + getattr(tf.translation, axis))
                 for i, axis in enumerate("xyz"))


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
            if self.get("PRE") == position:
                status = self.get("OBJ")
                if status in (1, 2):
                    return status
                if status == 3:
                    return status
            time.sleep(0.05)
        raise TimeoutError(
            f"Robotiq movement timeout: target={position}, PRE={self.get('PRE')}, "
            f"POS={self.get('POS')}, OBJ={self.get('OBJ')}, "
            f"STA={self.get('STA')}, FLT={self.get('FLT')}"
        )


class PickUp:
    def __init__(self):
        self.node = Node("ur3_pick_up")
        from .live_input import validate_settings
        from .anygrasp_input import load_tool_config
        defaults = {
            "base_frame": "base_link", "tool_link": "tool0", "tcp_offset": 0.174,
            "planning_group": "ur_manipulator", "robot_ip": "192.168.86.7",
            "gripper_port": 63352, "gripper_backend": "tcp",
            "max_velocity_scaling_factor": 0.1, "max_acceleration_scaling_factor": 0.1,
            "grasp_tool_config": str(Path(get_package_share_directory("ur3_pick_up"))
                                     / "config/grasp_tool.example.json"),
            "pointcloud_topic": "/camera/camera/depth/color/points",
            "color_image_topic": "/camera/camera/color/image_raw",
            "color_camera_info_topic": "/camera/camera/color/camera_info",
            "cloud_timeout": 10.0, "cloud_max_age": 1.0,
            "voxel_size": 0.005, "max_points": 120000, "max_camera_depth": 2.0,
            "min_roi_points": 50,
            "docker_container": "anygrasp-fixed", "docker_use_sudo": True,
            "docker_sdk_dir": "/opt/anygrasp_sdk/grasp_detection",
            "checkpoint_path": "/opt/anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar",
            "inference_timeout": 120.0, "gripper_height": 0.03,
            "exchange_dir": str(Path.home() / "anygrasp_docker/exchange"),
            "plan_only": False,
        }
        for name, default in defaults.items():
            self.node.declare_parameter(name, default)
        self.settings = {name: self.node.get_parameter(name).value for name in defaults}
        validate_settings(self.settings)
        self.base_frame = self.settings["base_frame"]
        self.input_frame = DISPLAY_FRAME
        self.tool_link = self.settings["tool_link"]
        self.tcp_offset = float(self.settings["tcp_offset"])
        self.group = self.settings["planning_group"]
        self.robot_ip = self.settings["robot_ip"]
        self.gripper_port = self.settings["gripper_port"]
        self.gripper_backend = self.settings["gripper_backend"]
        self.velocity_scaling = self.settings["max_velocity_scaling_factor"]
        self.acceleration_scaling = self.settings["max_acceleration_scaling_factor"]
        self.tip_from_tool, self.max_opening = load_tool_config(
            self.settings["grasp_tool_config"], self.tool_link, self.tcp_offset)
        if self.max_opening > 0.1:
            raise ValueError("AnyGrasp max opening must be <= 0.1 m")
        self.gripper = None
        self.last_markers = None
        self.snapshot_base_from_camera = None
        self.start_state = None

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
            "planning_attempts": 10,
            "max_velocity_scaling_factor": self.velocity_scaling,
            "max_acceleration_scaling_factor": self.acceleration_scaling,
        }
        self.robot = MoveItPy(
            node_name="ur3_pick_up_moveit", config_dict=moveit_parameters
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
        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_path_pub = self.node.create_publisher(
            MarkerArray, "~/target_and_path_markers", marker_qos)
        # Also feed the existing bringup RViz target/path display. Periodic
        # publication supports its volatile subscription and late subscribers.
        self.marker_timer = self.node.create_timer(1.0, self.refresh_markers)

    def refresh_markers(self):
        if self.last_markers is not None:
            self.target_path_pub.publish(self.last_markers)

    def verify_snapshot(self):
        from .anygrasp_input import matrix_from_transform
        from .live_input import verify_fixed_camera
        latest = self.tf_buffer.lookup_transform(
            self.base_frame, DISPLAY_FRAME, Time(), timeout=Duration(seconds=3.0))
        verify_fixed_camera(self.snapshot_base_from_camera, matrix_from_transform(latest.transform))

    def plan_pick_and_return(self, xyz, *, target_orientation):
        if self.start_state is None:
            print("Planning skipped: the arm's startup joint state was not captured.")
            return None, None
        return_label = "the recorded startup position"

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
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
        current_tcp = np.array(current_xyz) + rotate_vector(tool_tf.rotation, [0., 0., self.tcp_offset])
        target_tcp = np.array(xyz) + rotate_vector(target_orientation, [0., 0., self.tcp_offset])
        distance = float(np.linalg.norm(target_tcp - current_tcp))
        print(f"Current TCP position ({DISPLAY_FRAME}, m): {optical_xyz(self, current_tcp)}")
        print(f"Straight-line distance from current position to target: {distance:.3f} m")
        pose.pose.orientation = target_orientation

        self.planning.set_start_state(robot_state=self.start_state)
        self.planning.set_goal_state(pose_stamped_msg=pose, pose_link=self.tool_link)
        self.node.get_logger().info(f"Planning outbound path to TCP target ({DISPLAY_FRAME}) {optical_xyz(self, target_tcp)}")
        try:
            target_plan = self.planning.plan()
        except RuntimeError as exc:
            print(f"Outbound planning failed for this pose: {exc}")
            return None, None
        if not target_plan:
            print(f"Outbound planning failed: cannot reach TCP target {optical_xyz(self, target_tcp)} m in {DISPLAY_FRAME}.")
            return None, None
        # Preflight the return from the planned target joint state, before moving.
        target_trajectory = target_plan.trajectory
        if len(target_trajectory) == 0:
            print("Outbound planning failed: the trajectory has no waypoints.")
            return None, None
        self.publish_target_and_paths(target_plan, None)
        target_end_state = target_trajectory[len(target_trajectory) - 1]
        self.planning.set_start_state(robot_state=target_end_state)
        self.planning.set_goal_state(robot_state=self.start_state)
        self.node.get_logger().info(f"Planning return path from target to {return_label}")
        try:
            return_plan = self.planning.plan()
        except RuntimeError as exc:
            print(f"Return planning failed for this pose (target: {return_label}): {exc}")
            return target_plan, None
        if not return_plan or len(return_plan.trajectory) == 0:
            print(f"Return planning failed: cannot return from this pose to {return_label}.")
            return target_plan, None

        print(f"Outbound path is available for TCP target ({DISPLAY_FRAME}) {optical_xyz(self, target_tcp)} m.")
        print(f"Return path is available: target -> {return_label}.")
        self.publish_target_and_paths(target_plan, return_plan)
        return target_plan, return_plan

    def trajectory_tcp_points(self, plan):
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
            position = base_to_tool[:3, 3].copy()
            position += base_to_tool[:3, :3] @ np.array([0.0, 0.0, self.tcp_offset])
            point = Point()
            point.x, point.y, point.z = map(float, position)
            points.append(point)
        return points

    def publish_target_and_paths(self, target_plan, return_plan):
        # The final trajectory state supplies the orientation for both home
        # and grasp markers, avoiding stale candidate axes on a home target.
        from .anygrasp_input import quaternion_from_matrix
        from geometry_msgs.msg import Quaternion
        state = target_plan.trajectory[len(target_plan.trajectory) - 1]
        base_to_tool = (np.linalg.inv(np.asarray(state.get_global_link_transform(self.base_frame)))
                        @ np.asarray(state.get_global_link_transform(self.tool_link)))
        xyz = base_to_tool[:3, 3] + base_to_tool[:3, :3] @ [0.0, 0.0, self.tcp_offset]
        q = quaternion_from_matrix(base_to_tool)
        orientation = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.insert(0, clear)

        stamp = self.node.get_clock().now().to_msg()
        target = Marker()
        target.header.frame_id = self.base_frame
        target.header.stamp = stamp
        target.ns = "tcp_target"
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
        label.ns = "tcp_target_label"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = xyz[0]
        label.pose.position.y = xyz[1]
        label.pose.position.z = xyz[2] + 0.08
        label.pose.orientation.w = 1.0
        label.scale.z = 0.04
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        display = optical_xyz(self, xyz)
        label.text = f"TCP ({DISPLAY_FRAME}) ({display[0]:.3f}, {display[1]:.3f}, {display[2]:.3f}) m"
        markers.markers.append(label)

        for index, axis in enumerate(((0.08, 0., 0.), (0., 0.08, 0.), (0., 0., 0.08))):
            arrow = Marker()
            arrow.header = target.header
            arrow.ns = "tcp_axes"
            arrow.id = 10 + index
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.orientation.w = 1.0
            direction = rotate_vector(orientation, axis)
            arrow.points = [Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])),
                            Point(x=float(xyz[0] + direction[0]),
                                  y=float(xyz[1] + direction[1]), z=float(xyz[2] + direction[2]))]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.006, 0.012, 0.015
            arrow.color.r = 1.0 if index == 0 else 0.0
            arrow.color.g = 1.0 if index == 1 else 0.0
            arrow.color.b = 1.0 if index == 2 else 0.0
            arrow.color.a = 1.0
            markers.markers.append(arrow)

        for marker_id, plan, marker_name, color in (
            (2, target_plan, "outbound_path", (0.1, 0.75, 1.0, 1.0)),
            (3, return_plan, "return_path", (1.0, 0.75, 0.1, 1.0)),
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
            line.points = self.trajectory_tcp_points(plan)
            markers.markers.append(line)

        self.last_markers = markers
        self.target_path_pub.publish(markers)
        print(
            "Published target and planned paths to the RViz MarkerArray: "
            "/ur3_pick_up/target_and_path_markers"
        )

    def execute_plan(self, plan, label):
        result = self.robot.execute(plan.trajectory, controllers=[])
        error_code = getattr(result, "val", None)
        if not result or (error_code is not None and error_code != 1):
            print(f"{label} execution failed (MoveIt error code: {error_code}).")
            return False
        return True


    def capture_start_state(self):
        """Record the live joint state at startup as the task's start and return state."""
        self.planning.set_start_state_to_current_state()
        try:
            self.start_state = self.planning.get_start_state()
        except RuntimeError as exc:
            print(f"Could not record the arm's startup position: {exc}.")
            return False
        if self.start_state is None:
            print("Could not record the arm's startup position from MoveIt.")
            return False
        print("Recorded the arm's current joint position as the task start and return target.")
        return True

    def run(self):
        if not self.scene_ready:
            print("Scene check failed: the virtual table was not received. The task will not run.")
            return False
        if not self.capture_start_state():
            return False
        from .anygrasp_task import run_anygrasp
        return run_anygrasp(self)

    def shutdown(self):
        if self.gripper:
            self.gripper.close()
        self.robot.shutdown()
        self.tf_listener.unregister()
        self.node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    app = None
    exit_code = 1
    try:
        app = PickUp()
        exit_code = 0 if app.run() else 1
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, TransformException) as exc:
        print(f"Startup/configuration failed: {exc}")
        exit_code = 2
    except (KeyboardInterrupt, EOFError):
        print("Canceled by user or closed input.")
        exit_code = 130
    finally:
        try:
            if app:
                app.shutdown()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

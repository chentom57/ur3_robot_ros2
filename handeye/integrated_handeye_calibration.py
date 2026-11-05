#!/usr/bin/env python3
"""Integrated UR3 hand-eye calibration workflow."""

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from easy_handeye2_msgs.srv import ComputeCalibration, SaveCalibration, TakeSample
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def quat_multiply(a, b):
    """Multiply ROS-order quaternions (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def axis_angle_quaternion(axis, angle):
    half = angle / 2.0
    scale = math.sin(half)
    return (axis[0] * scale, axis[1] * scale, axis[2] * scale, math.cos(half))


class CalibrationReadiness(Node):
    """Monitor marker detection and wait for calibration services and TFs."""

    def __init__(self):
        super().__init__("integrated_handeye_readiness")
        self.service_clients = {
            "take_sample": self.create_client(
                TakeSample, "/easy_handeye2/calibration/take_sample"
            ),
            "get_sample_list": self.create_client(
                TakeSample, "/easy_handeye2/calibration/get_sample_list"
            ),
            "compute_calibration": self.create_client(
                ComputeCalibration, "/easy_handeye2/calibration/compute_calibration"
            ),
            "save_calibration": self.create_client(
                SaveCalibration, "/easy_handeye2/calibration/save_calibration"
            ),
        }
        self.diagnostic_clients = {
            "compute_ik": self.create_client(GetPositionIK, "/compute_ik"),
            "check_state_validity": self.create_client(
                GetStateValidity, "/check_state_validity"
            ),
        }
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def wait_for_marker(self, tracking_base, marker_frame, processes):
        """Wait indefinitely for the tracker to publish the requested marker TF."""
        last_report = 0.0
        while rclpy.ok():
            for label, process in processes:
                if process.poll() is not None:
                    raise RuntimeError(
                        f"{label} process exited early with code {process.returncode}"
                    )
            if self.tf_buffer.can_transform(tracking_base, marker_frame, Time()):
                print(
                    f"Detected marker: {tracking_base} -> {marker_frame}",
                    flush=True,
                )
                return

            now = time.monotonic()
            if now - last_report >= 5.0:
                print(
                    f"Marker not detected yet: {tracking_base} -> {marker_frame}. "
                    "Waiting. Check the camera image, marker ID, and camera view.",
                    flush=True,
                )
                last_report = now
            rclpy.spin_once(self, timeout_sec=0.1)

        raise RuntimeError("ROS 2 shut down while waiting for the ArUco marker")

    def wait_until_ready(self, base_frame, tool_link, tracking_base, marker_frame,
                         timeout_sec, processes):
        deadline = time.monotonic() + timeout_sec
        last_report = 0.0

        while rclpy.ok() and time.monotonic() < deadline:
            for label, process in processes:
                if process.poll() is not None:
                    raise RuntimeError(
                        f"{label} process exited early with code {process.returncode}"
                    )

            services_ready = all(
                client.wait_for_service(timeout_sec=0.0)
                for client in self.service_clients.values()
            )
            robot_tf_ready = self.tf_buffer.can_transform(base_frame, tool_link, Time())
            marker_tf_ready = self.tf_buffer.can_transform(
                tracking_base, marker_frame, Time()
            )
            if services_ready and robot_tf_ready and marker_tf_ready:
                self.get_logger().info(
                    "Calibration services, robot TF, and ArUco marker TF are ready."
                )
                return

            now = time.monotonic()
            if now - last_report >= 5.0:
                waiting = []
                if not services_ready:
                    waiting.append("Easy HandEye services")
                if not robot_tf_ready:
                    waiting.append(f"TF {base_frame} -> {tool_link}")
                if not marker_tf_ready:
                    waiting.append(
                        f"marker TF {tracking_base} -> {marker_frame} (make the marker visible)"
                    )
                self.get_logger().info("Waiting for: " + "; ".join(waiting))
                last_report = now
            rclpy.spin_once(self, timeout_sec=0.1)

        raise TimeoutError(
            f"Calibration setup was not ready within {timeout_sec:.0f} seconds. "
            "Check the camera image, marker, UR driver, and frame names."
        )

    def call_service(self, client, request, label, timeout_sec=None):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise TimeoutError(f"Timed out waiting for {label} service")
        if future.exception():
            raise RuntimeError(f"{label} service failed: {future.exception()}")
        return future.result()

    def diagnose_goal(self, goal, tool_link, group_name):
        """Check whether a failed pose has an IK solution and a collision-free state."""
        ik_client = self.diagnostic_clients["compute_ik"]
        validity_client = self.diagnostic_clients["check_state_validity"]
        for service_name, client in (
            ("/compute_ik", ik_client),
            ("/check_state_validity", validity_client),
        ):
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warning(
                    f"Planning diagnostic unavailable: {service_name} is not ready."
                )
                return

        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = group_name
        ik_request.ik_request.ik_link_name = tool_link
        ik_request.ik_request.pose_stamped = goal
        ik_request.ik_request.avoid_collisions = False
        ik_request.ik_request.timeout.sec = 2
        ik_result = self.call_service(
            ik_client, ik_request, "inverse kinematics diagnostic", timeout_sec=4.0
        )
        error_code = ik_result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            if error_code == MoveItErrorCodes.NO_IK_SOLUTION:
                detail = "No IK solution was found; the target may be unreachable."
            else:
                detail = f"IK returned MoveIt error code {error_code}."
            self.get_logger().error("Planning diagnosis: " + detail)
            return

        validity_request = GetStateValidity.Request()
        validity_request.robot_state = ik_result.solution
        validity_request.group_name = group_name
        validity_result = self.call_service(
            validity_client, validity_request, "collision diagnostic", timeout_sec=4.0
        )
        joint_state = ik_result.solution.joint_state
        solution = ", ".join(
            f"{name}={value:.3f}"
            for name, value in zip(joint_state.name, joint_state.position)
        )
        if validity_result.valid:
            self.get_logger().warning(
                "Planning diagnosis: IK found a collision-free goal state, but OMPL "
                "could not find a path from the current state. "
                f"IK joints: {solution}"
            )
            return

        contacts = [
            f"{contact.contact_body_1} vs {contact.contact_body_2}"
            for contact in validity_result.contacts[:5]
        ]
        contact_text = "; ".join(contacts) if contacts else "contact details unavailable"
        self.get_logger().error(
            "Planning diagnosis: IK found a goal state, but it is in collision or "
            f"invalid. Contacts: {contact_text}. IK joints: {solution}"
        )


class AutomaticCalibration:
    """Move the UR3 and acquire, compute, and save calibration samples."""

    def __init__(self, base_frame="base_link", tool_link="tool0"):
        self.node = Node("ur3_integrated_auto_handeye")
        self.base_frame = base_frame
        self.tool_link = tool_link
        self.rotation_delta_degrees = 20.0
        self.translation_delta_meters = 0.04
        self.settle_seconds = 1.5
        self.max_planning_retries = 3

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=True)
        moveit_config = (
            MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
            .robot_description_semantic(
                file_path=Path("srdf") / "ur.srdf.xacro", mappings={"name": "ur3"}
            )
            .to_moveit_configs()
        )
        moveit_parameters = moveit_config.to_dict()
        moveit_parameters["planning_pipelines"] = {"pipeline_names": ["ompl"]}
        moveit_parameters["plan_request_params"] = {
            "planning_pipeline": "ompl",
            "planner_id": "RRTConnectkConfigDefault",
            "planning_time": 5.0,
            "planning_attempts": 5,
            "max_velocity_scaling_factor": 0.2,
            "max_acceleration_scaling_factor": 0.2,
        }
        self.robot = MoveItPy(
            node_name="ur3_integrated_handeye_moveit", config_dict=moveit_parameters
        )
        self.planning = self.robot.get_planning_component("ur_manipulator")

    def current_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.tool_link,
            Time(),
            timeout=Duration(seconds=5.0),
        ).transform
        return (
            (transform.translation.x, transform.translation.y, transform.translation.z),
            (transform.rotation.x, transform.rotation.y,
             transform.rotation.z, transform.rotation.w),
        )

    def make_targets(self, position, orientation):
        """Create 12 wrist rotations and 3 small XYZ translations."""
        delta = math.radians(self.rotation_delta_degrees)
        axes = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
        rotations = []
        for degrees in (delta, -delta, delta / 2, -delta / 2):
            for axis in axes:
                rotations.append(
                    quat_multiply(orientation, axis_angle_quaternion(axis, degrees))
                )

        offset = self.translation_delta_meters
        translations = (
            (position[0] + offset, position[1], position[2]),
            (position[0], position[1] + offset, position[2]),
            (position[0], position[1], position[2] + offset),
        )
        return ([(position, q) for q in rotations]
                + [(p, orientation) for p in translations])

    def plan_and_execute(self, position, orientation, label, readiness):
        goal = PoseStamped()
        goal.header.frame_id = self.base_frame
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = position
        (goal.pose.orientation.x, goal.pose.orientation.y,
         goal.pose.orientation.z, goal.pose.orientation.w) = orientation
        start_position, start_orientation = self.current_pose()
        self.node.get_logger().info(
            f"Pose diagnostic for target {label}: "
            f"start position={tuple(round(v, 4) for v in start_position)}, "
            f"start quaternion={tuple(round(v, 4) for v in start_orientation)}; "
            f"goal position={tuple(round(v, 4) for v in position)}, "
            f"goal quaternion={tuple(round(v, 4) for v in orientation)}"
        )
        result = None
        for attempt in range(1, self.max_planning_retries + 1):
            self.planning.set_start_state_to_current_state()
            self.planning.set_goal_state(
                pose_stamped_msg=goal, pose_link=self.tool_link
            )
            result = self.planning.plan()
            if result:
                break
            self.node.get_logger().warning(
                f"Planning target {label} failed (attempt {attempt}/"
                f"{self.max_planning_retries}); retrying."
            )
            if attempt < self.max_planning_retries:
                time.sleep(0.25)
        if not result:
            try:
                readiness.diagnose_goal(goal, self.tool_link, "ur_manipulator")
            except Exception as exc:
                self.node.get_logger().error(
                    f"Planning diagnostics could not complete: {exc}"
                )
            raise RuntimeError(
                f"MoveIt could not plan target {label} after "
                f"{self.max_planning_retries} attempts. The trajectory was not executed. "
                "Review the pose and planning diagnosis above."
            )
        self.robot.execute(result.trajectory, controllers=[])

    @staticmethod
    def print_calibration(calibration):
        parameters = calibration.parameters
        translation = calibration.transform.translation
        rotation = calibration.transform.rotation
        if parameters.calibration_type == "eye_in_hand":
            parent, child = (
                parameters.robot_effector_frame, parameters.tracking_base_frame
            )
        else:
            parent, child = parameters.robot_base_frame, parameters.tracking_base_frame

        print("\n========== Easy HandEye 2 Calibration Result ==========")
        print(f"Calibration type: {parameters.calibration_type}")
        print(f"TF: {parent} -> {child}")
        print(
            "translation [m]: "
            f"x={translation.x:.9f}, y={translation.y:.9f}, z={translation.z:.9f}"
        )
        print(
            "rotation quaternion [x, y, z, w]: "
            f"[{rotation.x:.9f}, {rotation.y:.9f}, "
            f"{rotation.z:.9f}, {rotation.w:.9f}]"
        )
        x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
        matrix = (
            (1 - 2 * (y*y + z*z), 2 * (x*y - z*w),
             2 * (x*z + y*w), translation.x),
            (2 * (x*y + z*w), 1 - 2 * (x*x + z*z),
             2 * (y*z - x*w), translation.y),
            (2 * (x*z - y*w), 2 * (y*z + x*w),
             1 - 2 * (x*x + y*y), translation.z),
            (0.0, 0.0, 0.0, 1.0),
        )
        print("matrix [R | t]:")
        for row in matrix:
            print("  [" + ", ".join(f"{value:.9f}" for value in row) + "]")
        print("==============================================\n")

    def calibrate(self, readiness):
        for key in ("take_sample", "get_sample_list", "compute_calibration",
                    "save_calibration"):
            client = readiness.service_clients[key]
            while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info(f"Waiting for Easy HandEye 2 {key} service...")
            if not rclpy.ok():
                raise RuntimeError("ROS 2 shut down")

        sample_client = readiness.service_clients["take_sample"]
        samples_client = readiness.service_clients["get_sample_list"]
        existing = readiness.call_service(
            samples_client, TakeSample.Request(), "read existing samples"
        )
        if existing.samples.samples:
            raise RuntimeError(
                f"Easy HandEye 2 already has {len(existing.samples.samples)} samples. "
                "Clear the old samples in the calibration interface before restarting."
            )

        start_position, start_orientation = self.current_pose()
        targets = self.make_targets(start_position, start_orientation)
        print(f"Starting tool pose ({self.base_frame}): {start_position}", flush=True)
        print("Moving to and capturing 15 calibration samples...", flush=True)

        for index, (position, orientation) in enumerate(targets, start=1):
            print(f"\nPose {index}/15: planning and moving...", flush=True)
            self.plan_and_execute(position, orientation, index, readiness)
            time.sleep(self.settle_seconds)
            response = readiness.call_service(
                sample_client, TakeSample.Request(), "take sample"
            )
            sample_count = len(response.samples.samples)
            if sample_count != index:
                raise RuntimeError(
                    f"Easy HandEye 2 did not record sample {index} "
                    f"(current sample count: {sample_count})"
                )
            print(f"Captured sample {sample_count}/15", flush=True)

        print("\nReturning to the starting pose...", flush=True)
        self.plan_and_execute(start_position, start_orientation, "return", readiness)
        result = readiness.call_service(
            readiness.service_clients["compute_calibration"],
            ComputeCalibration.Request(), "compute calibration",
        )
        if not result.valid:
            raise RuntimeError("Easy HandEye 2 could not compute a valid calibration")
        self.print_calibration(result.calibration)
        save = readiness.call_service(
            readiness.service_clients["save_calibration"],
            SaveCalibration.Request(), "save calibration",
        )
        if not save.success:
            raise RuntimeError("Calibration computed, but Easy HandEye 2 could not save it")
        print(f"Calibration saved: {save.filepath.data}", flush=True)

    def shutdown(self):
        self.robot.shutdown()
        self.tf_listener.unregister()
        self.node.destroy_node()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Wait for an ArUco marker, then launch Easy HandEye 2 and calibrate."
    )
    parser.add_argument("--calibration-type", choices=("eye_on_base", "eye_in_hand"),
                        default="eye_on_base")
    parser.add_argument("--name", default="ur3_realsense_eob")
    parser.add_argument("--robot-base-frame", default="base_link")
    parser.add_argument("--robot-effector-frame", default="tool0")
    parser.add_argument("--tracking-base-frame", default="camera_color_optical_frame")
    parser.add_argument("--tracking-marker-frame", default="marker_582")
    parser.add_argument("--image-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--marker-size", type=float, default=0.04)
    parser.add_argument("--marker-dict", default="ARUCO_ORIGINAL")
    parser.add_argument("--startup-timeout", type=float, default=180.0,
                        help="Maximum wait for services and TFs in seconds (default: 180)")
    # Run MoveItPy in an isolated process so its C++ objects are released
    # separately from the long-lived tracker and calibration launch.
    parser.add_argument("--calibrate-only", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args()


def terminate_processes(processes):
    for _, process in reversed(processes):
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    for label, process in reversed(processes):
        if process.poll() is None:
            try:
                process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                print(f"{label} did not stop cleanly; forcing shutdown.", file=sys.stderr)
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()


def ask_yes_no(prompt):
    while True:
        try:
            answer = input(prompt).strip().lower()
        except EOFError:
            return False
        if answer in ("y", "yes"):
            return True
        if answer in ("n", "no"):
            return False
        print("Enter y or n.", flush=True)


def run_and_show(command):
    """Run a child process while forwarding output and retaining its transcript."""
    process = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1, start_new_session=True,
    )
    output = []
    try:
        for line in process.stdout:
            print(line, end="", flush=True)
            output.append(line)
        return_code = process.wait()
    except KeyboardInterrupt:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        process.wait()
        raise
    return return_code, "".join(output)


def run_worker(args):
    """Execute one internal MoveItPy stage using this same script."""
    rclpy.init()
    readiness = None
    motion = None
    result_code = 0
    try:
        readiness = CalibrationReadiness()
        motion = AutomaticCalibration(args.robot_base_frame, args.robot_effector_frame)
        motion.calibrate(readiness)
    except (KeyboardInterrupt, TransformException) as exc:
        print(f"Calibration interrupted: {exc}", file=sys.stderr, flush=True)
        result_code = 130
    except Exception as exc:
        print(f"Automatic calibration failed: {exc}", file=sys.stderr, flush=True)
        result_code = 1
    finally:
        if motion is not None:
            motion.shutdown()
        if readiness is not None:
            readiness.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result_code


def main():
    args = parse_args()
    if args.calibrate_only:
        return run_worker(args)

    processes = []
    readiness = None
    rclpy.init()
    try:
        readiness = CalibrationReadiness()
        integrated_cmd = ["ros2", "run", "easy_handeye2",
                          "integrated_auto_handeye_calibration"]
        tracker_cmd = [
            "ros2", "run", "aruco_opencv", "aruco_tracker_autostart", "--ros-args",
            "-p", f"cam_base_topic:={args.image_topic}",
            "-p", f"marker_size:={args.marker_size}",
            "-p", f"marker_dict:={args.marker_dict}",
        ]
        print("Starting ArUco tracker...", flush=True)
        processes.append(("ArUco tracker",
                          subprocess.Popen(tracker_cmd, start_new_session=True)))

        print("Waiting for ArUco marker detection...", flush=True)
        readiness.wait_for_marker(
            args.tracking_base_frame, args.tracking_marker_frame, processes
        )

        if not ask_yes_no("Marker detected. Start automatic calibration? [y/n]: "):
            print("Calibration declined. Shutting down the ArUco tracker.", flush=True)
            return_code = 0
            return return_code

        calibrate_cmd = [
            "ros2", "launch", "easy_handeye2", "calibrate.launch.py",
            f"calibration_type:={args.calibration_type}",
            f"name:={args.name}",
            f"robot_base_frame:={args.robot_base_frame}",
            f"robot_effector_frame:={args.robot_effector_frame}",
            f"tracking_base_frame:={args.tracking_base_frame}",
            f"tracking_marker_frame:={args.tracking_marker_frame}",
        ]
        print("Starting the Easy HandEye 2 calibration interface...", flush=True)
        processes.append(("Easy HandEye 2",
                          subprocess.Popen(calibrate_cmd, start_new_session=True)))

        readiness.wait_until_ready(
            args.robot_base_frame, args.robot_effector_frame,
            args.tracking_base_frame, args.tracking_marker_frame,
            args.startup_timeout, processes,
        )

        print("Starting automatic calibration from the current robot pose...", flush=True)
        auto_code, auto_output = run_and_show(
            integrated_cmd + ["--calibrate-only",
                              "--robot-base-frame", args.robot_base_frame,
                              "--robot-effector-frame", args.robot_effector_frame]
        )
        calibration_saved = "Calibration saved:" in auto_output
        if not calibration_saved or auto_code not in (0, -11, 245):
            raise RuntimeError(f"Automatic calibration did not complete (exit code {auto_code})")

        terminate_processes(processes)
        processes.clear()
        if ask_yes_no("Calibration saved. Start TF publishing? [y/n]: "):
            publish_cmd = [
                "ros2", "launch", "easy_handeye2", "publish.launch.py",
                f"name:={args.name}",
            ]
            print("Publishing the calibration TF. Press Ctrl+C to stop and exit.", flush=True)
            processes.append(("Calibration TF publisher",
                              subprocess.Popen(publish_cmd, start_new_session=True)))
            publish_code = processes[-1][1].wait()
            if publish_code != 0:
                raise RuntimeError(f"Calibration TF publisher exited with code {publish_code}")
        else:
            print("TF publishing skipped. Workflow complete.", flush=True)
    except KeyboardInterrupt:
        print("Ctrl+C received. Shutting down the calibration workflow...", flush=True)
        return_code = 130
    except Exception as exc:
        print(f"Integrated calibration workflow failed: {exc}", file=sys.stderr, flush=True)
        return_code = 1
    else:
        return_code = 0
    finally:
        if readiness is not None:
            readiness.destroy_node()
        terminate_processes(processes)
        if rclpy.ok():
            rclpy.shutdown()
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())

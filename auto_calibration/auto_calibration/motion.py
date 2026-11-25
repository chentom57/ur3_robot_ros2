"""MoveIt sampling and ROS service clients for automatic hand-eye calibration."""
import math
import shutil
import sys
import time
from pathlib import Path

import rclpy
from easy_handeye2_msgs.srv import ComputeCalibration, SaveCalibration, TakeSample, RemoveSample
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

def wait_for_single_service(node, client, label, timeout_sec=5.0):
    """Wait out ROS discovery changes; check actual service endpoints, not node names."""
    deadline = time.monotonic() + timeout_sec
    while True:
        count = node.count_services(client.srv_name)
        if count == 1:
            return
        if not rclpy.ok() or time.monotonic() >= deadline:
            raise RuntimeError(f"{label}: service {client.srv_name} has {count} providers "
                               f"after {timeout_sec:.1f}s; expected 1. No request sent. "
                               "Check missing or duplicate calibration servers.")
        rclpy.spin_once(node, timeout_sec=0.1)


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
        super().__init__("auto_calibration_readiness")
        self.service_clients = {
            "take_sample": self.create_client(
                TakeSample, "/easy_handeye2/calibration/take_sample"
            ),
            "remove_sample": self.create_client(
                RemoveSample, "/easy_handeye2/calibration/remove_sample"
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
        wait_for_single_service(self, client, label)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise TimeoutError(f"Timed out waiting for {label} service")
        if future.exception():
            raise RuntimeError(f"{label} service failed: {future.exception()}")
        return future.result()

    def clear_samples(self):
        """Remove server samples backwards, verify each response and final empty list."""
        response = self.call_service(self.service_clients["get_sample_list"],
                                     TakeSample.Request(), "read existing samples", timeout_sec=10.0)
        count = len(response.samples.samples)
        original = count
        print(f"Clearing previous samples: {count} samples currently stored.", flush=True)
        while count:
            request = RemoveSample.Request()
            request.sample_index = count-1
            response = self.call_service(self.service_clients["remove_sample"], request,
                                         "remove old sample", timeout_sec=10.0)
            new_count = len(response.samples.samples)
            if new_count != count-1:
                raise RuntimeError(f"Sample reset failed: expected {count-1} remaining samples, got {new_count}. Sampling will not start.")
            count = new_count
        final = self.call_service(self.service_clients["get_sample_list"],
                                  TakeSample.Request(), "verify empty sample list", timeout_sec=10.0)
        if final.samples.samples:
            raise RuntimeError("Samples reappeared after reset. Do not collect or load samples through the GUI concurrently. Sampling has not started.")
        print(f"Cleared {original} previous samples and verified an empty sample list. Starting preflight.", flush=True)

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
        self.node = Node("auto_calibration_motion")
        self.base_frame = base_frame
        self.tool_link = tool_link
        self.rotation_delta_degrees = 35.0
        self.translation_delta_meters = 0.10
        self.settle_seconds = 1.5
        self.inter_pose_pause_seconds = 1.5
        self.max_planning_retries = 10
        self._joint_state_received = False
        self._joint_state_names = ()
        self.joint_state_node = Node("auto_calibration_joint_state_readiness")
        self._joint_state_subscription = self.joint_state_node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=True)
        from moveit_configs_utils import MoveItConfigsBuilder

        moveit_config = (
            MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
            .robot_description_semantic(
                file_path=Path("srdf") / "ur.srdf.xacro", mappings={"name": "ur3"}
            )
            .to_moveit_configs()
        )
        moveit_parameters = moveit_config.to_dict()
        moveit_parameters["planning_pipelines"] = {"pipeline_names": ["ompl"]}
        moveit_parameters.setdefault("ompl", {})["ur_manipulator"] = {
            "planner_configs": ["RRTConnect"]
        }
        moveit_parameters["plan_request_params"] = {
            "planning_pipeline": "ompl",
            "planner_id": "RRTConnect",
            "planning_time": 5.0,
            "planning_attempts": 5,
            "max_velocity_scaling_factor": 0.2,
            "max_acceleration_scaling_factor": 0.2,
        }
        from moveit.planning import MoveItPy

        self.robot = MoveItPy(
            node_name="auto_calibration_moveit", config_dict=moveit_parameters
        )
        self.planning = self.robot.get_planning_component("ur_manipulator")

    def _on_joint_state(self, message):
        if len(message.name) >= 6 and len(message.position) >= 6:
            self._joint_state_received = True
            self._joint_state_names = tuple(message.name)

    def wait_for_joint_states(self, timeout_sec=10.0):
        """Do not start planning until the UR driver is publishing joint states."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not self._joint_state_received and time.monotonic() < deadline:
            rclpy.spin_once(self.joint_state_node, timeout_sec=0.1)
        if not self._joint_state_received:
            raise RuntimeError(
                f"No valid /joint_states message with six joints received within "
                f"{timeout_sec:.0f} seconds. Confirm the UR3 is connected and the "
                "driver is publishing /joint_states before starting calibration. "
                "No motion or sampling has started."
            )
        self.node.get_logger().info(
            "Received robot joint states: " + ", ".join(self._joint_state_names)
        )
        self.joint_state_node.destroy_node()

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
        """Sample single- and two-axis rotations plus base translations."""
        single = 35.0
        yaw = 40.0
        mixed = 25.0
        rotation_offsets = (
            (35.0, 0.0, 0.0), (-35.0, 0.0, 0.0),  # Poses 1-2: tool-X tilt ±35°
            (0.0, single, 0.0), (0.0, -single, 0.0),
            (0.0, 0.0, yaw), (0.0, 0.0, -yaw),
            (mixed, mixed, 0.0), (-mixed, -mixed, 0.0),
            (0.0, mixed, mixed), (0.0, -mixed, -mixed),
            (mixed, 0.0, -mixed), (-mixed, 0.0, mixed),
        )
        axes = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
        rotations = []
        for roll, pitch, yaw in rotation_offsets:
            target = orientation
            for axis, degrees in zip(axes, (roll, pitch, yaw)):
                if degrees:
                    target = quat_multiply(
                        target,
                        axis_angle_quaternion(axis, math.radians(degrees)),
                    )
            rotations.append(target)
        # Add three equal, widely separated tilt directions in the tool X/Y
        # plane. They move the marker plane normal farther from its starting
        # direction while keeping the added excitation balanced around it.
        normal_tilt = math.radians(50.0)
        tilt_axes = (
            (1.0, 0.0, 0.0),
            (-0.5, math.sqrt(3.0) / 2.0, 0.0),
            (-0.5, -math.sqrt(3.0) / 2.0, 0.0),
        )
        for axis in tilt_axes:
            rotations.append(
                quat_multiply(orientation, axis_angle_quaternion(axis, normal_tilt))
            )
        offset = self.translation_delta_meters
        front_back_offset = 0.14
        translations = (
            (position[0] + offset, position[1], position[2]),
            (position[0], position[1] + offset, position[2]),
            (position[0], position[1], position[2] + offset),
            (position[0] - front_back_offset, position[1], position[2]),
            (position[0] + front_back_offset, position[1], position[2]),
        )
        return [(position, q) for q in rotations] + [(p, orientation) for p in translations]

    def preflight_targets(self, targets, start_position, start_orientation, readiness):
        """Preflight each sample pose; mark unplannable poses to skip."""
        self.wait_for_joint_states()
        self.planning.set_start_state_to_current_state()
        previous = None
        print(f"Starting preflight for {len(targets)} sample poses and the return to the starting pose. No motion or sampling at this stage.", flush=True)
        feasible_indices = set()
        try:
            for index, (position, orientation) in enumerate(targets, start=1):
                label = f"sample pose {index}/{len(targets)}"
                goal = PoseStamped()
                goal.header.frame_id = self.base_frame
                goal.header.stamp = self.node.get_clock().now().to_msg()
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = position
                (goal.pose.orientation.x, goal.pose.orientation.y,
                 goal.pose.orientation.z, goal.pose.orientation.w) = orientation
                print(f"Preflight {label}: XYZ={position}, quaternion={orientation}", flush=True)
                plan = None
                for attempt in range(1, self.max_planning_retries + 1):
                    if previous is None:
                        self.planning.set_start_state_to_current_state()
                    else:
                        self.planning.set_start_state(robot_state=previous)
                    self.planning.set_goal_state(pose_stamped_msg=goal, pose_link=self.tool_link)
                    print(f"Preflight {label}: planning attempt {attempt}/{self.max_planning_retries}...", flush=True)
                    plan = self.planning.plan()
                    if plan and len(plan.trajectory):
                        break
                if not plan or not len(plan.trajectory):
                    try:
                        readiness.diagnose_goal(goal, self.tool_link, "ur_manipulator")
                    except Exception as exc:
                        self.node.get_logger().warning(
                            f"Preflight diagnostics for {label} failed: {exc}"
                        )
                    print(f"Preflight could not plan {label}; it will be skipped.", flush=True)
                    continue
                previous = plan.trajectory[len(plan.trajectory)-1]
                feasible_indices.add(index)
                print(f"Preflight passed: {label}", flush=True)

            # Returning home is useful but does not invalidate otherwise usable
            # calibration samples if MoveIt cannot plan that final segment.
            home = PoseStamped()
            home.header.frame_id = self.base_frame
            home.header.stamp = self.node.get_clock().now().to_msg()
            home.pose.position.x, home.pose.position.y, home.pose.position.z = start_position
            (home.pose.orientation.x, home.pose.orientation.y,
             home.pose.orientation.z, home.pose.orientation.w) = start_orientation
            self.planning.set_start_state_to_current_state() if previous is None else \
                self.planning.set_start_state(robot_state=previous)
            self.planning.set_goal_state(pose_stamped_msg=home, pose_link=self.tool_link)
            home_plan = self.planning.plan()
            return_feasible = bool(home_plan and len(home_plan.trajectory))
            if return_feasible:
                print("Preflight passed: return to the starting pose.", flush=True)
            else:
                print("Preflight could not plan the return to the starting pose; the workflow will still attempt it after sampling.", flush=True)
        finally:
            self.planning.set_start_state_to_current_state()
        print(f"Preflight complete: {len(feasible_indices)}/{len(targets)} sample poses are plannable.", flush=True)
        return feasible_indices, return_feasible

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
            print(
                f"MoveIt could not plan target {label} after "
                f"{self.max_planning_retries} attempts. Skipping this pose.",
                flush=True,
            )
            return False
        status = self.robot.execute(result.trajectory, controllers=[])
        if not status:
            raise RuntimeError(f"Target {label} execution failed; no sample collected")
        return True

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
        for key in ("take_sample", "get_sample_list", "remove_sample", "compute_calibration",
                    "save_calibration"):
            client = readiness.service_clients[key]
            while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info(f"Waiting for Easy HandEye 2 {key} service...")
            if not rclpy.ok():
                raise RuntimeError("ROS 2 shut down")

        sample_client = readiness.service_clients["take_sample"]
        readiness.clear_samples()

        start_position, start_orientation = self.current_pose()
        targets = self.make_targets(start_position, start_orientation)
        feasible_indices, _ = self.preflight_targets(
            targets, start_position, start_orientation, readiness
        )
        print(f"Starting tool pose ({self.base_frame}): {start_position}", flush=True)
        print(f"Capturing {len(targets)} calibration samples (6 single-axis rotations, 6 coupled rotations, 3 extended marker-normal tilts, and 5 translations including two extra base-X front/back poses).", flush=True)

        successful_poses = []
        skipped_poses = {}
        for index, (position, orientation) in enumerate(targets, start=1):
            if index not in feasible_indices:
                skipped_poses[index] = "preflight planning failed"
                continue
            print(f"\nPose {index}/{len(targets)}: planning and moving...", flush=True)
            if not self.plan_and_execute(position, orientation, index, readiness):
                skipped_poses[index] = "MoveIt planning failed during execution"
                continue
            previous_sample_count = len(successful_poses)
            sample_count = previous_sample_count
            max_sample_attempts = 3
            for sample_attempt in range(1, max_sample_attempts + 1):
                time.sleep(self.settle_seconds if sample_attempt == 1 else 1.0)
                response = readiness.call_service(
                    sample_client, TakeSample.Request(), "take sample"
                )
                sample_count = len(response.samples.samples)
                if sample_count == previous_sample_count + 1:
                    break
                if sample_count != previous_sample_count:
                    raise RuntimeError(
                        f"Unexpected Easy HandEye sample count after pose {index}: "
                        f"expected {previous_sample_count} or {previous_sample_count + 1}, got {sample_count}"
                    )
                if sample_attempt < max_sample_attempts:
                    print(
                        f"Sample {index} was rejected as unstable; keeping the robot "
                        f"still and retrying ({sample_attempt + 1}/{max_sample_attempts}).",
                        flush=True,
                    )
            if sample_count != previous_sample_count + 1:
                skipped_poses[index] = "marker not detected or pose rejected as unstable"
                print(
                    f"Pose {index} was not sampled after {max_sample_attempts} attempts; "
                    "skipping it and continuing.", flush=True,
                )
                continue
            successful_poses.append(index)
            print(f"Captured sample for pose {index}: {len(successful_poses)}/{len(targets)}", flush=True)
            if index < len(targets):
                time.sleep(self.inter_pose_pause_seconds)

        print("\nReturning to the starting pose...", flush=True)
        returned_to_start = self.plan_and_execute(
            start_position, start_orientation, "return", readiness
        )
        if not returned_to_start:
            print("Could not plan the return to the starting pose.", flush=True)

        print("\n========== Sampling Summary ==========")
        print(f"Successful poses: {len(successful_poses)}/{len(targets)} {successful_poses}")
        print(f"Skipped poses: {len(skipped_poses)}/{len(targets)}")
        for pose_index, reason in sorted(skipped_poses.items()):
            print(f"  Pose {pose_index}: {reason}")
        print(f"Returned to starting pose: {'yes' if returned_to_start else 'no'}")
        print("======================================\n", flush=True)

        minimum_successful_samples = math.ceil(0.8 * len(targets))
        if len(successful_poses) < minimum_successful_samples:
            raise RuntimeError(
                f"Only {len(successful_poses)}/{len(targets)} poses were sampled "
                f"successfully; at least {minimum_successful_samples} are required "
                "to compute a calibration."
            )
        result = readiness.call_service(
            readiness.service_clients["compute_calibration"],
            ComputeCalibration.Request(), "compute calibration",
        )
        if not result.valid:
            raise RuntimeError("Easy HandEye 2 could not compute a valid calibration")
        self.print_calibration(result.calibration)
        from easy_handeye2.handeye_calibration import filepath_for_calibration
        previous = filepath_for_calibration(result.calibration.parameters.name)
        if previous.exists():
            backup = previous.with_name(previous.name + f".backup-{time.time_ns()}")
            shutil.copy2(previous, backup)
            print(f"Previous calibration backed up: {backup}", flush=True)
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


def run_worker(args):
    """Execute one internal MoveItPy stage using this same script."""
    rclpy.init()
    readiness = None
    motion = None
    result_code = 0
    try:
        readiness = CalibrationReadiness()
        motion = AutomaticCalibration(args.robot_base_frame, args.robot_effector_frame)
        motion.tracking_base = args.tracking_base_frame
        motion.marker_frame = args.tracking_marker_frame
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



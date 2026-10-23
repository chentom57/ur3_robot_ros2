#!/usr/bin/env python3
"""Acquire ~15 diverse poses with MoveItPy and calibrate through easy_handeye2."""

import math
import time

import rclpy
from easy_handeye2_msgs.srv import ComputeCalibration, SaveCalibration, TakeSample
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path
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


class AutomaticHandeyeCalibration:
    def __init__(self):
        self.node = Node("ur3_auto_handeye_calibration")
        defaults = {
            "planning_group": "ur_manipulator",
            "tool_link": "tool0",
            "base_frame": "base_link",
            "rotation_delta_degrees": 20.0,
            "translation_delta_meters": 0.04,
            "settle_seconds": 1.5,
        }
        for name, value in defaults.items():
            self.node.declare_parameter(name, value)
        self.params = {name: self.node.get_parameter(name).value for name in defaults}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=True)
        moveit_config = (
            MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
            .robot_description_semantic(
                file_path=Path("srdf") / "ur.srdf.xacro", mappings={"name": "ur3"}
            )
            .to_moveit_configs()
        )
        self.robot = MoveItPy(
            node_name="ur3_auto_handeye_moveit", config_dict=moveit_config.to_dict()
        )
        self.planning = self.robot.get_planning_component(self.params["planning_group"])

        self.take_sample_client = self.node.create_client(TakeSample, "/easy_handeye2/calibration/take_sample")
        self.get_samples_client = self.node.create_client(TakeSample, "/easy_handeye2/calibration/get_sample_list")
        self.compute_client = self.node.create_client(
            ComputeCalibration, "/easy_handeye2/calibration/compute_calibration"
        )
        self.save_client = self.node.create_client(
            SaveCalibration, "/easy_handeye2/calibration/save_calibration"
        )

    def wait_for_service(self, client, label):
        self.node.get_logger().info(f"等待 Easy HandEye 2 {label} service…")
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f"尚未連上 {label}，請確認 calibrate.launch.py 正在執行")
        if not rclpy.ok():
            raise RuntimeError("ROS 2 已關閉")

    def call_service(self, client, request, label):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.exception():
            raise RuntimeError(f"{label} service 失敗：{future.exception()}")
        return future.result()

    def current_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.params["base_frame"], self.params["tool_link"], Time(),
            timeout=Duration(seconds=5.0),
        ).transform
        return (
            (transform.translation.x, transform.translation.y, transform.translation.z),
            (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
        )

    def make_targets(self, position, orientation):
        """12 varied wrist rotations plus 3 small XYZ translations = 15 poses."""
        delta = math.radians(float(self.params["rotation_delta_degrees"]))
        axes = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
        rotations = []
        for degrees in (delta, -delta, delta / 2, -delta / 2):
            for axis in axes:
                rotations.append(quat_multiply(orientation, axis_angle_quaternion(axis, degrees)))

        offset = float(self.params["translation_delta_meters"])
        translations = (
            (position[0] + offset, position[1], position[2]),
            (position[0], position[1] + offset, position[2]),
            (position[0], position[1], position[2] + offset),
        )
        targets = [(position, q) for q in rotations]
        targets.extend((p, orientation) for p in translations)
        return targets

    def plan_and_execute(self, position, orientation, label):
        goal = PoseStamped()
        goal.header.frame_id = self.params["base_frame"]
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = position
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = orientation
        self.planning.set_start_state_to_current_state()
        self.planning.set_goal_state(pose_stamped_msg=goal, pose_link=self.params["tool_link"])
        result = self.planning.plan()
        if not result:
            raise RuntimeError(f"MoveIt 無法規劃第 {label} 個目標；已停止，未執行此軌跡")
        self.robot.execute(result.trajectory, controllers=[])

    @staticmethod
    def print_calibration(calibration):
        p = calibration.parameters
        t = calibration.transform.translation
        q = calibration.transform.rotation
        if p.calibration_type == "eye_in_hand":
            parent, child = p.robot_effector_frame, p.tracking_base_frame
        else:
            parent, child = p.robot_base_frame, p.tracking_base_frame
        print("\n========== Easy HandEye 2 計算結果 ==========")
        print(f"校正類型：{p.calibration_type}")
        print(f"TF：{parent} -> {child}")
        print(f"translation [m]: x={t.x:.9f}, y={t.y:.9f}, z={t.z:.9f}")
        print(f"rotation quaternion [x, y, z, w]: [{q.x:.9f}, {q.y:.9f}, {q.z:.9f}, {q.w:.9f}]")
        print("matrix [R | t]:")
        # ROS quaternion in x,y,z,w order; explicit matrix makes the result easy to reuse.
        x, y, z, w = q.x, q.y, q.z, q.w
        matrix = (
            (1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w), t.x),
            (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w), t.y),
            (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y), t.z),
            (0.0, 0.0, 0.0, 1.0),
        )
        for row in matrix:
            print("  [" + ", ".join(f"{value:.9f}" for value in row) + "]")
        print("==============================================\n")

    def run(self):
        self.wait_for_service(self.take_sample_client, "take_sample")
        self.wait_for_service(self.get_samples_client, "get_sample_list")
        self.wait_for_service(self.compute_client, "compute_calibration")
        self.wait_for_service(self.save_client, "save_calibration")
        start_position, start_orientation = self.current_pose()
        targets = self.make_targets(start_position, start_orientation)
        existing = self.call_service(self.get_samples_client, TakeSample.Request(), "讀取既有樣本")
        if existing.samples.samples:
            raise RuntimeError(
                f"Easy HandEye 2 目前已有 {len(existing.samples.samples)} 個樣本。"
                "為避免把舊姿態混入新標定，請先在 Easy HandEye 2 介面清空舊樣本，再重新執行。"
            )

        print("自動手眼標定即將執行 15 個 MoveIt 姿態並逐點取樣。")
        print(f"起始 tool pose ({self.params['base_frame']}): {start_position}")
        print(f"旋轉範圍：±{self.params['rotation_delta_degrees']}°；平移偏移：{self.params['translation_delta_meters']} m")
        print("請先確認 marker 全程可見、機器人周圍安全，並在 RViz 檢查目前規劃場景。")
        if input("輸入 START 才開始移動手臂：").strip() != "START":
            print("使用者取消。")
            return

        settle = float(self.params["settle_seconds"])
        try:
            for index, (position, orientation) in enumerate(targets, start=1):
                print(f"\n姿態 {index}/15：移動並規劃…")
                self.plan_and_execute(position, orientation, index)
                time.sleep(settle)
                response = self.call_service(self.take_sample_client, TakeSample.Request(), "取樣")
                sample_count = len(response.samples.samples)
                if sample_count != index:
                    raise RuntimeError(f"Easy HandEye 2 未成功取得第 {index} 個樣本（目前 {sample_count} 個）")
                print(f"已取得樣本 {sample_count}/15")

            print("\n回到取樣前的姿態…")
            self.plan_and_execute(start_position, start_orientation, "return")
            result = self.call_service(self.compute_client, ComputeCalibration.Request(), "計算校正")
            if not result.valid:
                raise RuntimeError("Easy HandEye 2 無法從目前樣本計算有效校正")
            self.print_calibration(result.calibration)
            save = self.call_service(self.save_client, SaveCalibration.Request(), "儲存校正")
            if not save.success:
                raise RuntimeError("計算成功，但 Easy HandEye 2 儲存校正失敗")
            print(f"校正已儲存：{save.filepath.data}")
        except Exception:
            self.node.get_logger().error("自動標定中止。手臂可能停在目前姿態；請先檢查機器人狀態。")
            raise

    def shutdown(self):
        self.robot.shutdown()
        self.tf_listener.unregister()
        self.node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    app = None
    try:
        app = AutomaticHandeyeCalibration()
        app.run()
    except (KeyboardInterrupt, TransformException) as exc:
        if exc:
            print(f"標定中止：{exc}")
    except Exception as exc:
        print(f"自動標定失敗：{exc}")
    finally:
        if app:
            app.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

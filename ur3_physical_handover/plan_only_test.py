#!/usr/bin/env python3

import math
import rclpy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path


def main():
    rclpy.init()

    node = Node("ur3_physical_plan_only")

    robot = None
    listener = None

    try:
        config = (
            MoveItConfigsBuilder(
                robot_name="ur",
                package_name="ur_moveit_config"
            )
            .robot_description_semantic(
                file_path=Path("srdf") / "ur.srdf.xacro",
                mappings={"name": "ur3"}
            )
            .to_moveit_configs()
        )

        parameters = config.to_dict()
        parameters["planning_pipelines"] = {
            "pipeline_names": ["ompl"]
        }

        parameters.setdefault("ompl", {})["ur_manipulator"] = {
            "planner_configs": ["RRTConnect"]
        }

        parameters["plan_request_params"] = {
            "planning_pipeline": "ompl",
            "planner_id": "RRTConnect",
            "planning_time": 5.0,
            "planning_attempts": 5,
            "max_velocity_scaling_factor": 0.05,
            "max_acceleration_scaling_factor": 0.05,
        }

        robot = MoveItPy(
            node_name="ur3_physical_plan_only_moveit",
            config_dict=parameters
        )

        planning = robot.get_planning_component("ur_manipulator")

        tf_buffer = Buffer()
        listener = TransformListener(
            tf_buffer,
            node,
            spin_thread=True
        )

        tool = tf_buffer.lookup_transform(
            "base_link",
            "tool0",
            Time(),
            timeout=Duration(seconds=5.0)
        ).transform

        current_xyz = (
            tool.translation.x,
            tool.translation.y,
            tool.translation.z,
        )

        print("Current tool0 position:", current_xyz)

        # Candidate target: 2 cm above the current tool position.
        # This is a planning test, NOT an execution command.
        target_xyz = (
            current_xyz[0],
            current_xyz[1],
            current_xyz[2] + 0.02,
        )

        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x = target_xyz[0]
        target.pose.position.y = target_xyz[1]
        target.pose.position.z = target_xyz[2]
        target.pose.orientation = tool.rotation

        print("Candidate target:", target_xyz)

        planning.set_start_state_to_current_state()
        planning.set_goal_state(
            pose_stamped_msg=target,
            pose_link="tool0"
        )

        print("Planning only. No execution will occur.")

        result = planning.plan()

        if result:
            print("PLAN FOUND")
            print("Review the trajectory and physical workspace before execution.")
        else:
            print("NO PLAN FOUND")

    finally:
        if robot:
            robot.shutdown()
        if listener:
            listener.unregister()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

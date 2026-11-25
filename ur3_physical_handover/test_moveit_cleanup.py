#!/usr/bin/env python3

import gc
import rclpy
from pathlib import Path
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder


def main():
    rclpy.init()
    robot = None

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

        parameters.setdefault(
            "ompl", {}
        )["ur_manipulator"] = {
            "planner_configs": ["RRTConnect"]
        }

        print("Initializing MoveItPy...", flush=True)

        robot = MoveItPy(
            node_name="ur3_cleanup_test",
            config_dict=parameters
        )

        print(
            "MoveItPy initialized successfully.",
            flush=True
        )

    finally:
        if robot is not None:
            print("Stopping MoveItPy...", flush=True)
            robot.shutdown()

            print(
                "Destroying MoveItPy before ROS shutdown...",
                flush=True
            )

            del robot
            gc.collect()

            print(
                "MoveItPy destruction attempted.",
                flush=True
            )

        print("Shutting down ROS...", flush=True)
        rclpy.shutdown()

        print("Cleanup completed.", flush=True)


if __name__ == "__main__":
    main()

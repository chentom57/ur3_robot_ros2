#!/usr/bin/env python3
"""Diagnose MoveItPy shutdown without commanding robot motion."""

import sys
from pathlib import Path

import rclpy

PROJECT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT / "ur3_base_pick"))

from ur3_base_pick.base_coordinate_pick import BaseCoordinatePick


def main():
    rclpy.init()
    app = None

    try:
        print("TEST 1: Initializing MoveItPy", flush=True)
        app = BaseCoordinatePick()

        print("TEST 2: Initialization completed", flush=True)
        print("No planning or execution performed.", flush=True)

    finally:
        print("TEST 3: Starting cleanup", flush=True)

        if app is not None:
            app.shutdown()

        print("TEST 4: BaseCoordinatePick shutdown completed",
              flush=True)

        if rclpy.ok():
            rclpy.shutdown()

        print("TEST 5: ROS shutdown completed", flush=True)


if __name__ == "__main__":
    main()

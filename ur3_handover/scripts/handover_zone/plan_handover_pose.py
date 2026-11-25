#!/usr/bin/env python3
"""Plan UR3 handover and return paths without execution."""

import sys
from pathlib import Path

import rclpy

PROJECT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT / "ur3_base_pick"))

from ur3_base_pick.base_coordinate_pick import BaseCoordinatePick


HANDOVER_TARGET = (-0.25, 0.35, 0.20)


def main():
    rclpy.init()
    app = None

    try:
        app = BaseCoordinatePick()

        if not app.scene_ready:
            print("STOP: Virtual table collision object is missing.")
            return

        print("=== UR3 HANDOVER: PLAN ONLY ===")
        print("Target XYZ (base_link):", HANDOVER_TARGET)
        print("Preserving current tool0 orientation.")
        print("No execution or gripper commands.")

        outbound, return_plan = app.plan_pick_and_return(
            HANDOVER_TARGET
        )

        if outbound is None:
            print("FAILED: No reachable outbound trajectory.")
        elif return_plan is None:
            print("PARTIAL: Outbound planned; return path failed.")
        else:
            print("SUCCESS: Both trajectories planned.")
            print("Current -> Handover -> up")

        input("Press Enter to close the planner...")

    except KeyboardInterrupt:
        pass
    finally:
        if app is not None:
            app.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

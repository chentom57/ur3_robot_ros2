
#!/usr/bin/env python3
"""Mock-only UR3 handover motion test. No gripper commands."""

import subprocess
import sys
import time
from pathlib import Path

import rclpy

PROJECT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT / "ur3_base_pick"))

from ur3_base_pick.base_coordinate_pick import BaseCoordinatePick

TARGET = (-0.25, 0.35, 0.20)


def require_mock_hardware():
    """Refuse motion unless the active UR3 hardware is mock."""
    result = subprocess.run(
        ["ros2", "control", "list_hardware_components"],
        capture_output=True,
        text=True,
        timeout=15,
        check=True,
    )

    output = result.stdout

    if (
        "name: ur3" not in output
        or "plugin name: mock_components/GenericSystem" not in output
    ):
        raise RuntimeError(
            "STOP: UR3 mock hardware not confirmed. "
            "Execution is disabled."
        )

    services = subprocess.run(
        ["ros2", "service", "list"],
        capture_output=True,
        text=True,
        timeout=15,
        check=True,
    ).stdout

    managers = [
        line for line in services.splitlines()
        if line.endswith("/list_hardware_components")
    ]

    if managers != ["/controller_manager/list_hardware_components"]:
        raise RuntimeError(
            "STOP: Unexpected controller manager configuration."
        )

    print("SAFETY CHECK PASSED: UR3 mock hardware confirmed.")


def main():
    require_mock_hardware()

    rclpy.init()
    app = None

    try:
        app = BaseCoordinatePick()

        if not app.scene_ready:
            print("STOP: Virtual table collision object missing.")
            return

        print("\n=== AUTOMATIC MOCK HANDOVER ===")
        print("Target:", TARGET)
        print("No gripper commands.")
        print("No physical robot execution intended.")

        outbound, return_preflight = app.plan_pick_and_return(TARGET)

        if outbound is None or return_preflight is None:
            print("STOP: Both paths must plan successfully.")
            return

        print("\nBoth paths planned.")
        print("Review the trajectory and table clearance in RViz.")

        answer = input(
            "Execute on verified MOCK hardware? Type MOCK: "
        )

        if answer.strip() != "MOCK":
            print("Canceled.")
            return

        require_mock_hardware()

        print("\nMoving virtual UR3 to handover target...")

        if not app.execute_plan(outbound, "Mock outbound"):
            print("Outbound execution failed. Stopping.")
            return

        print("At handover target. Holding for 3 seconds.")
        time.sleep(3)

        # Replan return from the actual mock joint state.
        app.planning.set_start_state_to_current_state()
        app.planning.set_goal_state(configuration_name="up")
        return_plan = app.planning.plan()

        if return_plan is None:
            print("Return planning failed. Robot stays at current pose.")
            return

        require_mock_hardware()

        print("Returning virtual UR3 to 'up'...")

        if not app.execute_plan(return_plan, "Mock return"):
            print("Return execution failed.")
            return

        print("\nSUCCESS: Mock handover movement completed.")

    finally:
        if app is not None:
            app.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

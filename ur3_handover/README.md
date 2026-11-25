# UR3 Mock Handover Simulation

This folder contains the ROS 2 and MoveIt-based mock handover workflow.

## Main components

### Grasping
- add_virtual_object.py: Adds a virtual object to the planning scene.
- attach_virtual_object.py: Attaches the virtual object to the robot.
- show_handover_object.py: Visualizes the object in RViz.
- check_object_state.py: Checks object attachment and release state.
- ur3_mock_handover_cycle.py: Runs the mock carry, handover, and return workflow.
- ur3_mock_release_return.py: Simulates object release and robot return.
- ur3_hybrid_handover.py: Alternative mock handover workflow.

### Handover zone
- show_handover_zone.py: Visualizes the handover zone in RViz.
- plan_handover_pose.py: Plans the handover and return paths.
- execute_mock_handover.py: Tests mock handover movement.

## Important

These scripts are for mock testing and MoveIt planning.
They do not perform a complete physical human handover or control
the physical Robotiq gripper.
The example handover position is not a calibrated human handover position.

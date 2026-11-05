# UR3 Base-Frame Coordinate Pick Program

This ROS 2 package accepts a target in `base_link`, plans the outbound path for `tool0`, and preplans a return path to the MoveIt named state `up`. Once both paths are planned, it displays the target and paths in RViz and waits for confirmation before moving or operating the gripper.

## Features

- Accepts a `base_link` XYZ target in meters and keeps the current `tool0` orientation.
- Prints the current `tool0` position before each prompt and marks it in RViz.
- Plans from the current position to the target, then preplans the return from the outbound endpoint to `up`. No motion starts if either plan fails.
- Displays the target and both paths in RViz; execution requires the user to enter `y`.
- Pauses for one second at the target, closes the gripper, returns to `up`, and opens the gripper.
- Can sample an inverse-kinematics (IK) slice in XY at a selected height and print intervals with and without IK solutions.

## Requirements

- Ubuntu with ROS 2 Jazzy.
- UR robot bringup, `ur_moveit_config`, MoveIt 2, and TF publishing robot state.
- An `ur_manipulator` planning group and the named state `up` in the SRDF.
- For a physical gripper, a running Robotiq TCP service accessible at the configured IP and port.

## Add to a ROS 2 workspace and build

Place this folder under the workspace `src` directory and keep its name as `ur3_base_pick`:

```bash
cp -r /path/to/robotic\ control/ur3_base_pick ~/ur3_ws/src/
source /opt/ros/jazzy/setup.bash
cd ~/ur3_ws
colcon build --packages-select ur3_base_pick
source install/setup.bash
```

Before running the program in a new terminal, source the ROS and workspace environments:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run ur3_base_pick base_coordinate_pick
```

Start the robot driver, MoveIt, and any required gripper TCP service in other terminals first. The program connects to the running ROS 2 system; it does not start robot bringup itself.

## Operation

The program continuously prints the current `tool0` coordinates in `base_link` and accepts:

| Input | Action |
| --- | --- |
| `x y z` | Plan to the target and back to `up`. Separate values with spaces or commas, for example `-0.22 0.25 0.56` or `-0.22, 0.25, 0.56`. |
| `r` | Print the current `tool0` coordinates again. The robot does not move. |
| `m` | Enter a slice height `z`, print XY IK intervals, and publish workspace markers. |
| `q` | Quit. |

After both paths plan successfully, inspect them in RViz and check the surrounding space. Enter `y` to open the gripper, move to the target, grasp, and return to `up`. Any other input cancels without moving the arm or gripper.

## RViz visualization

Set RViz **Fixed Frame** to `base_link` and add a **MarkerArray** display:

- `/ur3_base_coordinate_pick/target_and_path_markers`: red sphere for the target, blue line for the outbound path, and yellow line for the return to `up`.
- `/ur3_base_coordinate_pick/workspace_map`: shows the current `tool0` position; after entering `m`, it shows the selected-height slice. Green cells indicate sampled points with an IK solution; red cells indicate points where the current sample found no IK solution.

The workspace slice holds the current `tool0` orientation fixed and checks IK only. It does not guarantee a collision-free path or represent the complete workspace boundary. Use MoveIt's outbound and return planning results for the final decision.

## Parameters

Example parameter overrides:

```bash
ros2 run ur3_base_pick base_coordinate_pick --ros-args \
  -p robot_ip:=192.168.86.7 \
  -p gripper_port:=63352 \
  -p gripper_backend:=tcp
```

| Parameter | Default | Description |
| --- | --- | --- |
| `base_frame` | `base_link` | Base frame for input coordinates |
| `tool_link` | `tool0` | End link targeted by MoveIt |
| `planning_group` | `ur_manipulator` | MoveIt planning group |
| `robot_ip` | `192.168.86.7` | Robotiq TCP service IP |
| `gripper_port` | `63352` | Robotiq TCP port |
| `gripper_backend` | `tcp` | `tcp` controls the physical gripper; `none` skips gripper commands |
| `workspace_map_min` | `-0.60` | Lower x and y bound for the slice, in meters |
| `workspace_map_max` | `0.60` | Upper x and y bound for the slice, in meters |
| `workspace_map_step` | `0.05` | XY sampling interval, in meters |

With `gripper_backend:=none`, the program still plans and executes the arm paths, but skips opening, grasping, and releasing the gripper.

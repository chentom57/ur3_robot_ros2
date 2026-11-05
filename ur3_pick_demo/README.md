# UR3 Coordinate-Based Pick Demo

This document describes the purpose, workflow, installation, and operation of the `ur3_pick_demo` Python prototype. It is separate from the ROS 2 environment setup guide. Set up ROS 2, the UR driver, MoveIt, the camera, and Easy HandEye 2 using [`ur3_ros2_setup.md`](../ur3_ros2_setup.md) first.

## 1. What the program does

The program accepts a 3D point `(x, y, z)` in a camera coordinate frame, transforms it into the robot base frame, and uses MoveIt to plan the arm's approach, grasp, and lift motions. With a physical Robotiq gripper, it sends open and close commands over a TCP socket.

This prototype is intended to check the hand-eye TF and basic motion sequence. The input is a 3D coordinate, not an image pixel `(u, v)`. The program does not detect objects, convert pixels to 3D, estimate grasp orientation, or add object collision geometry.

## 2. How it works

### Coordinate transform

The Easy HandEye 2 calibration result must be loaded and published into the ROS TF tree by its TF publisher. The program does not read the `.calib` file directly. It queries the transform from `camera_color_optical_frame` to `base_link` and transforms the input point into `base_link`.

The point is treated as the target position of `tool0`, not the object center. The input should already be the position where the tool center is intended to arrive. The program reads the current `tool0` orientation from TF and keeps that orientation during motion.

### Arm motion

The program uses MoveItPy. The default planning group is `ur_manipulator`, and the tool link is `tool0`. The sequence is:

1. Transform the point into `base_link` and calculate an approach point 10 cm above it.
2. Open the Robotiq gripper when using the TCP gripper backend.
3. Plan and execute the approach trajectory.
4. Plan and execute the trajectory to the target.
5. Close the gripper, then plan and execute a lift back to the approach point.

The program first asks whether to start, then asks for `y` before each arm trajectory. Inspect the trajectory in RViz, the workspace, and the space around the arm before confirming.

### Gripper connection

The default `tcp` backend connects to `robot_ip` and `gripper_port` (default `192.168.86.7:63352`), activates the gripper, and commands open position `0` and close position `255`. These are controller position commands; they do not confirm that an object was successfully grasped.

Set `gripper_backend:=none` to skip physical gripper commands. Arm trajectories are still sent to the MoveIt controller. For mock hardware, start the UR driver with `use_mock_hardware:=true` and confirm MoveIt connects to that mock controller. This does not simulate the gripper.

## 3. Prerequisites

- The UR driver, MoveIt, and RealSense camera are running.
- The Easy HandEye 2 TF publisher is running with the correct calibration name, and the configured camera frame is connected to `base_link` in TF.
- Camera coordinates are in meters and refer to the configured camera frame.
- For physical operation, the gripper TCP service is reachable and the workspace is clear of people and obstacles.

Example Easy HandEye 2 publisher command (replace the name with the saved calibration name):

```bash
ros2 launch easy_handeye2 publish.launch.py name:=ur3_realsense_eob
```

## 4. Install the package

Copy the supplied `ur3_pick_demo/` folder into the ROS workspace `src` directory. This example uses `~/ur3_ws`:

```bash
cp -r /path/to/ur3_pick_demo ~/ur3_ws/src/
cd ~/ur3_ws
source /opt/ros/jazzy/setup.bash
rosdep install -i --from-paths src --rosdistro jazzy -y
colcon build --symlink-install
```

In every terminal that runs the program, source the ROS and workspace environments:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
```

After the initial build, rebuild with `colcon build --symlink-install` whenever the package source changes.

## 5. Start and operate

Use separate terminals for the driver/MoveIt/camera, the Easy HandEye TF publisher, and the pick program. Source the ROS and workspace environments in each terminal.

In a new terminal, run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run ur3_pick_demo coordinate_pick
```

Enter camera-frame `x y z` coordinates in meters, for example:

```text
Camera x y z> 0.32 -0.08 0.45
```

Comma-separated values are also accepted. Enter `q`, `quit`, or `exit` to quit. The program prints the transformed `base_link` coordinates and asks for confirmation. It then asks for `y` before executing each arm trajectory. Do not confirm if the coordinates, orientation, or path look wrong.

## 6. Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `robot_ip` | `192.168.86.7` | IP address of the Robotiq TCP service |
| `gripper_port` | `63352` | Robotiq TCP port |
| `gripper_backend` | `tcp` | `tcp` controls the physical gripper; `none` skips gripper commands |
| `base_frame` | `base_link` | Robot base coordinate frame |
| `camera_frame` | `camera_color_optical_frame` | Camera frame for the input point |
| `tool_link` | `tool0` | MoveIt target tool link |
| `planning_group` | `ur_manipulator` | MoveIt planning group |
| `approach_height` | `0.10` | Approach and lift height above the target, in meters |

Example parameter overrides:

```bash
ros2 run ur3_pick_demo coordinate_pick --ros-args \
  -p camera_frame:=camera_color_optical_frame \
  -p approach_height:=0.10 \
  -p gripper_backend:=tcp
```

To use a mock arm and skip physical gripper commands:

```bash
ros2 run ur3_pick_demo coordinate_pick --ros-args -p gripper_backend:=none
```

## 7. Troubleshooting

### TF lookup fails

Check that the Easy HandEye 2 publisher is still running, the calibration name is correct, and the actual TF frame names match `base_frame` and `camera_frame`. Check the transform with `ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame`.

### MoveIt cannot plan

Check that the UR driver and MoveIt are running, and that `ur_manipulator` and `tool0` match the active UR configuration. Confirm that the target is reachable. If planning fails, the program does not execute that trajectory.

### Gripper connection or command fails

Check `robot_ip`, the port, network connectivity, and the Robotiq controller state. To test arm planning without the physical gripper, set `gripper_backend:=none`; this still controls the arm.

## 8. Future object-detection integration

The input loop is separate from `pick_camera_point(point_camera)`. A future perception node can convert an image detection into a 3D grasp point in the camera frame and call the same sequence. Grasp-orientation estimation, collision geometry, and grasp-result checks can be added later. Do not pass pixel coordinates directly to this program.

## RViz target marker

The entered camera-frame point is transformed into `base_link` and published as a red `GRASP` sphere. A green sphere marks the approach point. Both markers have labels showing the entered camera coordinates. In RViz, set **Fixed Frame** to `base_link`, add a **Marker** display, and set its topic to `/ur3_coordinate_pick/target_markers`. The markers remain visible until the next point is entered.

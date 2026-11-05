# UR3 + RealSense: Integrated Easy HandEye 2 Calibration

This guide uses the `integrated_auto_handeye_calibration` program. The script performs 15-pose sampling, calibration, and saving; no separate automatic calibration script needs to be installed or registered. It starts the ArUco tracker and waits for a marker. After detection, it asks whether to start Easy HandEye 2. Once calibration is saved, it asks whether to publish TF. Sampling starts from the arm's current pose and does not first move to a preset pose.

## Hardware and software

| Item | Configuration |
|---|---|
| Operating system | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Robot | Universal Robots UR3 |
| UR3 IP | `192.168.86.7` |
| Workspace | `~/ur3_ws` |
| Camera | RealSense with ROS 2 wrapper |
| Calibration marker | ArUco `ARUCO_ORIGINAL`, ID 582, 40 mm black-square edge |
| Calibration package | `easy_handeye2` |

## 1. Select the calibration type

| Camera mounting | ArUco board mounting | Calibration type |
|---|---|---|
| Fixed to a table or stand | Fixed to the UR3 end tool | `eye_on_base` |
| Fixed to the UR3 end tool | Fixed in the workspace | `eye_in_hand` |

## 2. Add the integrated calibration program to Easy HandEye 2 (one-time setup)

For RealSense SDK, ROS 2 wrapper, UR driver, and MoveIt installation, see the [ROS 2 setup guide](../ur3_ros2_setup.md). The steps below add the integrated script from this folder to `easy_handeye2` and register a `ros2 run` command.

### 2.1 Get the Easy HandEye 2 source

If the package is not already in the workspace:

```bash
cd ~/ur3_ws/src
git clone https://github.com/marcoesposito1988/easy_handeye2.git
```

If `~/ur3_ws/src/easy_handeye2` already exists, use that checkout.

### 2.2 Copy the integrated calibration program

```bash
cp "$HOME/Desktop/robotic control/handeye/integrated_handeye_calibration.py" \
  "$HOME/ur3_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/integrated_handeye_calibration.py"
```

Place the file in the Python package directory containing `handeye_server.py` and `handeye_publisher.py`.

### 2.3 Register the command in `setup.py`

Edit `~/ur3_ws/src/easy_handeye2/easy_handeye2/setup.py` and add this entry to `entry_points` → `console_scripts`:

```python
'integrated_auto_handeye_calibration = easy_handeye2.integrated_handeye_calibration:main',
```

### 2.4 Check runtime dependencies in `package.xml`

Edit `~/ur3_ws/src/easy_handeye2/easy_handeye2/package.xml` and ensure these dependencies exist. Add only missing entries:

```xml
<exec_depend>aruco_opencv</exec_depend>
<exec_depend>moveit_py</exec_depend>
<exec_depend>moveit_configs_utils</exec_depend>
```

Keep the existing `rclpy`, `tf2_ros`, `geometry_msgs`, and `easy_handeye2_msgs` dependencies.

### 2.5 Install dependencies, build, and verify the command

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ur3_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select easy_handeye2 --symlink-install
source ~/ur3_ws/install/setup.bash
ros2 pkg executables easy_handeye2
```

The executable list should include:

```text
easy_handeye2 integrated_auto_handeye_calibration
```

## 3. Start the robot and camera before calibration

Follow [Section 6 of the ROS 2 setup guide](../ur3_ros2_setup.md#6-launch-commands) to start the UR3 driver, MoveIt/RViz, and RealSense camera, and leave them running. The integrated program needs `/robot_description`, joint states, and TF for MoveIt to plan the 15 sample poses.

Confirm that the UR teach pendant is ready for External Control, the arm area is safe, and the ID 582 board is visible to the camera. The default image topic is `/camera/camera/color/image_raw`. In RViz2, add an **Image** display, or run `ros2 run rqt_image_view rqt_image_view` to check the image.

`marker_size` is the physical edge length of the black ArUco square, excluding the white border. The default is `0.04` meters.

## 4. Run integrated calibration

Open a new terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run easy_handeye2 integrated_auto_handeye_calibration
```

The integrated program proceeds as follows:

1. Starts the ArUco tracker and waits for the specified marker TF. The terminal reports every few seconds while the marker is not detected; there is no timeout for marker detection.
2. After detection, asks whether to start calibration. Enter `n` to stop the tracker and quit; enter `y` to launch the Easy HandEye 2 calibration interface.
3. Waits for the calibration service, robot TF, and marker TF, then starts automatic sampling from the arm's current pose. It does not move to `test_configuration` first.
4. After calibration succeeds and is saved, closes the tracker and calibration interface, then asks whether to publish TF. Enter `y` to publish continuously until Ctrl+C; enter `n` to quit.

Before starting, check marker visibility, motion clearance, and the MoveIt collision scene. Calibration physically moves the robot; keep the emergency stop accessible.

### Configurable arguments

The default type is `eye_on_base`, with the name `ur3_realsense_eob`. To select eye-in-hand:

```bash
ros2 run easy_handeye2 integrated_auto_handeye_calibration \
  --calibration-type eye_in_hand \
  --name ur3_realsense_eih
```

Run `ros2 run easy_handeye2 integrated_auto_handeye_calibration --help` to see other frame, image topic, marker size, and startup timeout options. The name is used for saving and publishing the result and must match in both places.

## 5. Automatic sampling

After confirmation, the program reads the current `base_link -> tool0` TF as the starting pose and generates 15 targets:

| Samples | Target poses |
|---|---|
| 1–12 | Keep the starting position and rotate around each tool-frame X, Y, and Z axis by `+20°`, `-20°`, `+10°`, and `-10°`, producing 12 poses. |
| 13–15 | Keep the starting orientation and translate `+0.04 m` (4 cm) along each `base_link` X, Y, and Z axis. |

MoveIt plans and executes each target. At each pose the program waits 1.5 seconds, then requests a sample from Easy HandEye 2. If planning a target fails, it retries up to three times. If all attempts fail, it diagnoses IK and state validity, prints the start/target poses and diagnostic results, then stops without executing that target. If old samples are present before the run, it aborts to avoid mixing batches. After each sample, it verifies that the sample count increased as expected. A planning, sampling, or count-check failure stops the process.

If interrupted, collected partial samples are not cleared automatically. Clear old samples in the Easy HandEye 2 interface before restarting. After all 15 samples, the arm returns to its starting pose, then the program computes and saves the calibration.

Keep the marker visible throughout the run and verify that target poses and paths are safe. If the process stops early, the arm may remain at its current pose; check the robot state before continuing.

## 6. Calibration result

By default, the calibration file is saved at:

```text
~/.ros2/easy_handeye2/calibrations/<name>.calib
```

The terminal prints the calibration type, TF parent and child frames, translation, quaternion, and 4×4 matrix. If you choose `y` when asked to publish, the integrated flow starts `publish.launch.py` with the same `<name>` and continuously publishes the calibration TF.

## References

- UR3 ROS 2 driver: [Universal Robots ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [Easy HandEye 2 documentation](https://github.com/marcoesposito1988/easy_handeye2)
- [RealSense ROS 2 Wrapper](https://github.com/realsenseai/realsense-ros)
- [aruco_opencv](https://github.com/fictionlab/ros_aruco_opencv)

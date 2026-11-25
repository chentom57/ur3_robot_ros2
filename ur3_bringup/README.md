# UR3 bringup

[繁體中文](README.zh-TW.md)

`ur3_bringup` starts the UR3 driver, MoveIt, RViz, the shared virtual collision table, and optionally the RealSense camera.

The official UR driver and `ur_moveit_config` are obtained from GitHub. The project's `launch/moveit.launch.py` includes upstream MoveIt and adds the table and RViz configuration from `config/moveit.rviz`; it requires no edits to the official driver. Launch it through bringup, or run it separately after starting the driver:

```bash
ros2 launch ur3_bringup moveit.launch.py ur_type:=ur3
```

The wrapper disables upstream RViz and any table added to a customized upstream launch, then starts one project RViz and one table publisher. Its RViz configuration includes the robot, scene, target/path markers, color image, point cloud and camera/base axes.

## Installation and build

Complete Sections 1–4 of the [host/source setup guide](../ROS2+driver/ur3_ros2_setup.md). That flow builds this package and the scene package. Continue with launching below; no repeated installation or build is needed.

| Dependency | Preparation |
| --- | --- |
| ROS 2 Jazzy, rosdep and colcon | Host guide Section 1. |
| `ur_robot_driver`, `ur_moveit_config` | Official GitHub `jazzy` source in host guide Section 4. |
| `realsense2_camera` and system SDK | Official source tag 4.59.1 and the system SDK in host guide Section 2. |
| `ur3_moveit_scene` | Included in this project. |
| MoveIt, RViz and launch dependencies | Installed together by rosdep in host guide Section 4. |

Building the complete project also needs `easy_handeye2_msgs`, whose source checkout is covered in the host guide. Running the mock bringup does not require calibration, Docker or an AnyGrasp license. Source ROS and the workspace in each new terminal.

## Physical robot

Set up the UR controller's external-control program and network connection, then run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup bringup.launch.py robot_ip:=192.168.86.7
```

Leave this terminal running. The configured robot type is UR3. Default physical bringup enables the camera and its point cloud, RViz and the table. If the camera driver is already running independently:

```bash
ros2 launch ur3_bringup bringup.launch.py launch_camera:=false
```

## Mock hardware

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup bringup.launch.py \
  use_mock_hardware:=true launch_camera:=false
```

This runs the driver with mock hardware rather than connecting to a physical robot. With `launch_camera:=auto`, mock hardware automatically disables the camera.

## RViz displays

Set RViz Fixed Frame to `base_link`. The supplied configuration displays the robot, planning scene, shared collision table, target and planned paths, camera image, point cloud and TF. The target/path MarkerArray is published on `/ur3_pick_up/target_and_path_markers`.

## Launch arguments

Pass launch arguments as `name:=value`, without `--ros-args -p`.

| Argument | Default | Meaning |
| --- | --- | --- |
| `robot_ip` | `192.168.86.7` | Physical UR controller IP. |
| `use_mock_hardware` | `false` | `true` for mock hardware; `false` for the physical robot. |
| `virtual_table_enabled` | `true` | Publish the shared collision slab into MoveIt's scene. |
| `virtual_table_z` | `-0.01` | Table top-surface height in `base_link`, meters. Set to the physical installation. |
| `launch_camera` | `auto` | `auto`: camera on with physical hardware, off with mock hardware; `true`/`false` force the choice. |
| `launch_rviz` | `true` | Start MoveIt RViz. |

When launched here, the camera receives `pointcloud.enable=true`. There is no separate point-cloud launch argument in this package. The table is a 4×4 m slab, 0.02 m thick; its top is at `virtual_table_z`. Its object ID is `ur3_virtual_table_collision`.

Example overrides:

```bash
ros2 launch ur3_bringup bringup.launch.py \
  robot_ip:=192.168.86.7 launch_camera:=true \
  virtual_table_z:=-0.01 launch_rviz:=true
```

## Camera SDK source and rebuilding

Use one RealSense SDK installation: the system `librealsense2` and `librealsense2-dev` packages from the RealSense apt repository. The current environment uses SDK 2.59.1; RealSense ROS 4.59.x source requires SDK 2.59.0 or later. See the [environment setup guide](../ROS2+driver/ur3_ros2_setup.md) for the SDK repository and installation.

Before running the host guide's `rosdep` command, install that system SDK. These commands use `--skip-keys librealsense2` because the ROS dependency key otherwise installs `ros-jazzy-librealsense2`, creating another SDK under `/opt/ros/jazzy`. The skip option leaves SDK installation under your control; it does not install the SDK itself.

Check the selected installation:

```bash
dpkg-query -W librealsense2 librealsense2-dev
apt-cache policy ros-jazzy-librealsense2
```

If the ROS SDK is also installed, inspect `apt-get --simulate remove ros-jazzy-librealsense2` before removing it with `sudo apt remove ros-jazzy-librealsense2`. Review the removal list on that computer, especially if it uses binary RealSense ROS packages. The source camera driver in this project uses the system SDK.

Rebuild the source camera driver after updating the SDK version or changing its installation source:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
cd ~/ur3_ws
colcon build --packages-select realsense2_camera --symlink-install \
  --cmake-force-configure --cmake-clean-first \
  --cmake-args -DBUILD_TESTING=OFF
source ~/ur3_ws/install/setup.bash
```

This uses the camera driver's normal CMake dependency discovery and needs no project-specific SDK helper file or changes to the official camera source. Restart bringup after rebuilding. Check matching compiled/runtime SDK versions, fresh depth/cloud messages and `camera_link → camera_color_optical_frame` before starting the hand-eye publisher. Matching SDK versions do not by themselves guarantee successful camera streaming.

## Check startup status

```bash
ros2 control list_controllers
ros2 topic list
```

Check controller status and camera topics. In RViz, use Fixed Frame `base_link` to inspect the robot model and collision table. Stop bringup with Ctrl+C in its terminal.

<a id="file-inventory"></a>
## File inventory

Every source file in this folder and its subfolders is listed below, relative to this folder. Build outputs, runtime logs and caches are excluded.

| File | Purpose |
| --- | --- |
| [CMakeLists.txt](CMakeLists.txt) | Define the ament_cmake package and install launch files, RViz configuration and bilingual READMEs. |
| [README.md](README.md) | English guide: Physical/mock startup, launch parameters, RViz, SDK troubleshooting and status checks. |
| [README.zh-TW.md](README.zh-TW.md) | Traditional Chinese guide: Physical/mock startup, launch parameters, RViz, SDK troubleshooting and status checks. |
| [config/moveit.rviz](config/moveit.rviz) | RViz displays for the robot, planning scene, table, target/path markers, image, cloud and TF. |
| [launch/bringup.launch.py](launch/bringup.launch.py) | Combine UR driver, MoveIt, RViz, table and camera for physical/mock hardware. |
| [launch/moveit.launch.py](launch/moveit.launch.py) | Include upstream MoveIt and manage the project RViz/collision table without duplicate launches. |
| [package.xml](package.xml) | ROS package identity, version, license and build/runtime dependency declarations. |

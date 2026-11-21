# Build the Robot and Camera Environment from Scratch (ROS 2 Jazzy)

This guide sets up the ROS 2, Python, robot-arm, and camera environment used by this project. Commands and settings below assume Ubuntu 24.04, ROS 2 Jazzy, a UR3, and a Robotiq gripper. Adapt network addresses and hardware-specific settings for other equipment.

Traditional Chinese version: [ur3_ros2_setup.zh-TW.md](ur3_ros2_setup.zh-TW.md)

## Contents

- [Hardware configuration](#hardware-configuration)
- [1. Install ROS 2](#1-install-ros-2)
- [2. Prepare host tools and RealSense SDK](#2-install-robot-camera-and-moveit-packages)
- [3. Create a workspace](#3-create-a-workspace)
- [4. Download source and build](#4-download-source-and-build)
- [5. Check UR3 and gripper network access](#5-check-ur3-and-gripper-network-access)
- [6. Launch the camera, UR3 and MoveIt separately](#6-launch-commands)
- [7. Check the driver and MoveIt](#7-check-the-driver-and-moveit)


## Hardware configuration

| Item | Configuration |
| --- | --- |
| Operating system | Ubuntu 24.04 |
| ROS version | ROS 2 Jazzy |
| UR model | UR3 |
| UR3 / gripper IP | `192.168.86.7` |
| Robotiq socket port | `63352` |
| MoveIt planning group | `ur_manipulator` |

For other hardware, update the shell commands and parameters in the Python programs as needed.

## 1. Install ROS 2

These instructions use Ubuntu 24.04 and ROS 2 Jazzy. To use another ROS 2 distribution, replace `jazzy` in the commands below with that distribution's name.

Ensure the system uses a UTF-8 locale:

```bash
locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Enable the required repository and install tools for adding the ROS repository:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the ROS 2 repository and update package lists:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

Install the ROS 2 Jazzy desktop distribution and development tools, including colcon, rosdep, and vcstool:

```bash
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

Source ROS 2 in every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

<a id="2-install-robot-camera-and-moveit-packages"></a>
## 2. Prepare host tools and RealSense SDK

This guide uses the official UR driver's GitHub source. Section 4 installs controller, message, MoveIt, RViz, ArUco, Qt/rqt and Python dependencies once through `rosdep`; no separate binary UR bundle is needed.

Install the source-download tool:

```bash
sudo apt install -y git
```

### RealSense SDK

If the required RealSense packages are not available through the configured apt sources, add the RealSense repository and install the SDK:

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
  | gpg --dearmor \
  | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-dev
```

Use this system SDK as the project's single SDK source. The RealSense ROS 4.59.x source requires SDK 2.59.0 or later; the current environment uses 2.59.1. Subsequent `rosdep` commands skip the `librealsense2` key to avoid installing a second SDK through `ros-jazzy-librealsense2`. Install the system SDK first; skipping the key does not install it. See the [bringup SDK guide](../ur3_bringup/README.md) when checking an existing installation.

## 3. Create a workspace

Create a workspace on each computer. The recommended path is `~/ur3_ws`; choose another path if desired.

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ur3_ws/src
cd ~/ur3_ws
```

## 4. Download source and build

Download the official UR driver (including `ur_moveit_config`) and the RealSense ROS 4.59.1 source into the workspace. These clone commands are for first-time setup; reuse existing checkouts instead of cloning duplicates:

```bash
cd ~/ur3_ws/src
git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
git clone --branch 4.59.1 --depth 1 https://github.com/realsenseai/realsense-ros.git
```

### Place the complete project in the workspace

Use one copy of this repository under `~/ur3_ws/src/robot_arm`. If it was downloaded elsewhere, copy the complete folder once, adjusting the source path:

```bash
cp -r "/path/to/robot_arm" "$HOME/ur3_ws/src/robot_arm"
cd ~/ur3_ws
colcon list
```

`colcon` recursively discovers five packages: `auto_calibration`, `easy_handeye2`, `ur3_bringup`, `ur3_moveit_scene` and `ur3_pick_up`. A workspace that already contains these packages separately can keep that layout; do not add a second copy. Do not copy build/install/log directories from another machine.

`ur3_bringup/launch/moveit.launch.py` includes official MoveIt and adds the table and bundled `config/moveit.rviz`. No official driver edits are required.

<a id="handeye-message-dependency"></a>
### Add the hand-eye message dependency

`easy_handeye2_msgs` is not bundled in this repository and has no rule in this machine's Jazzy rosdep index. Download only its subtree from upstream, pinned to the revision matching the package version 0.5.0 used here. Skip this step if `colcon list` already finds one `easy_handeye2_msgs` package.

```bash
cd ~/ur3_ws/src
git clone --filter=blob:none --sparse \
  https://github.com/marcoesposito1988/easy_handeye2.git \
  easy_handeye2_interfaces
git -C easy_handeye2_interfaces sparse-checkout set easy_handeye2_msgs
git -C easy_handeye2_interfaces checkout --detach \
  29bd50a7939e853d768d203d2abc61eb6e50facc
```

Sparse checkout excludes upstream's `easy_handeye2` implementation, avoiding a duplicate of this project's `easy_handeye2/` package. Do not clone another full upstream implementation alongside it.

### Install dependencies and build once

The five project packages and official drivers declare their dependencies in `package.xml`. The `rosdep` command below installs them, including ArUco, MoveItPy, Qt/rqt, OpenCV, NumPy, transforms3d and YAML for calibration. The host does not need PyTorch or AnyGrasp; prepare that environment separately with the [Docker guide](../docker+anygrasp/README.md).

`rosdep` handles declared dependencies with installation rules. Download driver/message sources first and install the system RealSense SDK in Section 2. The Docker guide covers separate user preparation of NVIDIA drivers, Docker/Container Toolkit, AnyGrasp SDK, licensing and model weights.

Initialize rosdep once per system if it has not been initialized before, then update its package index:

```bash
sudo rosdep init
rosdep update
```

`rosdep` reads each package's `package.xml` and installs build/runtime dependencies in one command. `--ignore-src` skips packages whose sources are already in the workspace. Only build/runtime dependency classes are selected below; test/documentation dependencies and CMake tests are excluded. It does not download source repositories or replace `colcon build`.

Install dependencies and build the workspace:

```bash
cd ~/ur3_ws
rosdep install --from-paths src --ignore-src --rosdistro jazzy \
  --skip-keys librealsense2 \
  -t build -t buildtool -t build_export -t buildtool_export -t exec -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source ~/ur3_ws/install/setup.bash
```


## 5. Check UR3 and gripper network access

These operating-system-level checks confirm that the laptop can reach the robot and gripper service. This guide uses UR3 IP `192.168.86.7`.

Check whether the UR3 responds to ICMP:

```bash
ping -c 3 192.168.86.7
```

If there is no response, check the network settings and device connection. Some networks block ICMP, so also check that the laptop is on the correct network, its IP is on a reachable subnet, and the UR3 is powered and connected.

Check whether the Robotiq TCP socket on port `63352` is reachable:

```bash
nc -vz 192.168.86.7 63352
```

If `nc` is unavailable:

```bash
sudo apt install netcat-openbsd
```

A successful connection confirms that the port is reachable; it does not verify ROS 2 or gripper operation. If it fails, check that the UR controller and Robotiq socket server are running, the IP and port are correct, and the network allows the connection.

<a id="6-launch-commands"></a>
## 6. Launch the camera, UR3 and MoveIt separately

Open three terminals in order, source ROS and the workspace in each, and keep them running. Reuse an already running service rather than launching a second instance.

### Terminal 1: RealSense camera and point cloud

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```

Connect RealSense to USB 3.x and confirm streaming starts. For camera or ArUco detection checks, this terminal alone is sufficient initially; see the [manual-calibration guide](../easy_handeye2/README.md) for tracker startup.

### Terminal 2: UR3 driver

For a physical arm, power it on and enable External Control. The laptop must have network access to the robot.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 robot_ip:=192.168.86.7 \
  use_mock_hardware:=false launch_rviz:=false
```

Without a physical arm, replace `use_mock_hardware:=false` with `use_mock_hardware:=true` in the same command. The camera runs independently in Terminal 1. Driver RViz is disabled because Terminal 3 starts the project RViz.

### Terminal 3: MoveIt, RViz and collision table

Start this after the UR3 driver is ready:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup moveit.launch.py \
  ur_type:=ur3 launch_rviz:=true \
  virtual_table_enabled:=true virtual_table_z:=-0.01
```

This launch integrates upstream MoveIt, the project RViz configuration and the collision table. The first two terminals manage the camera and UR3 driver independently. The virtual tabletop defaults to `z=-0.01 m` in `base_link`; set `virtual_table_z` to the measured height. See the [bringup guide](../ur3_bringup/README.md) for parameters and the [scene guide](../ur3_moveit_scene/README.md) for collision-table details.

Continue with the [automatic-calibration guide](../auto_calibration/README.md) or [pick-up guide](../ur3_pick_up/README.md). With these three services running, proceed directly to calibration or picking; skip those guides' combined-bringup startup commands. Press `Ctrl+C` to stop a terminal's services independently.

## 7. Check the driver and MoveIt

Check the trajectory controller:

```bash
ros2 topic list | grep joint_trajectory_controller
```

Check the MoveIt IK service:

```bash
ros2 service list | grep compute_ik
```

List ROS 2 nodes, topics, and services:

```bash
ros2 node list
ros2 topic list
ros2 service list
```


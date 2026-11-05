# Build the Robot and Camera Environment from Scratch (ROS 2 Jazzy)

This guide sets up the ROS 2, Python, robot-arm, and camera environment used by this project. Commands and settings below assume Ubuntu 24.04, ROS 2 Jazzy, a UR3, and a Robotiq gripper. Adapt network addresses and hardware-specific settings for other equipment.

## Contents

- [Hardware configuration](#hardware-configuration)
- [1. Install ROS 2](#1-install-ros-2)
- [2. Install robot, camera, and MoveIt packages](#2-install-robot-camera-and-moveit-packages)
- [3. Create a workspace](#3-create-a-workspace)
- [4. Download source and build](#4-download-source-and-build)
- [5. Check UR3 and gripper network access](#5-check-ur3-and-gripper-network-access)
- [6. Launch the system](#6-launch-commands)
- [7. Check the driver and MoveIt](#7-check-the-driver-and-moveit)
- [Laptop-specific settings](#laptop-specific-settings)
- [ROS 2 tools](#ros-2-tools)

## Hardware configuration

| Item | Configuration |
| --- | --- |
| Operating system | Ubuntu 24.04 |
| ROS version | ROS 2 Jazzy |
| UR model | UR3 |
| UR3 / gripper IP | `192.168.86.7` |
| Robotiq socket port | `63352` |
| MoveIt planning group | `manipulator` |

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
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the ROS 2 repository and update package lists:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
```

Install the ROS 2 Jazzy desktop distribution and development tools, including colcon, rosdep, and vcstool:

```bash
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

Source ROS 2 in every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

To source it automatically, add the command to `~/.bashrc`:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## 2. Install robot, camera, and MoveIt packages

Install the hardware abstraction and controller packages:

```bash
sudo apt install -y ros-jazzy-ros2-control
sudo apt install -y ros-jazzy-ros2-controllers
sudo apt install -y ros-jazzy-control-msgs
sudo apt install -y ros-jazzy-trajectory-msgs
```

Install motion planning and visualization packages:

```bash
sudo apt install -y ros-jazzy-moveit
sudo apt install -y ros-jazzy-rviz2
sudo apt install -y ros-jazzy-robot-state-publisher
sudo apt install -y ros-jazzy-moveit-py
```

The binary UR packages are optional because this guide builds the source versions below:

```bash
sudo apt update
sudo apt install -y ros-jazzy-ur
```

Install Python packages used by the programs:

```bash
sudo apt install -y python3-numpy python3-yaml python3-pip
```

### RealSense SDK and tools

If the required RealSense packages are not available through the configured apt sources, add the RealSense repository and install the SDK:

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
  | gpg --dearmor \
  | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev
```

## 3. Create a workspace

Create a workspace on each computer. The recommended path is `~/ur3_ws`; choose another path if desired.

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ur3_ws/src
cd ~/ur3_ws
```

## 4. Download source and build

Download the Universal Robots driver and RealSense ROS 2 wrapper into the workspace:

```bash
cd ~/ur3_ws/src
git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
git clone -b ros2-master https://github.com/realsenseai/realsense-ros.git
```

For Easy HandEye 2 installation and integration of the automatic calibration program, see [the Easy HandEye 2 guide](handeye/README.md), Section 2. Then continue with dependency installation and building below.

Initialize rosdep once per system if it has not been initialized before, then update its package index:

```bash
sudo rosdep init
rosdep update
```

Install dependencies and build the workspace:

```bash
cd ~/ur3_ws
rosdep install -i --from-paths src --rosdistro jazzy -y
colcon build --symlink-install
source ~/ur3_ws/install/setup.bash
```

To source the workspace automatically in new terminals, add this line to `~/.bashrc`:

```bash
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

If `nc` is unavailable, install it with `sudo apt install netcat-openbsd`. A successful connection confirms that the port is reachable; it does not verify ROS 2 or gripper operation. If it fails, check that the UR controller and Robotiq socket server are running, the IP and port are correct, and the network allows the connection.

## 6. Launch commands

Use separate terminals for the services so that each log can be inspected and each service can be stopped or restarted independently. Source ROS 2 and the workspace in every terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
```

Terminal 1: physical UR3 driver, with the driver's own RViz disabled:

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.86.7 use_mock_hardware:=false launch_rviz:=false
```

For mock hardware, change `use_mock_hardware:=false` to `use_mock_hardware:=true`. This does not connect to a physical UR3.

Terminal 2: MoveIt and RViz:

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true
```

Terminal 3: RealSense camera:

```bash
ros2 launch realsense2_camera rs_launch.py
```

To run a Python program in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
python3 ~/ur3_ws/src/<script_name>.py
```

For a physical arm, power it on and put the teach pendant in a state that accepts External Control. The laptop and robot must be on the same network. Connect RealSense to USB 3.x. Press `Ctrl+C` in a terminal to stop the service started there. Do not start another combined launch at the same time, as that can start duplicate nodes.

### Optional: open three terminal windows automatically

The supplied helper script can open three GNOME Terminal windows and start one service in each. On first use, copy the complete adjacent `ur3_bringup/` package, including `package.xml`, `CMakeLists.txt`, `launch/`, and `scripts/`, into `~/ur3_ws/src/`. After downloading the source packages in Section 4, install dependencies and build:

```bash
cd ~/ur3_ws
source /opt/ros/jazzy/setup.bash
rosdep install -i --from-paths src --rosdistro jazzy -y
colcon build --symlink-install
```

Confirm that the output includes `Finished <<< ur3_bringup`. From a desktop terminal, run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run ur3_bringup start_separate_terminals.sh
```

The script uses UR IP `192.168.86.7` by default and lets the RealSense driver select a camera. Its optional second argument selects mock hardware; the default `false` connects to the physical arm. The argument format is `[UR_IP] [true|false]`:

```bash
ros2 run ur3_bringup start_separate_terminals.sh 192.168.86.7 true
```

This example starts the UR driver with mock hardware, so it does not connect to the physical UR3; MoveIt and the camera still start. To control the physical arm, omit the second argument or set it to `false`. A camera serial number is usually not needed and is not exposed as a launch argument.

Each window shows its service log. Press `Ctrl+C` in a window to stop that service, then press Enter to close the window. This method requires GNOME Terminal; install it with `sudo apt install gnome-terminal` if needed. After copying or updating `ur3_bringup`, rebuild with `colcon build --symlink-install` and source the workspace again.

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

## Laptop-specific settings

Each laptop needs its own workspace path, `.bashrc` source paths, and network adapter configuration. If all computers use `~/ur3_ws`, the workspace commands can be the same.

## ROS 2 tools

Use `rqt_image_view` to view camera images published by ROS 2:

```bash
ros2 run rqt_image_view rqt_image_view
```

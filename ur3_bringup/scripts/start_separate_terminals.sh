#!/usr/bin/env bash
set -euo pipefail

robot_ip="${1:-192.168.86.7}"
use_mock_hardware="${2:-false}"

if [[ "$use_mock_hardware" != "true" && "$use_mock_hardware" != "false" ]]; then
  echo "use_mock_hardware must be true or false (received: $use_mock_hardware)" >&2
  exit 2
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal is required. Install it with: sudo apt install gnome-terminal" >&2
  exit 1
fi

open_ros_terminal() {
  local title="$1"
  local command="$2"
  shift 2
  gnome-terminal --title="$title" -- bash -lc \
    'source /opt/ros/jazzy/setup.bash &&
     source ~/ur3_ws/install/setup.bash &&
     '"$command"';
     status=$?;
     echo;
     echo "Process ended with status $status. Press Enter to close this window.";
     read -r' _ "$@"
}

open_ros_terminal "UR3 Driver" \
  'ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:="$1" use_mock_hardware:="$2" launch_rviz:=false' \
  "$robot_ip" "$use_mock_hardware"

open_ros_terminal "UR3 MoveIt and RViz" \
  'ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true'

open_ros_terminal "RealSense Camera" 'ros2 launch realsense2_camera rs_launch.py'

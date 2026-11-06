# UR3 Bringup

This package starts the UR3 driver, MoveIt, MoveIt RViz, and the shared virtual table. It can also start the RealSense camera. Configure each launch with `name:=value` arguments.

## Build and source

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ur3_ws
colcon build --packages-up-to ur3_bringup --symlink-install
source ~/ur3_ws/install/setup.bash
```

## Launch

Run the bringup with its default settings:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup bringup.launch.py
```

### Launch arguments

| Argument | Default | Description |
| --- | --- | --- |
| `robot_ip` | `192.168.86.7` | UR controller IP used when connecting to physical hardware. |
| `use_mock_hardware` | `false` | Set `true` to use the UR ROS 2 driver's mock hardware; `false` connects to the controller at `robot_ip`. |
| `virtual_table_enabled` | `true` | Add the shared virtual table collision object to MoveIt's Planning Scene. |
| `virtual_table_z` | `-0.01` | Virtual table top-surface height relative to `base_link`, in meters. |
| `launch_camera` | `auto` | `auto` disables the camera with mock hardware and enables it with physical hardware. Set `true` or `false` to override. |
| `launch_rviz` | `true` | Start MoveIt RViz. |

Override one or more settings in the launch command. For example:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup bringup.launch.py \
  use_mock_hardware:=true \
  launch_camera:=false \
  virtual_table_z:=-0.04
```

The virtual table is a 4 m × 4 m collision slab, 0.02 m thick, with its top face at `virtual_table_z`. Its default height is `-0.01 m` in `base_link`; set the argument to the measured tabletop height in that frame. MoveIt checks this collision object during planning. The table can be disabled for a launch with `virtual_table_enabled:=false`.

Check that the trajectory controller is active with:

```bash
ros2 control list_controllers
```

For full environment setup instructions, see the [English ROS 2 UR3 setup guide](../ur3_ros2_setup.md) or the [Traditional Chinese guide](../ur3_ros2_setup.zh-TW.md).


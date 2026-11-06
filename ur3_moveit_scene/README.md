# Shared Virtual Table for MoveIt

This package adds a large horizontal collision slab to the shared MoveIt Planning Scene. It is started from `ur_moveit_config/launch/ur_moveit.launch.py`, so `move_group`, RViz, the pick program, and other planning clients see the same table. The 4 m × 4 m slab covers the UR3 workspace. The pick node does not create or own the table.

The launch defaults to a slab whose top face is at `z = -0.01 m` in `base_link`, one centimeter below the frame origin. Measure the physical tabletop relative to `base_link` and adjust the height as needed. The collision geometry is a 2 cm thick box extending downward from that top face.

## Install in `ur3_ws`

Copy this package into the workspace source directory:

```bash
cp -r /path/to/robotic\ control/ur3_moveit_scene ~/ur3_ws/src/
```

In the Universal Robots driver source at `~/ur3_ws/src/Universal_Robots_ROS2_Driver`, make these two changes:

1. In `ur_moveit_config/package.xml`, add `<exec_depend>ur3_moveit_scene</exec_depend>`.
2. In `ur_moveit_config/launch/ur_moveit.launch.py`, start `ur3_moveit_scene`'s `virtual_table` executable along with MoveIt. The current workspace copy already includes these changes.

Build the package and the MoveIt configuration:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ur3_ws
colcon build --packages-up-to ur_moveit_config --symlink-install
source ~/ur3_ws/install/setup.bash
```

## Run

Use the normal MoveIt launch command. The virtual table is enabled by default:

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true
```

Set its height relative to `base_link` at launch, for example:

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3 launch_rviz:=true virtual_table_z:=-0.035
```

Disable it when needed with `virtual_table_enabled:=false`.

The combined `ur3_bringup` launch also starts this MoveIt configuration and forwards both table arguments:

```bash
ros2 launch ur3_bringup bringup.launch.py virtual_table_z:=-0.035
```

The separate-terminal helper also invokes the same `ur_moveit_config` launch, so its default table remains enabled.

The collision slab is a 4 m × 4 m box, large enough to cover the UR3 work envelope. Its top face is the configured `virtual_table_z`. The package publishes the same translucent blue slab as a marker on `/moveit_virtual_table/marker`. The supplied MoveIt RViz configuration includes this marker display.

## Topics and parameters

| Name | Default | Description |
|---|---:|---|
| Launch argument `virtual_table_enabled` | `true` | Start the shared table publisher |
| Launch argument `virtual_table_z` | `-0.01` | Table top height in `base_link`, meters |
| Node parameter `base_frame` | `base_link` | Frame in which the table is defined |
| Node parameter `table_extent` | `4.0` | Side length of the collision slab, meters |
| Node parameter `table_thickness` | `0.02` | Slab thickness below the top face, meters |
| Planning scene topic | `/planning_scene` | Shared collision-object update consumed by MoveIt scene monitors |
| RViz marker topic | `/moveit_virtual_table/marker` | Visualization of the collision slab |

# Shared Virtual Table for MoveIt

This package adds a large horizontal collision slab to the shared MoveIt Planning Scene. It is started from `ur3_bringup/launch/moveit.launch.py`, so `move_group`, RViz, the pick program, and other planning clients see the same table. The 4 m × 4 m slab covers the UR3 workspace. The pick node does not create or own the table.

The launch defaults to a slab whose top face is at `z = -0.01 m` in `base_link`, one centimeter below the frame origin. Measure the physical tabletop relative to `base_link` and adjust the height as needed. The collision geometry is a 2 cm thick box extending downward from that top face.

## Installation and build

Complete Sections 1–4 of the [host/source setup guide](../ROS2+driver/ur3_ros2_setup.md). This package is included in the complete project. `rosdep` installs rclpy, NumPy and geometry/visualization/MoveIt/shape message dependencies; `colcon` builds the package in the same flow. No repeated installation is needed here.

Integrated launching requires the official UR/MoveIt dependencies described in [bringup](../ur3_bringup/README.md). Standalone scene/workspace publishers do not need a camera, calibration result, Docker or AnyGrasp. Keep one copy of each package; no official UR driver edits are needed.

## Run

Use the project MoveIt launch command. The virtual table is enabled by default:

```bash
ros2 launch ur3_bringup moveit.launch.py ur_type:=ur3 launch_rviz:=true
```

Set its height relative to `base_link` at launch, for example:

```bash
ros2 launch ur3_bringup moveit.launch.py \
  ur_type:=ur3 launch_rviz:=true virtual_table_z:=-0.035
```

Disable it when needed with `virtual_table_enabled:=false`.

The combined `ur3_bringup` launch also starts this MoveIt configuration and forwards both table arguments:

```bash
ros2 launch ur3_bringup bringup.launch.py virtual_table_z:=-0.035
```

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

<a id="file-inventory"></a>
## File inventory

Every source file in this folder and its subfolders is listed below, relative to this folder. Build outputs, runtime logs and caches are excluded.

| File | Purpose |
| --- | --- |
| [README.md](README.md) | English guide: Shared collision table, workspace visualization and scene parameters. |
| [README.zh-TW.md](README.zh-TW.md) | Traditional Chinese guide: Shared collision table, workspace visualization and scene parameters. |
| [package.xml](package.xml) | ROS package identity, version, license and build/runtime dependency declarations. |
| [resource/ur3_moveit_scene](resource/ur3_moveit_scene) | Package marker in the ament index; required for ROS discovery even when empty. |
| [setup.cfg](setup.cfg) | Install registered executables under lib/ur3_moveit_scene for ros2 run discovery. |
| [setup.py](setup.py) | Register virtual_table/workspace_preview and install the ROS package marker and bilingual READMEs. |
| [ur3_moveit_scene/__init__.py](ur3_moveit_scene/__init__.py) | Python package marker for shared scene utilities. |
| [ur3_moveit_scene/virtual_table.py](ur3_moveit_scene/virtual_table.py) | Add the virtual table to the MoveIt Planning Scene and publish scene updates/RViz markers. |
| [ur3_moveit_scene/workspace.py](ur3_moveit_scene/workspace.py) | Shared workspace bounds, point-in-volume checks and workspace MarkerArray geometry. |
| [ur3_moveit_scene/workspace_preview.py](ur3_moveit_scene/workspace_preview.py) | Independent bringup workspace preview, yielding marker publication to an active pick task and resuming afterward. |

# Shared virtual collision table

This package adds one horizontal collision slab to the shared MoveIt Planning Scene. `ur3_bringup/launch/moveit.launch.py` starts it so `move_group`, RViz and planning clients use the same table. The pick-up node does not own or create the table.

The default top face is at `z = -0.01 m` in `base_link`. Measure the physical tabletop relative to `base_link` and adjust `virtual_table_z` as needed. The collision box is 4 m × 4 m and 2 cm thick, extending down from its top face. The package also publishes a matching translucent blue RViz marker.

## Build

Complete Sections 1–4 of the [host setup guide](../ROS2+driver/ur3_ros2_setup.md). This package is included in the complete project build. No separate installation is needed.

## Run

```bash
ros2 launch ur3_bringup moveit.launch.py ur_type:=ur3 launch_rviz:=true
```

Set the table height in `base_link`, for example:

```bash
ros2 launch ur3_bringup moveit.launch.py \
  ur_type:=ur3 launch_rviz:=true virtual_table_z:=-0.035
```

The combined bringup launch forwards the same setting:

```bash
ros2 launch ur3_bringup bringup.launch.py virtual_table_z:=-0.035
```

Set `virtual_table_enabled:=false` to omit the collision slab when appropriate.

## Topics and parameters

| Name | Default | Description |
|---|---:|---|
| Launch argument `virtual_table_enabled` | `true` | Start the shared table publisher |
| Launch argument `virtual_table_z` | `-0.01` | Table top height in `base_link`, meters |
| Node parameter `base_frame` | `base_link` | Frame in which the table is defined |
| Node parameter `table_extent` | `4.0` | Table side length, meters |
| Node parameter `table_thickness` | `0.02` | Thickness below the top face, meters |
| Planning scene topic | `/planning_scene` | Shared collision-object updates |
| RViz marker topic | `/moveit_virtual_table/marker` | Visualization of the collision slab |

<a id="file-inventory"></a>
## File inventory

| File | Purpose |
|---|---|
| [README.md](README.md) | English guide to the shared collision table. |
| [README.zh-TW.md](README.zh-TW.md) | Traditional Chinese guide to the shared collision table. |
| [package.xml](package.xml) | ROS package metadata and runtime dependencies. |
| [resource/ur3_moveit_scene](resource/ur3_moveit_scene) | Ament package marker. |
| [setup.cfg](setup.cfg) | Installs the executable under `lib/ur3_moveit_scene`. |
| [setup.py](setup.py) | Registers `virtual_table` and installs package resources and guides. |
| [ur3_moveit_scene/__init__.py](ur3_moveit_scene/__init__.py) | Python package marker. |
| [ur3_moveit_scene/virtual_table.py](ur3_moveit_scene/virtual_table.py) | Adds the shared table to MoveIt and publishes its RViz marker. |

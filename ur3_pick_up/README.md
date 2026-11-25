# UR3 live-camera AnyGrasp pick-up

[繁體中文](README.zh-TW.md)

`ros2 run ur3_pick_up pick_up` records the arm's current joint position as the task start and return target, then opens a live color view. Click two opposite corners around one target object to define a rectangular mask. The GUI stays open through inference, planning and motion. It asks for pick confirmation and, after the arm returns to its recorded start position, waits for a GUI release command before opening the gripper.

The rectangle mask maps to cloud points and steers AnyGrasp candidate generation. The complete filtered scene cloud is also sent for SDK collision checking. The host filters candidates by gripper opening and score; MoveIt plans outbound and return paths and checks robot/table collision. Candidate poses are tried in score order until both paths plan.

## Requirements and build

Use ROS 2 Jazzy with the UR driver, MoveItPy, RealSense ROS, `ur3_bringup`, `ur3_moveit_scene`, `auto_calibration`, and `easy_handeye2`. Follow Sections 1–4 of the [host setup guide](../ROS2+driver/ur3_ros2_setup.md); the complete project build includes this package. Keep PyTorch, CUDA, MinkowskiEngine and AnyGrasp inside Docker. See the [Docker guide](../docker+anygrasp/README.md) for the licensed SDK and weights.

The existing `anygrasp-fixed` container must provide `gsnet.so`, its license, and `log/checkpoint_detection.tar`. The robot controller must have the Robotiq socket service enabled. Keep one hand-eye transform publisher running; a new installation needs its own calibration file.

## Start the system

Terminal 1: enable External Control on the robot and start the driver, camera, MoveIt, RViz and shared virtual table:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup bringup.launch.py robot_ip:=192.168.86.7
```

Use `launch_camera:=false` if the camera is already running elsewhere.

Terminal 2: start the calibration publisher if one is not already running:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch auto_calibration publish.launch.py name:=ur3_realsense_eob tracking_root_frame:=camera_link
```

Terminal 3: authenticate Docker if needed and run the task:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
sudo -v
ros2 run ur3_pick_up pick_up
```

If Docker works directly for your account, omit `sudo -v` and set `docker_use_sudo:=false`.

For acquisition, inference and planning without arm/gripper commands:

```bash
sudo -v
ros2 run ur3_pick_up pick_up --ros-args -p plan_only:=true
```

`plan_only` still requires the live camera, calibration TF, MoveIt, Docker and collision table. The arm remains in place; the current joint state is recorded as the planning start and return target.

## Operator flow

1. The program records the current joint state without moving the arm.
2. The GUI displays the live RGB image. Click two opposite rectangle corners around one object; Enter accepts, right-click or `U` undoes a point, `R` resets, and Esc cancels.
3. A fresh synchronized image/cloud pair is acquired. Camera intrinsics map the selected image mask to cloud points. The complete filtered cloud is passed for collision filtering; the selected mask guides candidate generation. This is manual selection, not semantic segmentation. Keep nearby unwanted objects outside the rectangle.
4. Candidates wider than the configured gripper opening are filtered and remaining candidates are sorted by score. MoveIt plans both the outbound path and the return to the recorded startup state. Failed candidates are skipped; if all plans fail, the program reports the counts and does not execute a trajectory.
5. The GUI asks for execution confirmation. Before arm motion the gripper opens, then the arm moves to the chosen pose, closes the gripper and returns to its recorded start position.
6. The gripper stays closed until the GUI release command is confirmed. Abort leaves it closed. The GUI closes when the task finishes or stops.

The point cloud and camera transform are captured once for the task. Keep the camera and target still during inference and confirmation. The point cloud is used by AnyGrasp collision filtering; it is not inserted as a full-arm collision scene. MoveIt checks the robot against its planning scene, including the shared virtual tabletop.

## Coordinates and TCP

The SDK receives an N×3 cloud in camera optical coordinates, in meters: +X right in the image, +Y down, +Z forward from the lens. The configured `base_frame` is used for MoveIt planning. Terminal TCP values are shown in the optical frame.

The installed `config/grasp_tool.example.json` describes the current Robotiq 2F-85 setup: tool0 +Z TCP offset 0.174 m and maximum opening 0.085 m. Confirm these values for the installed tool.

```text
p_TCP_camera = translation + depth * R_grasp[:, 0]
R_TCP_camera = R_grasp * grasp_to_tcp_rotation
T_base_tool0 = T_base_camera * T_camera_grasp_tip * T_grasp_tip_tool0
```

## Main ROS parameters

Pass overrides after `--ros-args -p name:=value`.

| Parameter | Default | Purpose |
|---|---|---|
| `pointcloud_topic` | `/camera/camera/depth/color/points` | PointCloud2 source |
| `color_image_topic` | `/camera/camera/color/image_raw` | Raw image used for selection |
| `color_camera_info_topic` | `/camera/camera/color/camera_info` | Intrinsics for mapping selection to cloud points |
| `cloud_timeout` / `cloud_max_age` | `10.0` / `1.0` | Acquisition timeout / maximum sample age, seconds |
| `voxel_size` | `0.005` | Downsampling voxel size, meters |
| `max_points` / `min_roi_points` | `120000` / `50` | Full-scene point cap / minimum selected-region points after downsampling |
| `max_camera_depth` | `2.0` | Maximum optical Z, meters |
| `docker_container` / `docker_use_sudo` | `anygrasp-fixed` / `true` | Existing container / Docker privilege mode |
| `docker_sdk_dir` | `/opt/anygrasp_sdk/grasp_detection` | SDK and license directory in container |
| `checkpoint_path` | SDK `log/checkpoint_detection.tar` | Detection weights in container |
| `inference_timeout` | `120.0` | Inference time limit, seconds |
| `gripper_height` | `0.03` | Finger height for SDK collision filtering, meters |
| `exchange_dir` | `~/anygrasp_docker/exchange` | Per-run point cloud, metadata, result and log folders |
| `plan_only` | `false` | Capture, infer and plan without robot commands |
| `grasp_tool_config` | Installed tool JSON | Tool transform, grasp axes and opening limit |
| `base_frame` / `tool_link` | `base_link` / `tool0` | Planning frame and tip; current configuration requires tool0 |
| `tcp_offset` | `0.174` | tool0 +Z to TCP, meters; must match the tool |
| `planning_group` | `ur_manipulator` | MoveIt planning group |
| `robot_ip` / `gripper_port` | `192.168.86.7` / `63352` | Gripper controller address and port |
| `gripper_backend` | `tcp` | Set `none` to skip gripper commands |
| `max_velocity_scaling_factor` / `max_acceleration_scaling_factor` | `0.1` / `0.1` | MoveIt trajectory scaling, in (0,1] |

OMPL uses RRTConnect with a 5 s planning time and 10 configured attempts. A successful plan is required for both outbound and return motion; it does not guarantee execution success.

For GPU memory pressure, reduce the point cap and increase voxel size while using plan-only mode:

```bash
sudo -v
ros2 run ur3_pick_up pick_up --ros-args \
  -p plan_only:=true -p max_points:=60000 \
  -p voxel_size:=0.008 -p inference_timeout:=180.0
```

Downsampling can remove small-object detail. The selected region must retain at least `min_roi_points` after filtering and downsampling.

## Troubleshooting

- No cloud: check camera connection and the topic with `ros2 topic list`.
- Too few selected points: enlarge the rectangle around the target, check the camera view, and ensure the object is within the configured depth.
- PointCloud2 row-step error: inspect the active publisher and topic; the decoder reports dimensions, point step, row step and payload size.
- Docker permission denied: run `sudo -v` in the same terminal before launching.
- CUDA out of memory: reduce `max_points`, increase `voxel_size`, or reduce `max_camera_depth`.
- Planning failure: the program tries the next candidate in score order. If all fail, inspect candidate/planning logs and verify robot state, TF, table height and MoveIt scene.
- Each request is saved in a UUID folder under `exchange_dir`: `cloud.npz`, `capture.json`, `grasps.json` when inference succeeds, and `inference.log`.
- A segmentation fault after `Deleting MoveItCpp` is a native cleanup issue; check the printed task result to determine whether motion completed.

## Exit status

`pick_up` returns `0` after task completion or successful plan-only planning. Task or planning failures and declined execution return `1`; startup/configuration errors return `2`; Ctrl+C or closed interactive input returns `130`.

<a id="file-inventory"></a>
## File inventory

| File | Purpose |
|---|---|
| [README.md](README.md) | English operation guide. |
| [README.zh-TW.md](README.zh-TW.md) | Traditional Chinese operation guide. |
| [config/grasp_tool.example.json](config/grasp_tool.example.json) | Example tool/TCP settings. |
| [package.xml](package.xml) | ROS package metadata and runtime dependencies. |
| [resource/ur3_pick_up](resource/ur3_pick_up) | Ament package marker. |
| [scripts/anygrasp_worker.py](scripts/anygrasp_worker.py) | Container inference worker; does not control the robot. |
| [setup.cfg](setup.cfg) | Installs the executable for `ros2 run`. |
| [setup.py](setup.py) | Registers the executable and installs package resources. |
| [ur3_pick_up/anygrasp_input.py](ur3_pick_up/anygrasp_input.py) | Parses candidates and converts grasp poses to the configured tool/TCP. |
| [ur3_pick_up/anygrasp_task.py](ur3_pick_up/anygrasp_task.py) | Coordinates selection, inference, planning and confirmed task execution. |
| [ur3_pick_up/camera_capture.py](ur3_pick_up/camera_capture.py) | Acquires a fresh cloud and its matching camera transform. |
| [ur3_pick_up/docker_client.py](ur3_pick_up/docker_client.py) | Exchanges inference requests and results with the existing container. |
| [ur3_pick_up/live_input.py](ur3_pick_up/live_input.py) | Decodes clouds, maps selection masks and ranks candidates. |
| [ur3_pick_up/pick_up.py](ur3_pick_up/pick_up.py) | ROS entry point, MoveIt, markers and gripper control. |
| [ur3_pick_up/target_selection.py](ur3_pick_up/target_selection.py) | Live image selection and operator confirmation GUI. |

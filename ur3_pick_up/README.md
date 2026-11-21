# UR3 live-camera AnyGrasp pick-up

[繁體中文](README.zh-TW.md)

`ros2 run ur3_pick_up pick_up` captures a fresh camera cloud, calls AnyGrasp in the existing Docker container, tries valid workspace poses by descending score, and selects the first pose with successful outbound and return-to-home plans. Each invocation captures one scene and executes at most one pose. Arm motion still requires confirmation.

## Workspace and RViz

Forward is **base_link +Y**. The horizontal selection area is 60×60 cm:

| Axis | Limits (meters) |
| --- | --- |
| X | −0.30 to +0.30 |
| Y | 0.00 to +0.60 |
| Z | +0.02 to +0.60 |

Bringup displays a workspace preview in both physical and mock modes without this program, a camera or Docker. While this program runs, the preview releases its publishers and RViz displays the task's actual bounds. Bringup's preview resumes after exit.

RViz shows a translucent cyan box, edges and bounds label. These are included in `/ur3_pick_up/target_and_path_markers`, already displayed by the current bringup RViz configuration, together with the TCP target, orientation axes and outbound/return paths. For a workspace-only display, add **MarkerArray**, topic `/ur3_pick_up/workspace`, Fixed Frame `base_link`. Markers repeat every second while the program runs, including for late subscribers. The box indicates selection limits, not an obstacle or a reachability guarantee. Existing RViz displays may retain the last markers after exit.

## Installation

The host needs ROS 2 Jazzy, UR driver, MoveItPy, RealSense ROS driver, `ur3_bringup`, `ur3_moveit_scene`, `auto_calibration` and `easy_handeye2`. PyTorch, CUDA, MinkowskiEngine and AnyGrasp stay inside Docker; the ROS Python environment does not need model packages. Docker setup and licensing are documented in the [Docker guide](../docker+anygrasp/README.md).

Prepare these dependencies before building or running:

| Dependency | Installation / readiness |
| --- | --- |
| ROS, official UR driver, MoveItPy and RealSense | [Host setup](../ROS2+driver/ur3_ros2_setup.md), then [bringup](../ur3_bringup/README.md). |
| Scene package and this package | Included in the complete project under `~/ur3_ws/src`; keep one copy of each package. |
| `easy_handeye2_msgs`, ArUco and calibration tools | Download the message source in [host guide Section 4](../ROS2+driver/ur3_ros2_setup.md#handeye-message-dependency) and install dependencies through rosdep. |
| Robot-to-camera TF | Calibrate and keep one [hand-eye publisher](../auto_calibration/README.md) running. A fresh installation has no installation-specific calibration file. |
| Docker CLI, NVIDIA runtime, licensed SDK and detection weights | Complete [Docker setup](../docker+anygrasp/README.md), Sections 3–7, then its import/GPU and official-data checks. |
| Robotiq TCP server | Enable the gripper socket service on the UR controller and configure the robot IP/port. The Python socket client is included; no separate ROS 1 gripper node is needed. |

A successful official-data demo validates the model environment; live operation also requires fresh camera data and TF. `plan_only` skips hardware commands but still requires the live-inference dependencies above.

The existing `anygrasp-fixed` container must have a working license and:

- `/opt/anygrasp_sdk/grasp_detection/gsnet.so`, matching its Python version.
- `license/` in that SDK directory.
- `log/checkpoint_detection.tar`, the detection weights.

The program can start an existing stopped container. It does not recreate it, alter its MAC or copy license files into the repository.

Sections 1–4 of the [host guide](../ROS2+driver/ur3_ros2_setup.md) already build this package; do not repeat apt, rosdep or colcon here. Prepare Docker, licensing/weights and calibration TF, then follow the launch sequence below.

## Start the system

Terminal 1: enable External Control on the robot, then start the driver, camera/cloud, MoveIt, RViz and shared virtual table:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup bringup.launch.py robot_ip:=192.168.86.7
```

Add `launch_camera:=false` if another terminal already runs the camera driver. Avoid duplicate camera nodes.

Terminal 2: keep one calibration publisher running. Start it only if absent:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch auto_calibration publish.launch.py name:=ur3_realsense_eob tracking_root_frame:=camera_link
```

Terminal 3: authenticate Docker in this terminal, then run the ROS node as your normal user:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
sudo -v
ros2 run ur3_pick_up pick_up
```

If your account can already run Docker directly, omit `sudo -v` and append `--ros-args -p docker_use_sudo:=false`.

To display the workspace, infer and plan without sending any arm/gripper commands:

```bash
sudo -v
ros2 run ur3_pick_up pick_up --ros-args -p plan_only:=true
```

Camera, calibration TF, MoveIt and the table are still required. This captures and plans from the current arm state, skipping initial home motion.

## Preflight checks and operator prompts

Keep the bringup and calibration publisher terminals running. In the pick_up terminal, check the controller and cloud before launching:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 control list_controllers
ros2 topic info /camera/camera/depth/color/points
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

Verify that `scaled_joint_trajectory_controller` is `active`, the cloud has a publisher, and TF is available continuously. `tf2_echo` keeps running; press Ctrl+C after checking it, then launch pick_up. First check that the cloud and robot align in RViz and that the workspace box covers the intended physical tabletop area.

| Stage | Display or prompt | Operator action and result |
| --- | --- | --- |
| Initial home planned | `Move the arm to initial state 'home'? Enter y:` | Inspect the initial RViz path. Enter `y` to move; any other answer cancels the task. |
| Home reached | Fresh capture, scene/ROI point counts and log directory | Wait for one Docker inference. No candidate index entry is required. |
| Inference and filtering completed | Candidate index, score, width and camera-coordinate TCP | Valid poses are tried by descending score until both outbound and return-to-home plans succeed for one pose. |
| Both paths planned | `Confirm arm/gripper execution? Enter y:` | Inspect target orientation and both paths. Enter `y` to start the candidate task; any other answer cancels later actions. Initial home may already have completed. |
| Completed | Home arrival and gripper release result | The invocation ends. To grasp another object, run the launch command again for a fresh capture and inference. |
| Any stage fails | Stage name and reason | Later steps stop. Inspect the message and exchange logs, resolve the cause and relaunch. |

`plan_only:=true` skips both execution prompts and initial home motion. It still attempts cloud acquisition, Docker inference and planning from the current arm state.

## Task sequence

1. Display the workspace and check the shared table, container and SDK/checkpoint paths.
2. Plan initial home, display its path and ask `Move the arm to initial state 'home'? Enter y:`.
3. After arrival, wait 0.5 s and capture a new PointCloud2. Accept only samples captured after acquisition began and within `cloud_max_age`.
4. Use the actual message `header.frame_id` and capture-time TF to transform into `camera_color_optical_frame`. Use the same capture-time camera-to-base TF for workspace selection; do not relabel coordinates.
5. Remove invalid/out-of-depth points, voxel downsample and cap the point count. Keep surrounding scene points for SDK gripper collision checks and send a workspace boolean mask. Reject insufficient ROI points, including empty masks: the SDK treats an all-false mask as unrestricted steering.
6. Run one inference with `dense_grasp=false`, `collision_detection=true` and the workspace `region_steering` mask, followed by NMS and score sorting. SAM is unnecessary, but object names are not recognized.
7. Validate opening, rotations and bounds again on the host. Both the raw grasp center and depth-adjusted TCP must be inside the workspace. Sort valid poses by descending score and plan both outbound and return-to-home paths for each pose. If either segment fails, try the next-highest-scoring pose. Select the first pose for which both segments succeed. If every valid pose fails, print `All candidate poses failed to plan.` and end the candidate task. These planning attempts send no arm or gripper commands.
8. All poses use the same captured cloud and capture-time TF. Check camera TF before each attempt and after selecting a pose. Only after both plans succeed for the selected pose, ask `Confirm arm/gripper execution? Enter y:`. Check TF again after confirmation; a change above 3 mm or 0.5° cancels the task.
9. Move to candidate → open POS=0 → close POS=255 → return home → open POS=0 to release the object. Failures identify their stage and stop later steps. A gripper failure does not trigger an automatic return. Release is commanded only after successful return-to-home execution. A return failure skips release. If opening at home stops on contact before reaching the open position, the release stage reports failure.

The result describes a stationary captured scene. Keep the camera and objects still during inference and confirmation. This workflow assumes a fixed eye-on-base camera. SDK collision filtering concerns grasp geometry in the input cloud; it does not insert that cloud into MoveIt's full-arm collision scene. Arm paths use the existing MoveIt scene.

## Coordinates and TCP

The SDK receives an N×3 camera-coordinate cloud in meters. Optical +X points right in the image, +Y down and +Z forward from the lens. Workspace limits are defined separately in base_link. Terminal TCP positions use the optical frame.

The installed `config/grasp_tool.example.json` supplies the current Robotiq 2F-85 configuration: TCP is tool0 +Z 0.174 m, with parallel axes and a maximum opening of 0.085 m.

```text
p_TCP_camera = translation + depth * R_grasp[:, 0]
R_TCP_camera = R_grasp * grasp_to_tcp_rotation
T_base_tool0 = T_base_camera * T_camera_grasp_tip * T_grasp_tip_tool0
```

The rotated tool-axis offset is compensated once. Candidate width filters feasibility; arrival actions still fully open/close rather than mapping width to a Robotiq POS. POS=255 requests full closure; contact or a calibrated endpoint can produce a different actual POS.

## ROS parameters

Pass overrides after `--ros-args -p name:=value`. Example workspace:

```bash
ros2 run ur3_pick_up pick_up --ros-args \
  -p workspace_min:='[-0.30, 0.0, 0.02]' \
  -p workspace_max:='[0.30, 0.60, 0.60]'
```

| Parameter | Default | Purpose |
| --- | --- | --- |
| `workspace_min` / `workspace_max` | XYZ limits above | Three-element bounds in base_frame, meters. |
| `pointcloud_topic` | `/camera/camera/depth/color/points` | New PointCloud2 source, transformed from its true header frame. |
| `cloud_timeout` / `cloud_max_age` | `10.0` / `1.0` | Acquisition timeout / maximum sample age, seconds. |
| `voxel_size` | `0.005` | Voxel spacing, meters. |
| `max_points` / `min_roi_points` | `120000` / `50` | Full scene point cap / minimum ROI count after downsampling. |
| `max_camera_depth` | `2.0` | Maximum optical Z, meters. |
| `docker_container` / `docker_use_sudo` | `anygrasp-fixed` / `true` | Existing container / invoke Docker via sudo -n. |
| `docker_sdk_dir` | `/opt/anygrasp_sdk/grasp_detection` | Container SDK/license working directory. |
| `checkpoint_path` | Absolute SDK `log/checkpoint_detection.tar` path | Container detection weights. |
| `inference_timeout` | `120.0` | Maximum inference seconds; attempts to terminate that worker on timeout. |
| `gripper_height` | `0.03` | Finger height for SDK collision filtering, meters. |
| `exchange_dir` | `~/anygrasp_docker/exchange` | Unique per-run directories containing clouds, TF, results and logs. |
| `plan_only` | `false` | Capture, infer and plan without any arm/gripper commands. |
| `grasp_tool_config` | Installed tool JSON | Grasp/TCP axis mapping, tool transform and physical opening limit. |
| `base_frame` / `tool_link` | `base_link` / `tool0` | Workspace/planning frame and planning tip; current hardware requires tool0. |
| `tcp_offset` | `0.174` | tool0 +Z to TCP, meters; must match the hardware. |
| `planning_group` | `ur_manipulator` | MoveIt group. |
| `robot_ip` / `gripper_port` | `192.168.86.7` / `63352` | Gripper server; configure the arm IP separately in bringup. |
| `gripper_backend` | `tcp` | `none` skips gripper commands but still moves the arm after confirmation. |
| `max_velocity_scaling_factor` / `max_acceleration_scaling_factor` | `0.1` / `0.1` | MoveIt trajectory scaling in (0,1]. |

OMPL uses RRTConnect, 5 s planning time and 10 configured planning attempts. Bounds do not guarantee reachable orientations.

If inference runs out of memory on the 6 GB GPU, try a lower point cap and coarser voxels without motion:

```bash
sudo -v
ros2 run ur3_pick_up pick_up --ros-args \
  -p plan_only:=true \
  -p max_points:=60000 \
  -p voxel_size:=0.008 \
  -p inference_timeout:=180.0
```

Downsampling changes small-object detail and grasp predictions. The remaining workspace points must still satisfy `min_roi_points`.


## Troubleshooting and verification

- No cloud: check bringup, camera connection and the actual topic with `ros2 topic list`.
- Empty workspace: inspect the RViz box, cloud alignment, calibration and TF. Changing only a frame string does not transform points.
- Docker permission denied / sudo requires a password: run `sudo -v` in the same terminal before launching. Reauthenticate if the credential expired during a long wait.
- CUDA OOM: reduce `max_points`, increase `voxel_size` or reduce `max_camera_depth`. An empty resulting ROI still stops inference.
- Outbound/return planning failure: try the next valid pose by score until one succeeds or all are exhausted. Exhaustion prints `All candidate poses failed to plan.` Initial home may already have completed. Once execution has begun after confirmation, an execution failure stops later steps without trying other poses.
- Exchange UUID directories retain `cloud.npz` (points, mask and capture metadata), `capture.json` (TF/bounds), `grasps.json` (successful response) and `inference.log`. Container temporary files are removed; periodically clean host logs yourself.
- A Segmentation fault after `Deleting MoveItCpp` is an existing native cleanup issue; determine task status from individual stage results.

Use `plan_only:=true` to check fresh cloud acquisition, Docker inference, workspace selection and outbound/return planning before execution. Camera, calibration TF, MoveIt and the table remain required, but no arm/gripper commands are sent. Verify real camera/GPU operation, calibration accuracy and robot execution for the physical installation.

API reference: [official AnyGrasp usage](https://github.com/graspnet/anygrasp_sdk/blob/main/grasp_detection/USAGE.md).

## Exit status

`pick_up` returns `0` only when the task completes, or when both paths are successfully planned in `plan_only` mode. Failure to capture a cloud, infer, find a plannable pose, or execute the task returns `1`; declining an execution confirmation also returns `1`. Startup/configuration errors return `2`. Ctrl+C or closed interactive input returns `130`.

A native-library crash is reported by the operating system separately; these application codes do not suppress a MoveItCpp segmentation fault.

<a id="file-inventory"></a>
## File inventory

Every source file in this folder and its subfolders is listed below, relative to this folder. Build outputs, runtime logs and caches are excluded.

| File | Purpose |
| --- | --- |
| [README.md](README.md) | English guide: Live AnyGrasp picking, workspace, startup order, TCP, parameters, failure handling and exit codes. |
| [README.zh-TW.md](README.zh-TW.md) | Traditional Chinese guide: Live AnyGrasp picking, workspace, startup order, TCP, parameters, failure handling and exit codes. |
| [config/grasp_tool.example.json](config/grasp_tool.example.json) | Robotiq tool configuration template: tool0, application TCP offset, grasp-axis mapping and 0.085 m maximum opening. |
| [package.xml](package.xml) | ROS package identity, version, license and build/runtime dependency declarations. |
| [resource/ur3_pick_up](resource/ur3_pick_up) | Package marker in the ament index; required for ROS discovery even when empty. |
| [scripts/anygrasp_worker.py](scripts/anygrasp_worker.py) | Container worker for one cloud request; load licensed SDK/weights and return candidates/request metadata without robot control. |
| [setup.cfg](setup.cfg) | Install registered executables under lib/ur3_pick_up for ros2 run discovery. |
| [setup.py](setup.py) | Register pick_up and install tool settings, the container inference worker and bilingual READMEs. |
| [ur3_pick_up/__init__.py](ur3_pick_up/__init__.py) | Python package marker for live picking. |
| [ur3_pick_up/anygrasp_input.py](ur3_pick_up/anygrasp_input.py) | Parse/validate AnyGrasp/GraspNet candidates and tool settings; handle depth, axis mapping and TCP pose conversion. |
| [ur3_pick_up/anygrasp_task.py](ur3_pick_up/anygrasp_task.py) | Coordinate capture/inference, try ranked plannable poses and execute confirmed picking, return home and release. |
| [ur3_pick_up/camera_capture.py](ur3_pick_up/camera_capture.py) | Receive a fresh PointCloud2, check timing/frame and capture its corresponding TF snapshot. |
| [ur3_pick_up/docker_client.py](ur3_pick_up/docker_client.py) | Check/start the existing container and exchange inference requests/results through files and docker cp/exec. |
| [ur3_pick_up/live_input.py](ur3_pick_up/live_input.py) | Decode/transform clouds, apply workspace/voxel/count filters, rank candidates and check fixed-camera TF consistency. |
| [ur3_pick_up/pick_up.py](ur3_pick_up/pick_up.py) | pick_up entry point, ROS parameters, MoveIt planning/execution, TCP/path markers and Robotiq socket control. |

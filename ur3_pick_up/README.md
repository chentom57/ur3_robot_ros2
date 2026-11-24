# UR3 live-camera AnyGrasp pick-up

[繁體中文](README.zh-TW.md)

`ros2 run ur3_pick_up pick_up` records the arm's current joint position as the task start/return target, then opens a live color view where two clicks define opposite corners of a rectangle around one target object. After selection, the GUI stays open through inference, planning and motion. It asks for pick confirmation and, after the arm returns to the recorded start position, waits for a GUI release command before opening the gripper. The rectangle mask maps onto cloud points and steers AnyGrasp candidate generation; the full scene cloud remains available for collision checking. Valid workspace poses are ranked by score and tried until outbound and return-to-start paths both plan.

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

Camera, calibration TF, MoveIt and the table are still required. The arm stays in its current position. The program records its joint state at startup and uses it as the planning start and final return target.

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
| Startup | Record the current joint state as the start and return target | The arm does not move at startup. |
| Ready | A window shows the live RGB stream | Click two opposite corners of a rectangle; `Enter` accepts, right click or `U` undoes a point, `R` resets, and `Esc` cancels. The window stays open. A fresh synchronized image/cloud pair is captured after acceptance. |
| Selection accepted | Show selected-object/workspace counts, scene count and exchange path | The selected region intersected with workspace steers candidates; the full cloud remains for collision checking. |
| Inference and filtering completed | Candidate index, score, width and camera-coordinate TCP | Valid poses are tried by descending score until outbound and return-to-start plans succeed for one pose. |
| Both paths planned | GUI buttons `EXECUTE PICK` and `CANCEL` | Inspect target orientation and both paths. Confirm in the GUI (or press `Y`) to start; cancel (or press `N`) to stop. The arm remains at the recorded startup position until confirmed execution. |
| Returned to start | GUI buttons `OPEN GRIPPER / RELEASE` and `ABORT / KEEP CLOSED` | The gripper remains closed while awaiting a decision. Confirm release in the GUI (or press `O`); abort (or press `Q`) leaves the gripper closed. The GUI closes after release finishes or the task stops. |
| Completed | Return to the recorded startup position and gripper release result | The invocation ends. To grasp another object, run the command again for a fresh capture and inference. |
| Any stage fails | Stage name and reason | Later steps stop. Inspect the message and exchange logs, resolve the cause and relaunch. |

`plan_only:=true` skips grasp execution. It still records the startup joint state, then attempts cloud acquisition, GUI selection, Docker inference and planning without moving the arm.

## Task sequence

1. Display the workspace and check the shared table, container and SDK/checkpoint paths.
2. Record the current joint state as the task start and return target. The arm does not move before target selection or planning.
3. Wait for a fresh color image and PointCloud2 pair (at most 0.15 seconds apart and no older than `cloud_max_age`), and receive color CameraInfo.
4. Click two opposite corners of a rectangle around one target object in the live OpenCV view. After Enter, acquire a fresh synchronized image/cloud pair and CameraInfo. Camera intrinsics project cloud points into the raw image, and the selected image mask selects the corresponding points; cloud and image resolutions/layouts need not match. Cancel or too few selected points stops processing. This manual mask is not semantic segmentation; keep the arm and nearby objects outside the rectangle.
5. Use the actual cloud `header.frame_id` and capture-time TF to transform into `camera_color_optical_frame`. Filter invalid points/depth, voxel-downsample and cap the full scene. Intersect the selected-object mask with workspace for candidate generation while retaining the full scene for SDK collision checking.
6. Run one inference with `dense_grasp=false`, `collision_detection=true` and `region_steering=selected pixel mask mapped to cloud points`, followed by NMS and score sorting. This is manual region selection; SAM is not installed or called.
7. Validate opening, rotations and bounds again on the host. Both the raw grasp center and depth-adjusted TCP must be inside the workspace. Sort valid poses by descending score and plan both the outbound path and the return path to the recorded startup joint state. If either segment fails, try the next-highest-scoring pose. Select the first pose for which both segments succeed. If every valid pose fails, print a summary with success/failure counts, each failed candidate's stage and confirmation that no grasp trajectory was executed. These planning attempts send no arm or gripper commands.
8. All poses use the same captured cloud and capture-time TF. Check camera TF before each attempt and after selecting a pose. Only after both plans succeed for the selected pose, ask for confirmation in the still-open camera GUI. Check TF again after confirmation; a change above 3 mm or 0.5° cancels the task.
9. After execution confirmation, open POS=0 at the recorded startup position and verify the gripper is fully open before arm motion. Move to the candidate and close POS=255 at the grasp pose, then return to the recorded startup position. Keep the gripper closed and wait for the operator to choose `OPEN GRIPPER / RELEASE` in the GUI; only then command POS=0 to release. Failures identify their stage and stop later steps. A gripper failure does not trigger an automatic return. If the gripper cannot fully open before outbound motion, the arm does not move. A return failure skips the release prompt. If opening at the startup position stops on contact before reaching the open position, the release stage reports failure. The GUI closes after release completes or the workflow aborts.

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
| `color_image_topic` | `/camera/camera/color/image_raw` | Raw color image within 0.15 seconds of cloud capture. |
| `color_camera_info_topic` | `/camera/camera/color/camera_info` | Camera intrinsics used to project cloud points into the raw color image. |
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
- `Truncated PointCloud2 data or invalid row_step`: the decoder accepts a packed single-row cloud when its payload contains every point, even if the publisher leaves a stale larger `row_step`. For other malformed layouts, the error reports width, height, point step, row step and payload size; inspect the active publisher/topic.
- Empty workspace: inspect the RViz box, cloud alignment, calibration and TF. Changing only a frame string does not transform points.
- Docker permission denied / sudo requires a password: run `sudo -v` in the same terminal before launching. Reauthenticate if the credential expired during a long wait.
- CUDA OOM: reduce `max_points`, increase `voxel_size` or reduce `max_camera_depth`. An empty resulting ROI still stops inference.
- Outbound/return planning failure: try the next valid pose by score until one succeeds or all are exhausted. Exhaustion prints `All candidate poses failed to plan.` The arm remains at the recorded startup position until confirmed execution. Once execution has begun after confirmation, an execution failure stops later steps without trying other poses.
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
| [ur3_pick_up/anygrasp_task.py](ur3_pick_up/anygrasp_task.py) | Coordinate capture/inference, try ranked plannable poses and execute confirmed picking, return to the recorded startup position and release. |
| [ur3_pick_up/camera_capture.py](ur3_pick_up/camera_capture.py) | Receive a fresh PointCloud2, check timing/frame and capture its corresponding TF snapshot. |
| [ur3_pick_up/docker_client.py](ur3_pick_up/docker_client.py) | Check/start the existing container and exchange inference requests/results through files and docker cp/exec. |
| [ur3_pick_up/live_input.py](ur3_pick_up/live_input.py) | Decode/transform clouds, apply workspace/voxel/count filters, rank candidates and check fixed-camera TF consistency. |
| [ur3_pick_up/pick_up.py](ur3_pick_up/pick_up.py) | pick_up entry point, ROS parameters, MoveIt planning/execution, TCP/path markers and Robotiq socket control. |
| [ur3_pick_up/target_selection.py](ur3_pick_up/target_selection.py) | Persistent live-camera GUI for rectangle selection, pick confirmation and delayed gripper-release approval. |

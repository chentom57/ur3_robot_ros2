# UR3 Vision-Guided Pick-Up

[繁體中文](readme.zh-TW.md)

This project combines a UR3, a Robotiq 2F-85 gripper, a RealSense depth camera, ROS 2, MoveIt and AnyGrasp. It provides source code and guides for environment setup, hand-eye calibration, grasp inference and robot execution. ROS and robot control run on the host; AnyGrasp runs in a GPU-enabled Docker container.

ROS communication, camera capture, coordinate transforms, planning and hardware control run on the host. This repository contains shareable source code and documentation; build and run the ROS packages in your own workspace.

## Project goal

The goal is to recognize a target object in the camera view and have the robot grasp it. The intended workflow is object recognition, target localization or segmentation, grasp-pose estimation, coordinate conversion, motion planning and physical grasping.

The current implementation provides live point-cloud capture, AnyGrasp grasp proposals, coordinate conversion, planning and confirmed execution. Object recognition and the connection between a recognized target and its grasp candidates remain to be integrated. At present, candidate selection uses geometry, workspace bounds, scores and planning feasibility.

## Features

| Feature | Description |
| --- | --- |
| System bringup | Starts the robot driver, MoveIt, RViz, a virtual table and an optional camera with physical or mock hardware. |
| Hand-eye calibration | Samples robot poses, computes and saves calibration, and publishes or reloads the robot-to-camera TF. The default setup is eye-on-base. |
| Live grasp inference | Captures a fresh cloud and passes it with a workspace mask into Docker to obtain grasp candidates. |
| Candidate planning | Tries valid candidates by descending score and selects the first with both outbound and return-to-home plans. |
| Task execution | Requires confirmation before execution, then follows initial home, target pose, open, close, return home and release. |
| Visualization | Displays the workspace, TCP target and axes, paths and table. The workspace is a selection volume; the table is a separate collision object. |
| Planning only | `plan_only:=true` captures, infers and plans without sending arm or gripper commands. |

Each `pick_up` invocation captures one scene and executes at most one candidate. Continuous picking and selection by object name are not implemented. AnyGrasp proposes grasps from geometry; SAM and semantic object recognition are not integrated.

## Folders and tools

| Folder | Purpose | Tools | Guides |
| --- | --- | --- | --- |
| `ROS2+driver/` | Host, robot and camera setup guides; driver sources are obtained separately. | Ubuntu, ROS 2 Jazzy, UR ROS 2 Driver, RealSense SDK/ROS driver, rosdep, colcon, network diagnostics | [Guide](ROS2+driver/ur3_ros2_setup.md) |
| `ur3_moveit_scene/` | Shared collision table, workspace geometry and preview. | Python, rclpy, MoveIt Planning Scene, RViz Marker/MarkerArray, NumPy | [Guide](ur3_moveit_scene/README.md) |
| `ur3_bringup/` | Combined launch, hardware-mode selection, and the project MoveIt/RViz integration. | ROS 2 launch, UR driver, MoveIt, RViz, RealSense, CMake | [Guide](ur3_bringup/README.md) |
| `easy_handeye2/` | Generic sampling, OpenCV computation, persistence and manual/evaluation GUIs, with necessary fixes retained. | Python, TF2, OpenCV, Qt/rqt, NumPy, transforms3d | [Guide](easy_handeye2/README.md) |
| `auto_calibration/` | UR3 automatic sampling and TF publication management, using easy_handeye2 services. | MoveItPy, ArUco, TF2, ROS services and child-process management | [Guide](auto_calibration/README.md) |
| `docker+anygrasp/` | Docker builds, GPU and SDK setup/licensing guides, and a single-inference official-data demo. | Docker Engine, NVIDIA Container Toolkit, CUDA, PyTorch, MinkowskiEngine, PointNet2, GraspNetAPI, AnyGrasp SDK | [Guide](docker+anygrasp/README.md) |
| `ur3_pick_up/` | Live picking application: cloud capture, processing, Docker transport, pose conversion, planning and gripper control. | Python, rclpy, PointCloud2, TF2, NumPy, MoveItPy, Docker CLI, Robotiq TCP socket | [Guide](ur3_pick_up/README.md) |

The host copies `ur3_pick_up/scripts/anygrasp_worker.py` into Docker for inference. Docker does not subscribe directly to the ROS camera topic. `demo_once.py` uses the SDK's example scene rather than the live camera scene.

## Environment and data locations

| Item | Current configuration |
| --- | --- |
| Host | Ubuntu 24.04, ROS 2 Jazzy, Python 3.12 |
| Hardware | UR3, Robotiq 2F-85, RealSense D435; fixed camera |
| Model environment | Python 3.11, PyTorch 2.5.1 + CUDA 12.1, MinkowskiEngine 0.5.4 inside Docker |
| GPU build target | RTX 3060 Laptop; CUDA architecture `8.6` in the Dockerfile. Adjust for other GPUs. |
| ROS workspace | `~/ur3_ws`: runtime sources and build outputs |
| Model workspace | `~/anygrasp_docker`: Docker builds, exchanged clouds and inference logs |
| Default container | `anygrasp-fixed` |
| Calibration files | `~/.ros2/easy_handeye2/calibrations/` |
| Planning group | `ur_manipulator`; default named return state: `home` |
| Camera frame | `camera_color_optical_frame` |
| Cloud topic | `/camera/camera/depth/color/points` |
| Grasp TCP | Default offset of 0.174 m along tool0 +Z; axis mapping is defined in the tool configuration. |

Set hardware addresses, TCP, camera placement and workspace bounds for your installation. Obtain the AnyGrasp SDK, license and model weights separately as described in the Docker guide; they are not bundled here.

## Dependency setup

Place one complete repository at `~/ur3_ws/src/robot_arm` (the name is arbitrary). `colcon` recursively discovers five ROS packages: `easy_handeye2`, `auto_calibration`, `ur3_bringup`, `ur3_moveit_scene` and `ur3_pick_up`. Keep each package name only once; do not also copy its subfolders into `src`.

| Dependency | Where to install it |
| --- | --- |
| ROS 2 Jazzy, rosdep, colcon and host Python libraries | [Host setup](ROS2+driver/ur3_ros2_setup.md), Sections 1–4; Python dependencies are installed by rosdep in Section 4. |
| Official UR driver and `ur_moveit_config` | GitHub `jazzy` checkout in [host setup](ROS2+driver/ur3_ros2_setup.md), Section 4. rosdep supplies UR description/client-library dependencies. |
| RealSense ROS 4.59.1 and system SDK | Source checkout in host setup; SDK 2.59.0 or later, currently 2.59.1. Use `--skip-keys librealsense2` with rosdep to avoid a second SDK. |
| `easy_handeye2_msgs` 0.5.0 | Pinned sparse checkout in [host guide Section 4](ROS2+driver/ur3_ros2_setup.md#handeye-message-dependency); it excludes the upstream implementation to prevent duplicate packages. |
| ArUco, MoveItPy, Qt/rqt, OpenCV, NumPy, transforms3d and YAML | Installed from manifests by the single rosdep command in [host guide Section 4](ROS2+driver/ur3_ros2_setup.md). |
| GPU Docker, AnyGrasp SDK, license and detection weights | [Docker guide](docker+anygrasp/README.md), Sections 3–7. These are required for pick-up inference, not calibration or workspace display. |

Complete source downloads before running rosdep. Initialize rosdep once with `sudo rosdep init`, run `rosdep update`, then resolve dependencies and build from `~/ur3_ws` as described in the host guide. Stop and resolve any rosdep errors before building.

A typical source layout is:

```text
~/ur3_ws/src/
├── robot_arm/                       # this complete repository
│   ├── easy_handeye2/                # generic services and manual GUI
│   ├── auto_calibration/             # UR3 automation and TF publication
│   ├── ur3_bringup/
│   ├── ur3_moveit_scene/
│   └── ur3_pick_up/
├── Universal_Robots_ROS2_Driver/     # official GitHub source
├── realsense-ros/                   # official source, tag 4.59.1
└── easy_handeye2_interfaces/
    └── easy_handeye2_msgs/          # only the message package
```

Existing workspaces with packages in separate folders can retain that layout. Run `colcon list` to inspect package names before adding another source copy.

## Start-from-scratch reading order

**Start with the [ROS2+driver environment guide](ROS2+driver/ur3_ros2_setup.md), then follow the order below.** Its entry documents are named `ur3_ros2_setup.md` and `ur3_ros2_setup.zh-TW.md`, rather than `README.md`.

| Step | Read | Outcome |
| --- | --- | --- |
| 1 | `ROS2+driver/`：[Guide](ROS2+driver/ur3_ros2_setup.md) | Install ROS and drivers, create the workspace and configure networking. |
| 2 | `ur3_moveit_scene/`：[Guide](ur3_moveit_scene/README.md) | Review the built scene package, tabletop height and collision settings. |
| 3 | `ur3_bringup/`：[Guide](ur3_bringup/README.md) | Start the system and verify robot state, visualization and camera streams. |
| 4 | `auto_calibration/`：[Guide](auto_calibration/README.md) | Check calibration readiness, sample with a rigid marker, save calibration and publish TF. |
| 5 | `docker+anygrasp/`：[Guide](docker+anygrasp/README.md) | Set up GPU Docker, build the SDK, obtain licensing and weights, and run the official-data demo. This step can be prepared independently. |
| 6 | `ur3_pick_up/`：[Guide](ur3_pick_up/README.md) | Configure TCP and workspace, verify the live pipeline with planning only, then execute a physical task. |

For manual calibration or generic service details, read the [easy_handeye2 guide](easy_handeye2/README.md). Normal automatic operation uses `auto_calibration` and does not require manually starting its core GUI.

Complete host guide Sections 1–4 once, then read each package README for behavior, operation and parameters. Download official UR, RealSense ROS and `easy_handeye2_msgs` sources first; rosdep installs ROS/Python dependencies, then colcon builds them. The system RealSense SDK, GPU Docker and AnyGrasp licensing/weights have their own required preparation steps. Copying this repository alone does not provision the complete runtime environment.

## Validation by available equipment

| Equipment | Available workflow |
| --- | --- |
| Neither robot nor camera | Mock bringup and visualization; the official-data demo can run separately with GPU Docker. |
| Camera only | Use `use_mock_hardware:=true launch_camera:=true`, verify the cloud, then use calibration TF and `plan_only:=true` to inspect capture, inference and virtual planning. An old calibration cannot validate positioning accuracy after moving the camera. |
| Robot and camera | Calibrate, plan and execute a confirmed physical pick. |

`plan_only:=true` does not generate synthetic clouds. It still requires camera data, TF, a MoveIt scene and a working AnyGrasp container.

<a id="file-inventory"></a>
## File inventory

Source files below use paths relative to the project root. Git internals, build outputs, runtime logs and caches are excluded.

| File | Purpose |
| --- | --- |
| [.gitignore](.gitignore) | Git ignore rules for build outputs, caches and local artifacts. |
| [AGENTS.md](AGENTS.md) | Project contributor instructions, including the destination for final PowerPoint files. |
| [ROS2+driver/ur3_ros2_setup.md](ROS2+driver/ur3_ros2_setup.md) | English setup from scratch: ROS, upstream sources, system SDK, rosdep, build and connectivity checks. |
| [ROS2+driver/ur3_ros2_setup.zh-TW.md](ROS2+driver/ur3_ros2_setup.zh-TW.md) | Traditional Chinese setup from scratch: ROS, upstream sources, system SDK, rosdep, build and connectivity checks. |
| [auto_calibration/LICENSE.md](auto_calibration/LICENSE.md) | Bundled LGPLv3 terms and license text. |
| [auto_calibration/README.md](auto_calibration/README.md) | English guide: Automatic calibration, startup cleanup, parameters, TF publication and operation checks. |
| [auto_calibration/README.zh-TW.md](auto_calibration/README.zh-TW.md) | Traditional Chinese guide: Automatic calibration, startup cleanup, parameters, TF publication and operation checks. |
| [auto_calibration/auto_calibration/__init__.py](auto_calibration/auto_calibration/__init__.py) | Python package marker and description for automatic calibration. |
| [auto_calibration/auto_calibration/calibrate.py](auto_calibration/auto_calibration/calibrate.py) | Automatic calibration entry point coordinating cleanup, tracker, services, prompts, motion worker and TF publication. |
| [auto_calibration/auto_calibration/motion.py](auto_calibration/auto_calibration/motion.py) | Service clients, 15-pose generation, planning diagnostics/retries, motion, sample reset and compute/save workflow. |
| [auto_calibration/auto_calibration/publisher.py](auto_calibration/auto_calibration/publisher.py) | Load calibration, compose/publish camera-root TF, enforce one publisher and provide the reload service. |
| [auto_calibration/auto_calibration/startup.py](auto_calibration/auto_calibration/startup.py) | Identify/stop previous calibration and same-camera tracker processes; check ROS conflicts and locks before startup. |
| [auto_calibration/launch/publish.launch.py](auto_calibration/launch/publish.launch.py) | Launch the saved-calibration TF publisher with calibration-name and camera-root arguments. |
| [auto_calibration/package.xml](auto_calibration/package.xml) | ROS package identity, version, license and build/runtime dependency declarations. |
| [auto_calibration/resource/auto_calibration](auto_calibration/resource/auto_calibration) | Package marker in the ament index; required for ROS discovery even when empty. |
| [auto_calibration/setup.cfg](auto_calibration/setup.cfg) | Install registered executables under lib/auto_calibration for ros2 run discovery. |
| [auto_calibration/setup.py](auto_calibration/setup.py) | Register calibrate/handeye_publisher commands and install launch, license and bilingual READMEs. |
| [docker+anygrasp/.dockerignore](docker+anygrasp/.dockerignore) | Exclude licenses, weight archives, results and caches from the Docker build context. |
| [docker+anygrasp/.gitignore](docker+anygrasp/.gitignore) | Keep local SDK assets, licenses, weight archives, inference outputs and caches out of Git. |
| [docker+anygrasp/Dockerfile](docker+anygrasp/Dockerfile) | Build the PyTorch/CUDA base, configure GPU architecture and compile the specified MinkowskiEngine fork/branch. |
| [docker+anygrasp/Dockerfile.sdk](docker+anygrasp/Dockerfile.sdk) | Extend the MinkowskiEngine image with SDK, PointNet2, GraspNetAPI, Open3D dependencies and required system libraries. |
| [docker+anygrasp/README.md](docker+anygrasp/README.md) | English guide: Docker/GPU installation, SDK build, licensing/weights, inference and container operations. |
| [docker+anygrasp/README.zh-TW.md](docker+anygrasp/README.zh-TW.md) | Traditional Chinese guide: Docker/GPU installation, SDK build, licensing/weights, inference and container operations. |
| [docker+anygrasp/demo_once.py](docker+anygrasp/demo_once.py) | Infer once on the SDK example scene and save candidates for environment validation; does not use the live camera. |
| [easy_handeye2/LICENSE.md](easy_handeye2/LICENSE.md) | Bundled LGPLv3 terms and license text. |
| [easy_handeye2/README.md](easy_handeye2/README.md) | English guide: Generic services, manual/evaluation GUIs and saved-result usage. |
| [easy_handeye2/README.zh-TW.md](easy_handeye2/README.zh-TW.md) | Traditional Chinese guide: Generic services, manual/evaluation GUIs and saved-result usage. |
| [easy_handeye2/easy_handeye2/__init__.py](easy_handeye2/easy_handeye2/__init__.py) | Define sample/calibration directories and service names; provide local resource ownership locks. |
| [easy_handeye2/easy_handeye2/handeye_calibration.py](easy_handeye2/easy_handeye2/handeye_calibration.py) | Declare/read calibration parameters, resolve calibration paths and load/save YAML results. |
| [easy_handeye2/easy_handeye2/handeye_calibration_backend_opencv.py](easy_handeye2/easy_handeye2/handeye_calibration_backend_opencv.py) | Convert ROS samples to OpenCV inputs, run hand-eye algorithms and construct calibration results. |
| [easy_handeye2/easy_handeye2/handeye_client.py](easy_handeye2/easy_handeye2/handeye_client.py) | Service client for manual sampling, algorithm selection, computation and saving. |
| [easy_handeye2/easy_handeye2/handeye_rqt_calibrator.py](easy_handeye2/easy_handeye2/handeye_rqt_calibrator.py) | Register the manual-calibration rqt plugin, rqt_calibrator.py entry point and widget lifecycle. |
| [easy_handeye2/easy_handeye2/handeye_rqt_calibrator_widget.py](easy_handeye2/easy_handeye2/handeye_rqt_calibrator_widget.py) | Load manual-calibration UI, update pose/sample displays and handle sampling, computation and save controls. |
| [easy_handeye2/easy_handeye2/handeye_rqt_evaluator.py](easy_handeye2/easy_handeye2/handeye_rqt_evaluator.py) | Register the evaluation rqt plugin, rqt_evaluator.py entry point and widget lifecycle. |
| [easy_handeye2/easy_handeye2/handeye_rqt_evaluator_widget.py](easy_handeye2/easy_handeye2/handeye_rqt_evaluator_widget.py) | Load evaluation UI, read calibration/current TF and display poses and consistency information. |
| [easy_handeye2/easy_handeye2/handeye_sampler.py](easy_handeye2/easy_handeye2/handeye_sampler.py) | Acquire robot/marker TF, add/remove samples and load/save sample lists. |
| [easy_handeye2/easy_handeye2/handeye_server.py](easy_handeye2/easy_handeye2/handeye_server.py) | Serve sampling/sample management, algorithm selection, computation and saving; handle errors and duplicate servers. |
| [easy_handeye2/launch/calibrate.launch.py](easy_handeye2/launch/calibrate.launch.py) | Launch the manual server and rqt GUI using six calibration-name/type/frame arguments. |
| [easy_handeye2/launch/evaluate.launch.py](easy_handeye2/launch/evaluate.launch.py) | Launch the rqt evaluation interface for a named calibration. |
| [easy_handeye2/package.xml](easy_handeye2/package.xml) | ROS package identity, version, license and build/runtime dependency declarations. |
| [easy_handeye2/plugin_calibrator.xml](easy_handeye2/plugin_calibrator.xml) | Declare the manual-calibration plugin class and menu metadata for rqt. |
| [easy_handeye2/plugin_evaluator.xml](easy_handeye2/plugin_evaluator.xml) | Declare the evaluation plugin class and menu metadata for rqt. |
| [easy_handeye2/resource/easy_handeye2](easy_handeye2/resource/easy_handeye2) | Package marker in the ament index; required for ROS discovery even when empty. |
| [easy_handeye2/resource/rqt_handeye.ui](easy_handeye2/resource/rqt_handeye.ui) | Qt Designer manual-calibration layout with sample list and sampling/computation/save controls. |
| [easy_handeye2/resource/rqt_handeye_evaluator.ui](easy_handeye2/resource/rqt_handeye_evaluator.ui) | Qt Designer calibration-evaluation layout. |
| [easy_handeye2/resource/rqt_handeye_info.ui](easy_handeye2/resource/rqt_handeye_info.ui) | Shared calibration-parameter/frame information layout for calibration and evaluation widgets. |
| [easy_handeye2/setup.cfg](easy_handeye2/setup.cfg) | Install registered executables under lib/easy_handeye2 for ros2 run discovery. |
| [easy_handeye2/setup.py](easy_handeye2/setup.py) | Register the server/two rqt commands and install launches, GUI/plugin resources, license and bilingual READMEs. |
| [readme.md](readme.md) | English project overview, features, dependency setup, reading order and complete file inventory. |
| [readme.zh-TW.md](readme.zh-TW.md) | Traditional Chinese project overview, features, dependency setup, reading order and complete file inventory. |
| [ur3_bringup/CMakeLists.txt](ur3_bringup/CMakeLists.txt) | Define the ament_cmake package and install launch files, RViz configuration and bilingual READMEs. |
| [ur3_bringup/README.md](ur3_bringup/README.md) | English guide: Physical/mock startup, launch parameters, RViz, SDK troubleshooting and status checks. |
| [ur3_bringup/README.zh-TW.md](ur3_bringup/README.zh-TW.md) | Traditional Chinese guide: Physical/mock startup, launch parameters, RViz, SDK troubleshooting and status checks. |
| [ur3_bringup/config/moveit.rviz](ur3_bringup/config/moveit.rviz) | RViz displays for the robot, planning scene, table, workspace/path markers, image, cloud and TF. |
| [ur3_bringup/launch/bringup.launch.py](ur3_bringup/launch/bringup.launch.py) | Combine UR driver, MoveIt, RViz, table, workspace preview and camera for physical/mock hardware. |
| [ur3_bringup/launch/moveit.launch.py](ur3_bringup/launch/moveit.launch.py) | Include upstream MoveIt and manage the project RViz/collision table without duplicate launches. |
| [ur3_bringup/package.xml](ur3_bringup/package.xml) | ROS package identity, version, license and build/runtime dependency declarations. |
| [ur3_moveit_scene/README.md](ur3_moveit_scene/README.md) | English guide: Shared collision table, workspace visualization and scene parameters. |
| [ur3_moveit_scene/README.zh-TW.md](ur3_moveit_scene/README.zh-TW.md) | Traditional Chinese guide: Shared collision table, workspace visualization and scene parameters. |
| [ur3_moveit_scene/package.xml](ur3_moveit_scene/package.xml) | ROS package identity, version, license and build/runtime dependency declarations. |
| [ur3_moveit_scene/resource/ur3_moveit_scene](ur3_moveit_scene/resource/ur3_moveit_scene) | Package marker in the ament index; required for ROS discovery even when empty. |
| [ur3_moveit_scene/setup.cfg](ur3_moveit_scene/setup.cfg) | Install registered executables under lib/ur3_moveit_scene for ros2 run discovery. |
| [ur3_moveit_scene/setup.py](ur3_moveit_scene/setup.py) | Register virtual_table/workspace_preview and install the ROS package marker and bilingual READMEs. |
| [ur3_moveit_scene/ur3_moveit_scene/__init__.py](ur3_moveit_scene/ur3_moveit_scene/__init__.py) | Python package marker for shared scene utilities. |
| [ur3_moveit_scene/ur3_moveit_scene/virtual_table.py](ur3_moveit_scene/ur3_moveit_scene/virtual_table.py) | Add the virtual table to the MoveIt Planning Scene and publish scene updates/RViz markers. |
| [ur3_moveit_scene/ur3_moveit_scene/workspace.py](ur3_moveit_scene/ur3_moveit_scene/workspace.py) | Shared workspace bounds, point-in-volume checks and workspace MarkerArray geometry. |
| [ur3_moveit_scene/ur3_moveit_scene/workspace_preview.py](ur3_moveit_scene/ur3_moveit_scene/workspace_preview.py) | Independent bringup workspace preview, yielding marker publication to an active pick task and resuming afterward. |
| [ur3_pick_up/README.md](ur3_pick_up/README.md) | English guide: Live AnyGrasp picking, workspace, startup order, TCP, parameters, failure handling and exit codes. |
| [ur3_pick_up/README.zh-TW.md](ur3_pick_up/README.zh-TW.md) | Traditional Chinese guide: Live AnyGrasp picking, workspace, startup order, TCP, parameters, failure handling and exit codes. |
| [ur3_pick_up/config/grasp_tool.example.json](ur3_pick_up/config/grasp_tool.example.json) | Robotiq tool configuration template: tool0, application TCP offset, grasp-axis mapping and 0.085 m maximum opening. |
| [ur3_pick_up/package.xml](ur3_pick_up/package.xml) | ROS package identity, version, license and build/runtime dependency declarations. |
| [ur3_pick_up/resource/ur3_pick_up](ur3_pick_up/resource/ur3_pick_up) | Package marker in the ament index; required for ROS discovery even when empty. |
| [ur3_pick_up/scripts/anygrasp_worker.py](ur3_pick_up/scripts/anygrasp_worker.py) | Container worker for one cloud request; load licensed SDK/weights and return candidates/request metadata without robot control. |
| [ur3_pick_up/setup.cfg](ur3_pick_up/setup.cfg) | Install registered executables under lib/ur3_pick_up for ros2 run discovery. |
| [ur3_pick_up/setup.py](ur3_pick_up/setup.py) | Register pick_up and install tool settings, the container inference worker and bilingual READMEs. |
| [ur3_pick_up/ur3_pick_up/__init__.py](ur3_pick_up/ur3_pick_up/__init__.py) | Python package marker for live picking. |
| [ur3_pick_up/ur3_pick_up/anygrasp_input.py](ur3_pick_up/ur3_pick_up/anygrasp_input.py) | Parse/validate AnyGrasp/GraspNet candidates and tool settings; handle depth, axis mapping and TCP pose conversion. |
| [ur3_pick_up/ur3_pick_up/anygrasp_task.py](ur3_pick_up/ur3_pick_up/anygrasp_task.py) | Coordinate capture/inference, try ranked plannable poses and execute confirmed picking, return home and release. |
| [ur3_pick_up/ur3_pick_up/camera_capture.py](ur3_pick_up/ur3_pick_up/camera_capture.py) | Receive a fresh PointCloud2, check timing/frame and capture its corresponding TF snapshot. |
| [ur3_pick_up/ur3_pick_up/docker_client.py](ur3_pick_up/ur3_pick_up/docker_client.py) | Check/start the existing container and exchange inference requests/results through files and docker cp/exec. |
| [ur3_pick_up/ur3_pick_up/live_input.py](ur3_pick_up/ur3_pick_up/live_input.py) | Decode/transform clouds, apply workspace/voxel/count filters, rank candidates and check fixed-camera TF consistency. |
| [ur3_pick_up/ur3_pick_up/pick_up.py](ur3_pick_up/ur3_pick_up/pick_up.py) | pick_up entry point, ROS parameters, MoveIt planning/execution, TCP/path markers and Robotiq socket control. |

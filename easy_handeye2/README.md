# Easy HandEye 2 calibration services and manual interface

[繁體中文](README.zh-TW.md)

`easy_handeye2` provides generic hand-eye sampling, OpenCV calibration computation, result storage and rqt calibration/evaluation interfaces. It does not generate UR3 motion or launch ArUco. Use [auto_calibration](../auto_calibration/README.md) for the automatic workflow and camera-root TF publication.

This is the project's maintained Easy HandEye 2 core, with existing TF/service error handling and duplicate-server protection retained. It is not an untouched upstream checkout. UR-specific motion and publisher integration belong to `auto_calibration`; the core has no MoveItPy, UR configuration or ArUco dependency.

## Installation

Complete Sections 1–4 of the [host setup guide](../ROS2+driver/ur3_ros2_setup.md) once. The core source is included here. Download only `easy_handeye2_msgs` from upstream as described there; do not add a second Easy HandEye implementation to the workspace.

`package.xml` declares rclpy, TF, ROS interfaces, OpenCV, NumPy, transforms3d, YAML and Qt/rqt dependencies. The shared rosdep command installs them. The core does not require Docker, AnyGrasp or MoveIt. Automatic operation has its own dependencies in `auto_calibration`.

## Manual calibration

For the default fixed-camera configuration, keep the camera stationary and secure the ArUco marker to the robot tool. Start [bringup](../ur3_bringup/README.md), then run the tracker in a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run aruco_opencv aruco_tracker_autostart --ros-args \
  -p cam_base_topic:=/camera/camera/color/image_raw \
  -p marker_size:=0.04 -p marker_dict:=ARUCO_ORIGINAL
```

The marker side length must match the physical marker in meters. Start the manual calibration GUI and service in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch easy_handeye2 calibrate.launch.py \
  name:=ur3_realsense_eob calibration_type:=eye_on_base \
  robot_base_frame:=base_link robot_effector_frame:=tool0 \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=marker_582
```

Move the arm using your existing robot controls. In the GUI, start with an empty sample list, collect samples at varied orientations/positions, compute the calibration and save it. This manual interface does not move the arm or perform the automatic workflow's preflight/sample reset. Do not run it alongside an automatic calibration session; they share one service namespace.

| Launch argument | Meaning |
| --- | --- |
| `name` | Calibration file name. |
| `calibration_type` | `eye_on_base`: fixed camera, marker on tool; `eye_in_hand`: camera on tool, fixed marker. |
| `robot_base_frame` | Robot reference frame. |
| `robot_effector_frame` | Robot end-effector frame. |
| `tracking_base_frame` | Frame in which the tracker reports the marker pose. |
| `tracking_marker_frame` | Detected marker frame; match the actual marker ID. |

All six arguments are required. The service needs separate robot-base→tool and camera→marker TF chains; no placeholder transform connects the trees during calibration.

## Saved results and TF

Results use `~/.ros2/easy_handeye2/calibrations/<name>.calib`; sample lists use `~/.ros2/easy_handeye2/samples/`. The core stores the calibrated tracking-frame pose. For RealSense, let [auto_calibration's publisher](../auto_calibration/README.md) attach the camera root without replacing camera-internal TF:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch auto_calibration publish.launch.py \
  name:=ur3_realsense_eob tracking_root_frame:=camera_link
```

Run only one calibration publisher. For reload, single-publisher behavior and camera readiness, follow the automatic-calibration guide.

## Evaluation interface

After saving a calibration and preparing the needed TF/tracker, launch the evaluation GUI:

```bash
ros2 launch easy_handeye2 evaluate.launch.py name:=ur3_realsense_eob
```

The `rqt_calibrator.py` and `rqt_evaluator.py` executable names are registered console entry points. GUI resources and plugin descriptors remain part of this package.

## Service behavior

Services use `/easy_handeye2/calibration/`: `take_sample`, `get_sample_list`, `get_current_transforms`, `remove_sample`, `save_samples`, `load_samples`, `compute_calibration`, `save_calibration`, `list_algorithms` and `set_algorithm`. Their interfaces come from `easy_handeye2_msgs`.

Missing/disconnected TF returns no current transform or new sample. Loading a missing or invalid sample file returns `success=false` and keeps existing samples. Removing the last sample accepts an empty list. Computation errors return `valid=false`. One local server may own this service namespace per ROS domain. These checks are independent of the automatic motion workflow.

The package uses the bundled LGPLv3 license and preserves upstream attribution.

<a id="file-inventory"></a>
## File inventory

Every source file in this folder and its subfolders is listed below, relative to this folder. Build outputs, runtime logs and caches are excluded.

| File | Purpose |
| --- | --- |
| [LICENSE.md](LICENSE.md) | Bundled LGPLv3 terms and license text. |
| [README.md](README.md) | English guide: Generic services, manual/evaluation GUIs and saved-result usage. |
| [README.zh-TW.md](README.zh-TW.md) | Traditional Chinese guide: Generic services, manual/evaluation GUIs and saved-result usage. |
| [easy_handeye2/__init__.py](easy_handeye2/__init__.py) | Define sample/calibration directories and service names; provide local resource ownership locks. |
| [easy_handeye2/handeye_calibration.py](easy_handeye2/handeye_calibration.py) | Declare/read calibration parameters, resolve calibration paths and load/save YAML results. |
| [easy_handeye2/handeye_calibration_backend_opencv.py](easy_handeye2/handeye_calibration_backend_opencv.py) | Convert ROS samples to OpenCV inputs, run hand-eye algorithms and construct calibration results. |
| [easy_handeye2/handeye_client.py](easy_handeye2/handeye_client.py) | Service client for manual sampling, algorithm selection, computation and saving. |
| [easy_handeye2/handeye_rqt_calibrator.py](easy_handeye2/handeye_rqt_calibrator.py) | Register the manual-calibration rqt plugin, rqt_calibrator.py entry point and widget lifecycle. |
| [easy_handeye2/handeye_rqt_calibrator_widget.py](easy_handeye2/handeye_rqt_calibrator_widget.py) | Load manual-calibration UI, update pose/sample displays and handle sampling, computation and save controls. |
| [easy_handeye2/handeye_rqt_evaluator.py](easy_handeye2/handeye_rqt_evaluator.py) | Register the evaluation rqt plugin, rqt_evaluator.py entry point and widget lifecycle. |
| [easy_handeye2/handeye_rqt_evaluator_widget.py](easy_handeye2/handeye_rqt_evaluator_widget.py) | Load evaluation UI, read calibration/current TF and display poses and consistency information. |
| [easy_handeye2/handeye_sampler.py](easy_handeye2/handeye_sampler.py) | Acquire robot/marker TF, add/remove samples and load/save sample lists. |
| [easy_handeye2/handeye_server.py](easy_handeye2/handeye_server.py) | Serve sampling/sample management, algorithm selection, computation and saving; handle errors and duplicate servers. |
| [launch/calibrate.launch.py](launch/calibrate.launch.py) | Launch the manual server and rqt GUI using six calibration-name/type/frame arguments. |
| [launch/evaluate.launch.py](launch/evaluate.launch.py) | Launch the rqt evaluation interface for a named calibration. |
| [package.xml](package.xml) | ROS package identity, version, license and build/runtime dependency declarations. |
| [plugin_calibrator.xml](plugin_calibrator.xml) | Declare the manual-calibration plugin class and menu metadata for rqt. |
| [plugin_evaluator.xml](plugin_evaluator.xml) | Declare the evaluation plugin class and menu metadata for rqt. |
| [resource/easy_handeye2](resource/easy_handeye2) | Package marker in the ament index; required for ROS discovery even when empty. |
| [resource/rqt_handeye.ui](resource/rqt_handeye.ui) | Qt Designer manual-calibration layout with sample list and sampling/computation/save controls. |
| [resource/rqt_handeye_evaluator.ui](resource/rqt_handeye_evaluator.ui) | Qt Designer calibration-evaluation layout. |
| [resource/rqt_handeye_info.ui](resource/rqt_handeye_info.ui) | Shared calibration-parameter/frame information layout for calibration and evaluation widgets. |
| [setup.cfg](setup.cfg) | Install registered executables under lib/easy_handeye2 for ros2 run discovery. |
| [setup.py](setup.py) | Register the server/two rqt commands and install launches, GUI/plugin resources, license and bilingual READMEs. |

# UR3 automatic hand-eye calibration and TF publication

[繁體中文](README.zh-TW.md)

`auto_calibration` moves the UR3, calls easy_handeye2 services for sampling/computation, and manages the TF used by `ur3_pick_up`. The default configuration is a fixed camera (`eye_on_base`); secure the ArUco marker rigidly to the tool during sampling.

## Installation and build

Complete Sections 1–4 of the [host/source setup guide](../ROS2+driver/ur3_ros2_setup.md). The complete workspace builds `auto_calibration` and `easy_handeye2` together; no repeated installation is needed.

| Dependency | Preparation |
| --- | --- |
| ROS 2 Jazzy, UR driver, MoveItPy, RealSense and system SDK | Follow the host guide. |
| `easy_handeye2` and `easy_handeye2_msgs` 0.5.0 | The generic calibration service is included; download message sources through the host guide. |
| ArUco, TF and Python libraries | Declared in package.xml and installed through the host guide's rosdep command. |

`auto_calibration` owns UR3 motion, tracker startup, operator prompts and camera-root TF publication. [easy_handeye2](../easy_handeye2/README.md) owns sampling, calibration computation, persistence and the manual GUI. Automatic operation starts only the background calibration service, without the manual GUI. Neither package requires Docker, an AnyGrasp license or weights.

Check the required entry points:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 pkg executables auto_calibration
ros2 interface show easy_handeye2_msgs/srv/TakeSample
ros2 pkg executables aruco_opencv
```

The tracker list must contain `aruco_tracker_autostart`. Keep one copy of each package name.

## Start calibration

Start the robot's external-control program, driver, MoveIt, camera and RViz:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup bringup.launch.py robot_ip:=192.168.86.7
```

Keep the camera fixed. Secure the marker rigidly to the tool and keep it visible through the planned motion. Marker size must be the actual marker side length in meters. Choose a starting robot pose with room for rotation and translation.

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run auto_calibration calibrate
```

Every startup first identifies and stops previous local automatic workflows, Easy HandEye servers/GUIs, calibration publishers and ArUco trackers using the same image topic. Cleanup is restricted to the current user and `ROS_DOMAIN_ID`, including orphaned children from an interrupted run. It requests graceful shutdown with SIGINT, then sends SIGTERM only on timeout. It does not stop bringup, camera drivers or robot controllers.

Cleanup verifies that matching processes, services, ROS nodes and resource locks are released before creating a fresh TF buffer and starting the tracker. If a process restarts, cannot be stopped, or remote/unidentified conflicts remain, startup fails without sampling. An empty lock file does not imply a running process; cleanup does not delete lock files to bypass ownership. Calibration files and backups are retained.

Startup cleanup also stops an independently running calibration publisher. Publication is still offered after saving; answer `y` to publish again. Existing RViz listeners may retain previously received static TF; stopping a publisher does not erase listener caches.

The program starts the tracker, waits for `camera_color_optical_frame → marker_582`, then asks `Marker detected. Start automatic calibration? [y/n]:`. Answering `y` authorizes the automatic motion and sampling sequence; the preflight does not ask again for each pose.

The workflow clears the server sample list and verifies zero, preflights all targets plus the return to the starting pose, then replans and executes each segment. It takes 15 samples: rotations of ±20° and ±10° about each tool axis, plus +0.04 m along each robot-base axis. After each move it waits 1.5 s before sampling. Each target can be planned up to 10 times. After returning to the starting pose, it computes and saves the calibration. These motion settings are code constants, not CLI options.

After saving, answer `Calibration saved. Start TF publishing? [y/n]:`. `y` reloads an existing publisher for this calibration, or starts one and keeps the terminal running. `n` saves without starting publication. Ctrl+C stops the processes owned by this workflow; an independently running publisher remains independent.

## Calibration command-line arguments

Use `--option value` directly, not ROS `-p` parameters:

```bash
ros2 run auto_calibration calibrate \
  --name ur3_realsense_eob \
  --calibration-type eye_on_base \
  --robot-base-frame base_link \
  --robot-effector-frame tool0 \
  --tracking-base-frame camera_color_optical_frame \
  --tracking-root-frame camera_link \
  --tracking-marker-frame marker_582 \
  --image-topic /camera/camera/color/image_raw \
  --marker-size 0.04 \
  --marker-dict ARUCO_ORIGINAL \
  --startup-timeout 180
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `--name` | `ur3_realsense_eob` | Calibration file name and publication service name. |
| `--calibration-type` | `eye_on_base` | `eye_on_base`: fixed camera, marker on tool. `eye_in_hand`: camera on tool, fixed marker; mounting must match the choice. |
| `--robot-base-frame` | `base_link` | Robot reference frame. |
| `--robot-effector-frame` | `tool0` | Robot end-effector frame. |
| `--tracking-base-frame` | `camera_color_optical_frame` | Frame in which the tracker reports the marker pose. |
| `--tracking-root-frame` | `camera_link` | Camera root attached to the robot tree during publication. |
| `--tracking-marker-frame` | `marker_582` | Detected marker TF frame; must match marker ID/dictionary. |
| `--image-topic` | `/camera/camera/color/image_raw` | Tracker camera image topic. |
| `--marker-size` | `0.04` | Positive, finite marker side length in meters. |
| `--marker-dict` | `ARUCO_ORIGINAL` | ArUco dictionary name. |
| `--startup-timeout` | `180.0` | Positive, finite seconds to wait for calibration services and required TF after tracker detection; initial marker detection waits until found or interrupted. |

Use `--help` to list options. The internal worker option is not needed for normal operation.

## Publish a saved calibration

The running camera must first provide its internal `camera_link → camera_color_optical_frame` TF. For `No camera-internal TF`, inspect streaming and camera startup logs first; this error alone does not mean the calibration file is missing. See the [bringup guide](../ur3_bringup/README.md) for mixed compiled/runtime camera SDK versions.

If a publisher for this calibration is already running, do not start another. Otherwise:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch auto_calibration publish.launch.py \
  name:=ur3_realsense_eob tracking_root_frame:=camera_link
```

| Launch argument | Default | Meaning |
| --- | --- | --- |
| `name` | required | Name of the calibration to load. |
| `tracking_root_frame` | `camera_link` | Camera root frame attached by the publisher. |

Keep the publisher and camera driver running for pick-up execution. Calibration files are stored at `~/.ros2/easy_handeye2/calibrations/<name>.calib`. Saving over an existing calibration creates a timestamped backup. The file contains the calibrated tracking-frame pose; the publisher uses the camera's internal TF to attach its root. For this eye-on-base setup the published edge is `base_link → camera_link`; RealSense supplies `camera_link → ... → camera_color_optical_frame`. The publisher owns only its calibration edge and prevents another local publisher from claiming the same camera root in the same ROS domain.

Reload a saved file into the running publisher:

```bash
ros2 service call /easy_handeye2/ur3_realsense_eob/reload_calibration \
  std_srvs/srv/Trigger '{}'
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

A successful reload updates the existing static edge, including for new listeners. Changing frame identities or calibration type requires restarting the publisher. Existing listeners may retain static TF after a publisher stops; use a fresh `tf2_echo` to check publication.

## Operation checks

The calibration workflow prints its log paths under `~/.ros/log/handeye-workflow-*/`: `aruco_tracker.log` and `calibration_server.log`. If a marker is not detected, check image, dictionary, ID, side length and visibility. If preflight fails, reposition the starting pose before retrying. A service-provider conflict requires stopping the duplicate calibration session. Before pick-up, verify the point cloud and robot model agree in the robot frame; successful publication alone does not establish calibration accuracy. A native shutdown crash is an abnormal process exit; verify the saved-file message and published transform rather than inferring success from the exit alone.

The calibration format, storage directory and `/easy_handeye2/<name>/reload_calibration` service interface stay consistent; pick_up only needs the resulting TF. This package uses the bundled LGPLv3 license.

<a id="file-inventory"></a>
## File inventory

Every source file in this folder and its subfolders is listed below, relative to this folder. Build outputs, runtime logs and caches are excluded.

| File | Purpose |
| --- | --- |
| [LICENSE.md](LICENSE.md) | Bundled LGPLv3 terms and license text. |
| [README.md](README.md) | English guide: Automatic calibration, startup cleanup, parameters, TF publication and operation checks. |
| [README.zh-TW.md](README.zh-TW.md) | Traditional Chinese guide: Automatic calibration, startup cleanup, parameters, TF publication and operation checks. |
| [auto_calibration/__init__.py](auto_calibration/__init__.py) | Python package marker and description for automatic calibration. |
| [auto_calibration/calibrate.py](auto_calibration/calibrate.py) | Automatic calibration entry point coordinating cleanup, tracker, services, prompts, motion worker and TF publication. |
| [auto_calibration/motion.py](auto_calibration/motion.py) | Service clients, 15-pose generation, planning diagnostics/retries, motion, sample reset and compute/save workflow. |
| [auto_calibration/publisher.py](auto_calibration/publisher.py) | Load calibration, compose/publish camera-root TF, enforce one publisher and provide the reload service. |
| [auto_calibration/startup.py](auto_calibration/startup.py) | Identify/stop previous calibration and same-camera tracker processes; check ROS conflicts and locks before startup. |
| [launch/publish.launch.py](launch/publish.launch.py) | Launch the saved-calibration TF publisher with calibration-name and camera-root arguments. |
| [package.xml](package.xml) | ROS package identity, version, license and build/runtime dependency declarations. |
| [resource/auto_calibration](resource/auto_calibration) | Package marker in the ament index; required for ROS discovery even when empty. |
| [setup.cfg](setup.cfg) | Install registered executables under lib/auto_calibration for ros2 run discovery. |
| [setup.py](setup.py) | Register calibrate/handeye_publisher commands and install launch, license and bilingual READMEs. |

# UR3 IK and Trajectory Planning Experiments

This directory contains experimental ROS 2 Jazzy / MoveIt 2 scripts and a progress log (6 November 2026). 

**Execution caution:** Scripts with `execute` in their filenames can command the physical UR3 when explicitly run with `--execute`. They are research prototypes, not safety-certified controllers. Keep the workspace clear and verify every planned trajectory and controller/pendant status before any motion. All other preview and visualization scripts should be used in planning-only mode.

**Status:** IK, multiple-route planning, TCP trajectory lines, virtual animation, and short supervised physical movements were demonstrated. Cartesian planning in RViz was demonstrated. **Physical Cartesian execution and verified obstacle-avoidance detours are not yet confirmed.** Do not describe these as completed.

## Files

- `scripts/ur3_ik_50mm_preview.py`: first live-state IK + MoveIt preview.
- `scripts/ur3_ik_tcp_path_preview.py`: preview TCP path visualization.
- `scripts/ur3_multiseed_long_paths.py`, `scripts/ur3_thin_long_distance_preview.py`: multi-seed path searches and line rendering.
- `scripts/ur3_fixed_target_pose_preview.py`, `scripts/ur3_fixed_target_alternative_routes.py`, `scripts/ur3_fixed_target_animated_routes.py`: same target B, alternate routes and RViz animation.
- `scripts/ur3_equal_distance_directions_preview.py`, `scripts/ur3_extension_search_plan_only.py`: direction and extension feasibility experiments.
- `scripts/ur3_rviz_tcp_line_visualizer.py`: listens to `/display_planned_path` and draws TCP marker line using `/compute_fk`.
- `scripts/ur3_obstacle_scene.py`: add/list/remove a box collision object; scene editing only, no motion.
- `scripts/ur3_guarded_ik_execute_v2.py`, `scripts/ur3_ik_plan_execute_qos.py`: execution-capable prototypes; require independent safety review.
- `scripts/ur3_multi_trajectory_preview.py`, `scripts/ur3_multi_trajectory_reachable_preview.py`: earlier multi-trajectory experiments.

Scripts are preserved from existing experiment files and were **syntax checked only** in the packaging environment. ROS runtime compatibility and physical safety have not been revalidated here. The UR3 configuration, frames, and required ROS interfaces are described below.

---

# UR3 Robot Arm: IK, Trajectory Planning, Execution & Obstacle Avoidance

**Progress as of 6 November 2026**  
**Environment:** Ubuntu 24.04, ROS 2 Jazzy, MoveIt 2, RViz 2, physical UR3  
**Current phase:** Static obstacle-avoidance setup (**not yet experimentally validated**)

## 1. Goal and status

Build a repeatable physical UR3 workflow: read live TCP/joints, define target pose B, solve inverse kinematics (IK), plan paths from A to B, visualize TCP trajectories, and execute selected safe movements with operator approval. Next, represent a box obstacle in MoveIt's planning scene and compare collision-aware trajectories with and without the obstacle.

| Milestone | Status |
|---|---|
| UR3 driver and ROS 2 connectivity | Demonstrated |
| Live `/joint_states` and TF `base_link -> tool0` | Demonstrated |
| IK via `/compute_ik` | Demonstrated |
| MoveIt planning via `/move_action` | Demonstrated |
| Several routes to a common target B | Demonstrated |
| RViz colored TCP trajectory lines | Demonstrated |
| RViz virtual trajectory animation | Demonstrated |
| Supervised real UR3 motion | Demonstrated (short moves) |
| Joint-space vs Cartesian planning | Demonstrated in RViz |
| Physical Cartesian execution | Not independently verified |
| Add box and verify collision-aware detour | Next experiment |
| Physical obstacle avoidance | Not attempted/verified |
| ROS 2 bag measurement and accuracy analysis | Future work |

No human-handover experiment has been performed.

## 2. Configuration

- **Host:** Lenovo V14-IIL; Ubuntu 24.04 LTS; ROS 2 Jazzy.
- **Robot:** physical Universal Robots UR3; planning group `ur_manipulator`.
- **Frames:** `base_link` and TCP `tool0`.
- **Ethernet (recorded lab setup):** host `192.168.86.70/24` on `enx00e04c360169`; robot `192.168.86.7`.
- **ROS interfaces:** `/joint_states`, `/tf`, `/compute_ik`, `/move_action`, `/display_planned_path`, `/apply_planning_scene`.
- **Custom marker topic:** `/ur3_ik_tcp_path` (exists only while the visualization node publishes).

## 3. Setup and diagnostics: commands

```bash
# Configure host network if required by this lab setup
sudo ip addr add 192.168.86.70/24 dev enx00e04c360169
ip -br addr
ping -c 4 192.168.86.7

# ROS environment
source /opt/ros/jazzy/setup.bash

# Driver/controller status
ros2 control list_controllers
ros2 topic echo /io_and_status_controller/robot_program_running \
  --once --qos-reliability reliable --qos-durability transient_local
ros2 topic echo /io_and_status_controller/robot_mode --once
ros2 topic echo /io_and_status_controller/safety_mode --once

# Current real robot state
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
ros2 run tf2_ros tf2_echo base_link tool0  # Ctrl+C to stop

# MoveIt interfaces
ros2 service list | grep compute_ik
ros2 action list | grep move_action
ros2 service list | grep planning_scene
```

Previously confirmed active controllers: `scaled_joint_trajectory_controller`, `joint_state_broadcaster`, `io_and_status_controller`. A working state included `robot_program_running: true`, `robot_mode: 7` and `safety_mode: 1`. **A retained `true` message is not proof of present readiness**: confirm the current robot and teach-pendant conditions before any execution.

## 4. IK and multi-trajectory progress

The IK solver calculates a joint configuration for a TCP target; the motion planner computes a collision-checked path between configurations. **IK by itself does not guarantee a straight path.** Some experiments used a custom 30-degree maximum joint-change screen. This is an experimental rejection criterion, *not* a mechanical joint limit or safety certification.

### Distinct paths to the same target

Example from an 8 cm target test with 2 cm intermediate detours:

| Route | Reported TCP path | Trajectory points |
|---|---:|---:|
| Direct | 7.9 cm | 20 |
| Y+ at 35% | 9.2 cm | 44 |
| Y− at 35% | 9.1 cm | 72 |
| Z+ at 35% | 9.2 cm | 37 |
| Z− at 35% | 9.2 cm | 35 |

**Result:** `5 distinct routes to ONE fixed target B`; all routes were planned and previewed, not automatically executed.

Representative planning-only scripts from the experiments (run only if present locally):

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/Downloads/ur3_fixed_target_alternative_routes.py --dx-cm 18 --detour-cm 4
python3 ~/Downloads/ur3_fixed_target_animated_routes.py --dx-cm 8 --detour-cm 2
```

### Outward-extension search

```bash
python3 ~/Downloads/ur3_extension_search_plan_only.py
```

In a later run starting from `(0.1026, 0.1979, 0.6036)` m, the program tested 5, 10, 15 and 20 cm targets across three outward directions and returned **0/12 accepted plans**:

- 5 cm and 10 cm goals often returned IK configurations needing about 92–139 degrees of joint motion, above the 30-degree script screening threshold.
- Several 15 cm targets and all 20 cm targets returned `-31` (`NO_IK_SOLUTION`).
- Fixed tool orientation, limited tested directions, IK solution branches, collisions and possible singularity effects remain hypotheses, not conclusively isolated causes.

The physical arm can still execute other reachable poses via RViz. **Never disable collision checks or increase limits just to force a solution.**

### Live poses recorded at different times

| Sample | X (m) | Y (m) | Z (m) |
|---|---:|---:|---:|
| Earlier | -0.008 | 0.177 | 0.639 |
| Later | 0.071 | 0.197 | 0.610 |
| Later still | 0.102 | 0.198 | 0.604 |

These are **not guaranteed consecutive pre/post motion readings**. Fresh readings are required for every experiment.

A near-zero `wrist_2_joint` was observed in one joint-state sample; it suggests a possible wrist singularity, but is not a proven cause of all IK failures.

## 5. RViz visualization and animation

MoveIt publishes `moveit_msgs/msg/DisplayTrajectory` on `/display_planned_path` to animate planned robot motion. A persistent colored **TCP line** is a separate `visualization_msgs/Marker` visualization.

```bash
# Start automatic TCP line visualizer, if the local script is available
source /opt/ros/jazzy/setup.bash
python3 ~/Downloads/ur3_rviz_tcp_line_visualizer.py

# In another terminal, verify publication
ros2 topic info /display_planned_path -v
ros2 topic info /ur3_ik_tcp_path -v
```

RViz: `Displays → Add → By display type → Marker`, then set Topic to `/ur3_ik_tcp_path`. Click **Plan** in the MotionPlanning panel. When the custom visualizer is stopped, `Unknown topic '/ur3_ik_tcp_path'` is expected.

## 6. Physical motion and Cartesian comparison

A supervised physical UR3 movement was successfully executed with MoveIt/RViz and another short IK execution was reported successful with a guarded Python execution workflow. **Endpoint positioning error was not measured.** Controller activation and a successful prior movement do not validate any new trajectory.

RViz joint-space / Cartesian comparison:

1. `MotionPlanning → Planning`, select `ur_manipulator` and Start State `Current`.
2. Set a deliberately chosen, reachable target B.
3. Turn **Use Cartesian Path OFF**, select **Plan**, inspect the joint-space path.
4. Keep the same A/B and turn **Use Cartesian Path ON**, select **Plan**; verify the full Cartesian fraction, not merely a partial straight-line preview.
5. Compare the TCP lines, joint motions and overall clearance. Do **not** execute unreviewed plans.

A later screenshot showed an executable Cartesian plan in RViz. **The supplied logs do not independently establish physical Cartesian execution.**

## 7. Obstacle avoidance: CURRENT, NOT YET COMPLETED

### Aim

Add a virtual 3D box as a MoveIt **collision object**, then compare plans between identical A and B with and without it. MoveIt must consider **all robot links and the modeled tool**, not just the TCP trace.

### Planning-only procedure

1. Choose a reachable A-to-B target and record a baseline plan without the test box.
2. In RViz's `MotionPlanning → Scene Objects`, add a **Box** with deliberate dimensions and pose. Alternatively apply a collision object using `/apply_planning_scene`.
3. Publish the changed planning scene and visually confirm the box relative to `base_link` and the robot. Confirm it does not overlap the current robot state.
4. Select the **same** A and B; switch **Use Cartesian Path OFF** for a general joint-space detour planner; click **Plan** only.
5. Compare success/failure, TCP route, joint-space changes, and full-body modeled clearance.
6. Do not execute against a physical box without its accurate measured pose and size, validated tool/gripper model, relevant surrounding geometry and separately approved motion.

Useful checks:

```bash
source /opt/ros/jazzy/setup.bash
ros2 service list | grep planning_scene
ros2 topic info /display_planned_path -v
ros2 topic info /ur3_ik_tcp_path -v
```

**Current evidence:** obstacle-avoidance procedure and box-creation options were discussed, but **no confirmed added-box scene or successful avoiding trajectory has been reported yet**.

## 8. Issues encountered and findings

| Issue | Current understanding |
|---|---|
| IK error `-31` | No solution returned for that specific pose and constraints; not proof of global unreachability |
| Large IK joint changes | Rejected by custom 30-degree screen, which is not a physical joint limit |
| `robot_program_running: false` | External Control program had stopped; scaled controller then became inactive |
| Python program missed status | Status topic's publisher advertised `TRANSIENT_LOCAL` QoS; subscription needed correct QoS |
| `A message was lost!!!` | A one-off subscriber warning; live states and transforms were still received |
| Initial TF frame error | Transform discovery delay; subsequent `base_link -> tool0` data appeared |
| Animated robot but no TCP line | Custom Marker publisher not running or Marker display not configured |
| Cartesian `Failed` in RViz | Complete path may be unreachable; inspect completion fraction and scene |

## 9. Next experiments

- [ ] Verify a collision box appears in MoveIt's planning scene.
- [ ] Compare obstacle-free and obstacle-aware plan-only A-to-B trajectories.
- [ ] Confirm full-arm/tool geometry and environment collision clearance.
- [ ] Later add RealSense-based scene perception, after static tests.
- [ ] Later record `/joint_states`, `/tf`, `/tf_static`, and `/display_planned_path` with `ros2 bag`.

```bash
# Future recording command only; no execution is triggered
mkdir -p ~/ur3_experiments
ros2 bag record -o ~/ur3_experiments/ik_motion_test_01 \
  /joint_states /tf /tf_static /display_planned_path
# Stop: Ctrl+C
ros2 bag info ~/ur3_experiments/ik_motion_test_01
```

**Safety:** Planning results are based on a software model. Execute on physical hardware only after current-state checks, physically accurate collision geometry, path inspection and operator authorization. This report does not claim safe human-proximate motion or completed physical obstacle avoidance.

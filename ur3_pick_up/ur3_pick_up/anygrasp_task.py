"""Live cloud -> ranked AnyGrasp poses -> first plannable confirmed home task."""

from geometry_msgs.msg import Quaternion
from tf2_ros import TransformException

from .anygrasp_input import (
    quaternion_from_matrix,
)
from .pick_up import DISPLAY_FRAME, RobotiqGripper


def plan_ranked_poses(app, ranked, base_from_camera):
    """Try poses by score without moving; require outbound AND home plans."""
    for rank, (index, candidate) in enumerate(ranked, start=1):
        # A changed camera calibration invalidates every captured pose, so this
        # check must abort the search rather than skip to another candidate.
        app.verify_snapshot()
        print(f'Trying candidate pose {index} (rank {rank}/{len(ranked)}): '
              f'score={candidate.score:.4f}, width={candidate.width:.4f} m', flush=True)
        try:
            target = candidate.tool_pose(base_from_camera, app.tip_from_tool)
            q = quaternion_from_matrix(target)
        except ValueError as exc:
            print(f'Candidate pose {index} cannot be converted: {exc}. Trying the next pose.')
            continue
        orientation = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
        xyz = tuple(float(v) for v in target[:3, 3])
        tcp_camera = candidate.translation + candidate.depth * candidate.rotation[:, 0]
        print(f'TCP ({DISPLAY_FRAME}, m): {tuple(float(v) for v in tcp_camera)}')
        try:
            outbound, returning = app.plan_pick_and_return(xyz, target_orientation=orientation)
        except TransformException:
            raise
        except RuntimeError as exc:
            print(f'Planning failed for candidate pose {index}: {exc}. Trying the next pose.')
            continue
        if outbound is None or returning is None:
            failed_stage = 'outbound' if outbound is None else 'return-to-home'
            print(f'Candidate pose {index}: {failed_stage} planning failed. Trying the next pose.')
            continue
        app.verify_snapshot()
        print(f'Selected candidate pose {index}: score={candidate.score:.4f}; '
              'outbound and return-to-home planning succeeded.')
        return outbound, returning
    print('All candidate poses failed to plan.')
    return None, None


def execute_task(app, outbound, returning):
    """Open/close at the grasp pose, then release only after returning home."""
    if outbound is None or returning is None:
        return False
    print("Outbound and return-to-home planning succeeded. Check the paths in RViz.")
    print("Sequence: move to the selected grasp pose -> open -> close -> return home -> open to release (POS=0).")
    if input("Confirm arm/gripper execution? Enter y: ").strip().lower() != "y":
        print("Canceled; no arm or gripper commands sent")
        return False
    stage = "Pre-execution TF check"
    try:
        app.verify_snapshot()
        stage = "Gripper initialization"
        if app.gripper_backend == "tcp":
            app.gripper = RobotiqGripper(app.robot_ip, app.gripper_port)
            app.gripper.activate()
        stage = "Outbound execution"
        if not app.execute_plan(outbound, "Outbound"):
            print("Outbound execution failed. Gripper actions at the target and return motion were not started.")
            return False
        stage = "Gripper actions at the selected grasp pose"
        if app.gripper_backend == "tcp":
            stage = "Opening at the selected grasp pose (POS=0)"
            app.gripper.move_and_wait(0)
            stage = "Closing at the selected grasp pose (POS=255)"
            app.gripper.move_and_wait(255)
        else:
            print("gripper_backend=none: skipped arrival open/close")
        stage = "Return-to-home execution"
        if not app.execute_plan(returning, "Return to home"):
            print("Return-to-home execution failed. Home was not reached; the release command was not sent.")
            return False
        stage = "Opening to release at home (POS=0)"
        if app.gripper_backend == "tcp":
            status = app.gripper.move_and_wait(0)
            if status != 3:
                raise RuntimeError("Gripper opening stopped on contact before reaching the open position")
            print("Returned home. Gripper opened (POS=0).")
        else:
            print("Returned home; gripper_backend=none, so the release command was skipped.")
        return True
    except (OSError, RuntimeError, TimeoutError, ValueError, TransformException) as exc:
        print(f"{stage} failed: {exc}. Stopping subsequent steps.")
        return False
    finally:
        if app.gripper:
            app.gripper.close()
            app.gripper = None


def run_anygrasp(app):
    """One fresh scene; try valid poses by score until both paths are planned."""
    from pathlib import Path
    from ament_index_python.packages import get_package_share_directory
    from .camera_capture import capture_cloud
    from .docker_client import DockerClient
    from .live_input import prepare_cloud, ranked_candidates
    import time

    stage = "Docker/model preflight"
    try:
        client = DockerClient(app.settings, Path(get_package_share_directory('ur3_pick_up'))
                              / 'scripts/anygrasp_worker.py')
        client.preflight()
        stage = "Initial motion to home"
        if not app.settings['plan_only']:
            if not app.move_to_initial_home():
                return False
        else:
            print('plan_only=true: capture and plan from the current state; skip initial home and all hardware commands.')
        time.sleep(.5)
        stage = "Camera cloud capture / TF conversion"
        print(f'Capturing a fresh point cloud from {app.settings["pointcloud_topic"]}...', flush=True)
        raw_points, stamp, base_from_camera = capture_cloud(app)
        app.snapshot_base_from_camera = base_from_camera
        stage = "Workspace cloud filtering"
        points, mask = prepare_cloud(raw_points, base_from_camera, app.workspace,
            voxel_size=app.settings['voxel_size'], max_points=app.settings['max_points'],
            max_depth=app.settings['max_camera_depth'], min_roi_points=app.settings['min_roi_points'])
        print(f'Sending {len(points)} scene points; {int(mask.sum())} points are inside the workspace.', flush=True)
        stage = "AnyGrasp Docker inference"
        candidates = client.infer(points, mask, DISPLAY_FRAME, stamp, base_from_camera, app.max_opening)
        stage = "Candidate pose filtering"
        ranked = ranked_candidates(candidates, base_from_camera, app.workspace, app.max_opening)
        stage = "Candidate outbound/return-to-home planning"
        outbound, returning = plan_ranked_poses(app, ranked, base_from_camera)
        if outbound is None or returning is None:
            return False
        if app.settings['plan_only']:
            print('plan_only=true: planning completed; no arm or gripper commands were sent.')
            return True
        return execute_task(app, outbound, returning)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, TransformException) as exc:
        print(f'{stage} failed: {exc}. Stopping subsequent steps.')
        return False

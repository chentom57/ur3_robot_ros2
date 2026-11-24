"""Live cloud -> ranked AnyGrasp poses -> confirmed pick and return task."""

from geometry_msgs.msg import Quaternion
from tf2_ros import TransformException

from .anygrasp_input import (
    quaternion_from_matrix,
)
from .pick_up import DISPLAY_FRAME, RobotiqGripper


def plan_ranked_poses(app, ranked, base_from_camera):
    """Try poses by score without moving; require outbound and startup-return plans."""
    failures = []
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
            failures.append((index, 'pose conversion'))
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
            failures.append((index, 'planning exception'))
            continue
        if outbound is None or returning is None:
            failed_stage = 'outbound' if outbound is None else 'return-to-start'
            print(f'Candidate pose {index}: {failed_stage} planning failed. Trying the next pose.')
            failures.append((index, failed_stage))
            continue
        app.verify_snapshot()
        print(f'Selected candidate pose {index}: score={candidate.score:.4f}; '
              'outbound and return-to-start planning succeeded.')
        return outbound, returning
    failure_list = ', '.join(f'{index} ({stage})' for index, stage in failures)
    print(f'Planning summary: 0/{len(ranked)} candidate poses planned successfully; '
          f'{len(failures)} failed: {failure_list}. No grasp trajectory was executed.', flush=True)
    return None, None


def execute_task(app, outbound, returning, operator_gui):
    """Pick, return to startup position, then wait for GUI approval to release."""
    if outbound is None or returning is None:
        return False
    print("Outbound and return-to-start planning succeeded. Confirm execution in the camera GUI.")
    print("Sequence: open at the startup position -> move to the selected grasp pose -> close -> return to startup position -> wait for GUI release command.")
    if not operator_gui.confirm_execution():
        print("Canceled in the GUI; no arm or gripper commands sent")
        return False
    stage = "Pre-execution TF check"
    try:
        app.verify_snapshot()
        stage = "Gripper initialization"
        if app.gripper_backend == "tcp":
            app.gripper = RobotiqGripper(app.robot_ip, app.gripper_port)
            app.gripper.activate()
            stage = "Opening before outbound motion (POS=0)"
            status = app.gripper.move_and_wait(0)
            if status != 3:
                raise RuntimeError("Gripper did not reach the fully open position before outbound motion")
        stage = "Outbound execution"
        if not app.execute_plan(outbound, "Outbound"):
            print("Outbound execution failed. Gripper actions at the target and return motion were not started.")
            return False
        stage = "Gripper actions at the selected grasp pose"
        if app.gripper_backend == "tcp":
            stage = "Closing at the selected grasp pose (POS=255)"
            app.gripper.move_and_wait(255)
        else:
            print("gripper_backend=none: skipped pre-approach open and arrival close")
        stage = "Return-to-start execution"
        if not app.execute_plan(returning, "Return to startup position"):
            print("Return-to-start execution failed. The startup position was not reached; the release command was not sent.")
            return False
        if app.gripper_backend == "tcp":
            stage = "Waiting for GUI release approval"
            print("Arm returned to its startup position. The gripper remains closed until you choose OPEN GRIPPER / RELEASE in the camera GUI.", flush=True)
            if not operator_gui.confirm_release():
                print("Release canceled in the GUI. The arm remains at its startup position and the gripper stays closed.")
                return False
            stage = "Opening to release at the startup position (POS=0)"
            status = app.gripper.move_and_wait(0)
            if status != 3:
                raise RuntimeError("Gripper opening stopped on contact before reaching the open position")
            print("Returned to the startup position. Gripper opened (POS=0).")
        else:
            print("Returned to the startup position; gripper_backend=none, so GUI release and gripper commands were skipped.")
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
    from .live_input import image_mask_to_point_mask, prepare_cloud, ranked_candidates
    from .target_selection import OperatorGUI
    import time

    stage = "Docker/model preflight"
    operator_gui = None
    try:
        client = DockerClient(app.settings, Path(get_package_share_directory('ur3_pick_up'))
                              / 'scripts/anygrasp_worker.py')
        client.preflight()
        print('The arm will remain where it is. Planning starts from the joint position recorded when this program launched.')
        time.sleep(.5)
        stage = "Manual target selection"
        print('Select ONE target object from the live camera image with two opposite corners. Right-click/U undo; R reset; Enter accepts; Esc cancels.',
              flush=True)
        operator_gui = OperatorGUI(app)
        selected_pixels = operator_gui.select_target_rectangle()
        if selected_pixels is None:
            print('Target selection canceled; no inference or robot motion was started.')
            return False
        stage = "Camera cloud capture / TF conversion"
        print(f'Capturing a fresh synchronized image and point cloud from '
              f'{app.settings["pointcloud_topic"]}...', flush=True)
        (raw_points, stamp, base_from_camera, color_image,
         projection_points, intrinsics) = capture_cloud(app)
        if selected_pixels.shape != color_image.shape[:2]:
            raise ValueError('Color image resolution changed during selection; restart the task and select again')
        app.snapshot_base_from_camera = base_from_camera
        point_mask = image_mask_to_point_mask(projection_points, selected_pixels, intrinsics)
        stage = "Point cloud filtering"
        points, mask = prepare_cloud(raw_points,
            target_mask=point_mask, voxel_size=app.settings['voxel_size'], max_points=app.settings['max_points'],
            max_depth=app.settings['max_camera_depth'], min_roi_points=app.settings['min_roi_points'])
        print(f'Sending {len(points)} scene points; {int(mask.sum())} points are inside the selected image region.', flush=True)
        stage = "AnyGrasp Docker inference"
        candidates = client.infer(points, mask, DISPLAY_FRAME, stamp, base_from_camera, app.max_opening)
        stage = "Candidate pose filtering"
        ranked = ranked_candidates(candidates, app.max_opening)
        stage = "Candidate outbound/return-to-start planning"
        outbound, returning = plan_ranked_poses(app, ranked, base_from_camera)
        if outbound is None or returning is None:
            return False
        if app.settings['plan_only']:
            print('plan_only=true: planning completed; no arm or gripper commands were sent.')
            return True
        return execute_task(app, outbound, returning, operator_gui)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, TransformException) as exc:
        print(f'{stage} failed: {exc}. Stopping subsequent steps.')
        return False
    finally:
        if operator_gui is not None:
            operator_gui.close()

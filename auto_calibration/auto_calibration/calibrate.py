"""Coordinate tracker, headless Easy HandEye services, motion and TF publication."""
import argparse
import codecs
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from std_srvs.srv import Trigger
from .motion import CalibrationReadiness, run_worker
from .startup import cleanup_previous_session

def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Wait for an ArUco marker, then launch Easy HandEye 2 and calibrate."
    )
    parser.add_argument("--calibration-type", choices=("eye_on_base", "eye_in_hand"),
                        default="eye_on_base")
    parser.add_argument("--name", default="ur3_realsense_eob")
    parser.add_argument("--robot-base-frame", default="base_link")
    parser.add_argument("--robot-effector-frame", default="tool0")
    parser.add_argument("--tracking-base-frame", default="camera_color_optical_frame")
    parser.add_argument("--tracking-root-frame", default="camera_link",
                        help="Camera root to attach when publishing the calibration TF")
    parser.add_argument("--tracking-marker-frame", default="marker_582")
    parser.add_argument("--image-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--marker-size", type=float, default=0.04)
    parser.add_argument("--marker-dict", default="ARUCO_ORIGINAL")
    parser.add_argument("--startup-timeout", type=float, default=180.0,
                        help="Maximum wait for services and TFs in seconds (default: 180)")
    # Run MoveItPy in an isolated process so its C++ objects are released
    # separately from the long-lived tracker and calibration launch.
    parser.add_argument("--calibrate-only", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if not math.isfinite(args.marker_size) or args.marker_size <= 0:
        parser.error("--marker-size must be positive and finite (meters)")
    if not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0:
        parser.error("--startup-timeout must be positive and finite (seconds)")
    for field in ("robot_base_frame", "robot_effector_frame", "tracking_base_frame",
                  "tracking_root_frame", "tracking_marker_frame"):
        value = getattr(args, field)
        if not value or value.startswith("/") or any(c.isspace() for c in value):
            parser.error(f"--{field.replace('_', '-')} must be a TF frame without leading slash or whitespace")
    if not args.name or args.name in {".", ".."} or "/" in args.name or "\\" in args.name:
        parser.error("--name must be a nonempty calibration name, not a path")
    return args


def terminate_processes(processes):
    # A launch leader may have exited while its children still hold services.
    # Each tracked process was started in its own session/process group.
    for _, process in reversed(processes):
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    for label, process in reversed(processes):
        try:
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            print(f"{label} did not stop cleanly; forcing shutdown.", file=sys.stderr)
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


def ask_yes_no(prompt):
    while True:
        try:
            answer = input(prompt).strip().lower()
        except EOFError:
            return False
        if answer in ("y", "yes"):
            return True
        if answer in ("n", "no"):
            return False
        print("Enter y or n.", flush=True)


def run_and_show(command):
    """Forward available bytes, including prompts with no newline."""
    process = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1, start_new_session=True,
    )
    output = []
    try:
        decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
        while True:
            chunk = os.read(process.stdout.fileno(), 4096)
            if not chunk:
                tail = decoder.decode(b"", final=True)
                if tail:
                    print(tail, end="", flush=True)
                    output.append(tail)
                break
            text = decoder.decode(chunk)
            print(text, end="", flush=True)
            output.append(text)
        return_code = process.wait()
    except KeyboardInterrupt:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        process.wait()
        raise
    return return_code, "".join(output)


def start_logged_process(command, logfile):
    """Keep background ROS logs out of interactive prompts; retain full diagnostics."""
    logfile = Path(logfile)
    logfile.parent.mkdir(parents=True, exist_ok=True)
    with logfile.open("w") as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
    print(f"Background log: {logfile}", flush=True)
    return process


def reload_existing_publisher(readiness, name):
    client = readiness.create_client(Trigger, f"/easy_handeye2/{name}/reload_calibration")
    try:
        if not client.wait_for_service(timeout_sec=2.0):
            return False
        response = readiness.call_service(client, Trigger.Request(), 'reload calibration TF', timeout_sec=10.0)
        if not response.success:
            raise RuntimeError(response.message)
        print(f"Existing publisher updated: {response.message}. RViz will receive the new TF.", flush=True)
        return True
    finally:
        readiness.destroy_client(client)


def main():
    args = parse_args()
    if args.calibrate_only:
        return run_worker(args)

    processes = []
    readiness = None
    rclpy.init()
    try:
        cleanup_previous_session(args)
        readiness = CalibrationReadiness()
        log_directory = Path.home() / '.ros' / 'log' / f'handeye-workflow-{time.time_ns()}'
        integrated_cmd = ["ros2", "run", "auto_calibration", "calibrate"]
        tracker_cmd = [
            "ros2", "run", "aruco_opencv", "aruco_tracker_autostart", "--ros-args",
            "-p", f"cam_base_topic:={args.image_topic}",
            "-p", f"marker_size:={args.marker_size}",
            "-p", f"marker_dict:={args.marker_dict}",
        ]
        print("Starting ArUco tracker...", flush=True)
        processes.append(("ArUco tracker",
                          start_logged_process(tracker_cmd, log_directory / "aruco_tracker.log")))

        print("Waiting for ArUco marker detection...", flush=True)
        readiness.wait_for_marker(
            args.tracking_base_frame, args.tracking_marker_frame, processes
        )

        if not ask_yes_no("Marker detected. Start automatic calibration? [y/n]: "):
            print("Calibration declined. Shutting down the ArUco tracker.", flush=True)
            return_code = 0
            return return_code

        if 'handeye_server' in readiness.get_node_names():
            raise RuntimeError('A previous handeye_server is still running. Stop the old calibration session; '
                               'no new calibration server started.')
        # Automatic acquisition needs the server, without the manual rqt GUI.
        calibrate_cmd = [
            "ros2", "run", "easy_handeye2", "handeye_server", "--ros-args",
            "-p", f"calibration_type:={args.calibration_type}",
            "-p", f"name:={args.name}",
            "-p", f"robot_base_frame:={args.robot_base_frame}",
            "-p", f"robot_effector_frame:={args.robot_effector_frame}",
            "-p", f"tracking_base_frame:={args.tracking_base_frame}",
            "-p", f"tracking_marker_frame:={args.tracking_marker_frame}",
        ]
        print("Starting the Easy HandEye 2 calibration service...", flush=True)
        processes.append(("Easy HandEye 2 server",
                          start_logged_process(calibrate_cmd, log_directory / "calibration_server.log")))

        readiness.wait_until_ready(
            args.robot_base_frame, args.robot_effector_frame,
            args.tracking_base_frame, args.tracking_marker_frame,
            args.startup_timeout, processes,
        )

        print("Starting automatic calibration from the current robot pose...", flush=True)
        auto_code, auto_output = run_and_show(
            integrated_cmd + ["--calibrate-only",
                              "--robot-base-frame", args.robot_base_frame,
                              "--robot-effector-frame", args.robot_effector_frame,
                              "--tracking-base-frame", args.tracking_base_frame,
                              "--tracking-marker-frame", args.tracking_marker_frame]
        )
        calibration_saved = "Calibration saved:" in auto_output
        if not calibration_saved or auto_code not in (0, -11, 245):
            raise RuntimeError(f"Automatic calibration did not complete (exit code {auto_code})")

        terminate_processes(processes)
        processes.clear()
        if ask_yes_no("Calibration saved. Start TF publishing? [y/n]: "):
            if reload_existing_publisher(readiness, args.name):
                return 0
            publish_cmd = [
                "ros2", "run", "auto_calibration", "handeye_publisher", "--ros-args",
                "-p", f"name:={args.name}",
                "-p", f"tracking_root_frame:={args.tracking_root_frame}",
            ]
            print("Publishing the calibration TF. Press Ctrl+C to stop and exit.", flush=True)
            processes.append(("Calibration TF publisher",
                              subprocess.Popen(publish_cmd, start_new_session=True)))
            publish_code = processes[-1][1].wait()
            if publish_code != 0:
                raise RuntimeError(f"Calibration TF publisher exited with code {publish_code}")
        else:
            print("TF publishing skipped. Workflow complete.", flush=True)
    except KeyboardInterrupt:
        print("Ctrl+C received. Shutting down the calibration workflow...", flush=True)
        return_code = 130
    except Exception as exc:
        print(f"Integrated calibration workflow failed: {exc}", file=sys.stderr, flush=True)
        return_code = 1
    else:
        return_code = 0
    finally:
        terminate_processes(processes)
        if readiness is not None:
            readiness.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())

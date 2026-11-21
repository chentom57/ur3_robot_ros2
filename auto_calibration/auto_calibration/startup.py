"""Clean up identifiable local hand-eye sessions before starting a new workflow."""
from dataclasses import dataclass
import os
from pathlib import Path
import re
import signal
import time

import rclpy
from rclpy.node import Node
from easy_handeye2 import claim_local_resource


@dataclass(frozen=True)
class PreviousProcess:
    pid: int
    started: str
    label: str
    coordinator: bool = False


def process_identity(pid):
    # The parenthesized comm field may contain spaces and parentheses.
    fields = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
    return fields[19], int(fields[1]), fields[0]  # start ticks, parent PID, state


def own_ancestors():
    result = set()
    pid = os.getpid()
    while pid > 0 and pid not in result:
        result.add(pid)
        try:
            _, pid, _ = process_identity(pid)
        except (OSError, ValueError, IndexError):
            break
    return result


def classify_command(argv, image_topic):
    """Match executable tokens, never substrings in a shell/python command string."""
    if not argv:
        return None
    command = argv
    if re.fullmatch(r'python(?:[0-9]+(?:\.[0-9]+)*)?', Path(argv[0]).name):
        command = argv[1:]
        while command and command[0] in ('-u', '-B', '-s', '-S', '-E', '-I'):
            command = command[1:]
        if command[:2] == ['-m', 'auto_calibration.calibrate']:
            return 'automatic calibration', '--calibrate-only' not in command
        if not command or command[0].startswith('-'):
            return None
    executable = Path(command[0])
    name = executable.name
    if name == 'ros2':
        if len(command) >= 4 and command[1] == 'run':
            package, entry = command[2:4]
            if package == 'auto_calibration' and entry == 'calibrate':
                return 'automatic calibration launcher', False
            if package == 'easy_handeye2' and entry == 'integrated_auto_handeye_calibration':
                return 'legacy automatic calibration launcher', False
            if package in ('easy_handeye2', 'auto_calibration') and entry == 'handeye_publisher':
                return 'calibration TF launcher', False
            if package == 'easy_handeye2' and entry in ('handeye_server', 'rqt_calibrator.py', 'rqt_evaluator.py'):
                return 'Easy HandEye launcher', False
            if package == 'aruco_opencv' and entry == 'aruco_tracker_autostart':
                return ('camera marker tracker launcher', False) if tracker_matches(command, image_topic) else None
        if (len(command) >= 4 and command[1] == 'launch'
                and command[2] in ('easy_handeye2', 'auto_calibration')
                and command[3] in ('calibrate.launch.py', 'publish.launch.py', 'evaluate.launch.py')):
            return 'hand-eye launch session', True
        return None
    if executable.parent.name in ('easy_handeye2', 'auto_calibration'):
        if name in ('calibrate', 'integrated_auto_handeye_calibration', 'integrated_handeye_calibration.py'):
            return 'automatic calibration', '--calibrate-only' not in command
        if name in ('handeye_server', 'handeye_publisher', 'rqt_calibrator.py', 'rqt_evaluator.py'):
            return name, False
    if name == 'aruco_tracker_autostart' and tracker_matches(command, image_topic):
        return 'camera marker tracker', False
    return None


def tracker_matches(command, image_topic):
    return f'cam_base_topic:={image_topic}' in command


def find_previous_processes(image_topic):
    excluded = own_ancestors()
    domain = str(int(os.environ.get('ROS_DOMAIN_ID', '0')))
    result = []
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit() or int(directory.name) in excluded:
            continue
        try:
            if directory.stat().st_uid != os.getuid():
                continue
            argv = [item.decode(errors='replace') for item in (directory / 'cmdline').read_bytes().split(b'\0') if item]
            match = classify_command(argv, image_topic)
            if match is None:
                continue
            # Do not signal another ROS domain, even if it uses the same executable.
            environment = dict(item.split(b'=', 1) for item in (directory / 'environ').read_bytes().split(b'\0') if b'=' in item)
            other_domain = str(int(environment.get(b'ROS_DOMAIN_ID', b'0')))
            if other_domain != domain:
                continue
            started, _, state = process_identity(int(directory.name))
            if state != 'Z':
                result.append(PreviousProcess(int(directory.name), started, *match))
        except (FileNotFoundError, ProcessLookupError):
            continue
        except (PermissionError, ValueError, IndexError) as exc:
            raise RuntimeError(f'Cannot verify previous hand-eye process PID {directory.name}: {exc}') from exc
    return sorted(result, key=lambda item: item.pid)


def is_alive(process):
    try:
        started, _, state = process_identity(process.pid)
        return started == process.started and state != 'Z'
    except (FileNotFoundError, ProcessLookupError):
        return False


def signal_processes(processes, sig):
    for process in processes:
        if not is_alive(process):
            continue
        descriptor = None
        try:
            # Pin this process identity so PID reuse cannot redirect the signal.
            descriptor = os.pidfd_open(process.pid)
            if is_alive(process):
                signal.pidfd_send_signal(descriptor, sig)
        except ProcessLookupError:
            pass
        except PermissionError as exc:
            raise RuntimeError(f'Cannot stop {process.label} PID {process.pid}: {exc}') from exc
        finally:
            if descriptor is not None:
                os.close(descriptor)


def wait_for_exit(processes, seconds):
    deadline = time.monotonic() + seconds
    remaining = [process for process in processes if is_alive(process)]
    while remaining and time.monotonic() < deadline:
        time.sleep(0.1)
        remaining = [process for process in remaining if is_alive(process)]
    return remaining


def stop_processes(processes):
    if not processes:
        return
    for process in processes:
        print(f'Stopping previous {process.label} (PID {process.pid})...', flush=True)
    signal_processes(processes, signal.SIGINT)
    remaining = wait_for_exit(processes, 8.0)
    if remaining:
        print('Previous calibration processes did not exit on SIGINT; sending SIGTERM.', flush=True)
        signal_processes(remaining, signal.SIGTERM)
        remaining = wait_for_exit(remaining, 3.0)
    if remaining:
        detail = ', '.join(f'{p.label} PID {p.pid}' for p in remaining)
        raise RuntimeError(f'Previous calibration processes are still running: {detail}. No new session started.')


def graph_conflicts(node):
    names = {
        'handeye_server', 'handeye_publisher', 'handeye_rqt_calibrator',
        'handeye_rqt_evaluator', 'ur3_integrated_auto_handeye',
        'ur3_integrated_handeye_moveit', 'integrated_handeye_readiness',
        'auto_calibration_readiness', 'auto_calibration_motion', 'auto_calibration_moveit',
    }
    conflicts = [f'node {namespace.rstrip("/")}/{name}'
                 for name, namespace in node.get_node_names_and_namespaces() if name in names]
    conflicts.extend(f'service {name}' for name, _ in node.get_service_names_and_types()
                     if name.startswith('/easy_handeye2/calibration/')
                     or (name.startswith('/easy_handeye2/') and name.endswith('/reload_calibration')))
    return sorted(set(conflicts))


def wait_for_clean_graph(node):
    # Allow initial DDS discovery, then require a continuous quiet interval.
    started = time.monotonic()
    deadline = started + 8.0
    clear_since = None
    last_conflicts = []
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        last_conflicts = graph_conflicts(node)
        now = time.monotonic()
        if last_conflicts:
            clear_since = None
        elif clear_since is None:
            clear_since = now
        if clear_since is not None and now - started >= 2.0 and now - clear_since >= 1.0:
            return
    detail = '; '.join(last_conflicts) or 'ROS discovery interrupted'
    raise RuntimeError(f'Hand-eye startup conflicts remain: {detail}. '
                       'Unidentified or remote processes were not stopped; no new session started.')


def cleanup_previous_session(args):
    """Stop prior local sessions, verify ownership/ROS discovery, then release startup."""
    print('Checking for previous hand-eye sessions and calibration publishers...', flush=True)
    # Serialize startup cleanup only; a subsequent invocation may stop a running workflow.
    with claim_local_resource('startup:/auto_calibration'):
        node = Node('auto_calibration_startup')
        try:
            previous = find_previous_processes(args.image_topic)
            # Let an old workflow/launch shut down its children before cleaning orphans.
            stop_processes([process for process in previous if process.coordinator])
            stop_processes(find_previous_processes(args.image_topic))
            remaining = find_previous_processes(args.image_topic)
            if remaining:
                raise RuntimeError('Previous hand-eye processes restarted during cleanup; no new session started.')
            # An empty lock file is harmless; actual ownership is checked with flock.
            for resource in dict.fromkeys(('services:/easy_handeye2/calibration',
                                           args.tracking_root_frame, args.tracking_base_frame)):
                with claim_local_resource(resource):
                    pass
            wait_for_clean_graph(node)
        finally:
            node.destroy_node()
    print('Previous hand-eye sessions cleared. Starting a fresh calibration workflow.', flush=True)

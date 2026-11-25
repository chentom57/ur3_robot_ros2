import os
import pathlib

SAMPLES_DIRECTORY = pathlib.Path(os.path.expanduser('~/.ros2/easy_handeye2/samples'))
CALIBRATIONS_DIRECTORY = pathlib.Path(os.path.expanduser('~/.ros2/easy_handeye2/calibrations'))

CALIBRATION_NAMESPACE = '/easy_handeye2/calibration/'

LIST_ALGORITHMS_TOPIC = CALIBRATION_NAMESPACE + 'list_algorithms'
SET_ALGORITHM_TOPIC = CALIBRATION_NAMESPACE + 'set_algorithm'
GET_CURRENT_TRANSFORMS_TOPIC = CALIBRATION_NAMESPACE + 'get_current_transforms'
GET_SAMPLE_LIST_TOPIC = CALIBRATION_NAMESPACE + 'get_sample_list'
TAKE_SAMPLE_TOPIC = CALIBRATION_NAMESPACE + 'take_sample'
REMOVE_SAMPLE_TOPIC = CALIBRATION_NAMESPACE + 'remove_sample'
SAVE_SAMPLES_TOPIC = CALIBRATION_NAMESPACE + 'save_samples'
LOAD_SAMPLES_TOPIC = CALIBRATION_NAMESPACE + 'load_samples'
COMPUTE_CALIBRATION_TOPIC = CALIBRATION_NAMESPACE + 'compute_calibration'
SAVE_CALIBRATION_TOPIC = CALIBRATION_NAMESPACE + 'save_calibration'


def claim_local_resource(resource):
    """Hold one local owner per ROS domain/resource; keep the lock file inode stable."""
    import fcntl
    import hashlib

    key = f"{os.environ.get('ROS_DOMAIN_ID', '0')}:{resource}"
    digest = hashlib.sha256(key.encode()).hexdigest()[:24]
    directory = pathlib.Path.home() / '.ros2' / 'easy_handeye2' / 'publisher_locks'
    directory.mkdir(parents=True, exist_ok=True)
    handle = (directory / f'{digest}.lock').open('a+')
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError(f'Another process already owns {resource} in this ROS domain; '
                           'stop that session before starting a replacement.')
    return handle

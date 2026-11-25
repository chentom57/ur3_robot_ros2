"""Capture a fresh synchronized color image and point cloud."""
import threading
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from .anygrasp_input import matrix_from_transform
from .live_input import decode_xyz, transform_points


def capture_cloud(app):
    event = threading.Event()
    clouds, images, camera_infos = [], [], []
    last_reason = ['no messages received']
    started = app.node.get_clock().now().nanoseconds

    def fresh(msg):
        stamp = msg.header.stamp
        captured = stamp.sec * 1_000_000_000 + stamp.nanosec
        age = (app.node.get_clock().now().nanoseconds - captured) / 1e9
        if not msg.header.frame_id or captured < started or not 0 <= age <= app.settings['cloud_max_age']:
            last_reason[0] = f'cloud frame={msg.header.frame_id}, age={age:.3f}s; require a fresh sample'
            return False
        return True

    def find_pair():
        for cloud in reversed(clouds):
            ct = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
            for image in reversed(images):
                it = image.header.stamp.sec * 1_000_000_000 + image.header.stamp.nanosec
                if abs(ct - it) <= 150_000_000:
                    matching_info = [info for info in camera_infos
                                     if info.width == image.width and info.height == image.height
                                     and info.header.frame_id]
                    if matching_info:
                        return cloud, image, matching_info[-1]
        return None

    def cloud_callback(msg):
        if not event.is_set() and fresh(msg):
            clouds.append(msg)
            del clouds[:-10]
            if find_pair():
                event.set()

    def image_callback(msg):
        if not event.is_set() and fresh(msg):
            images.append(msg)
            del images[:-10]
            if find_pair():
                event.set()

    def info_callback(msg):
        if not event.is_set() and msg.width > 0 and msg.height > 0 and msg.header.frame_id:
            camera_infos.append(msg)
            del camera_infos[:-10]
            if find_pair():
                event.set()

    subscription = app.node.create_subscription(
        PointCloud2, app.settings['pointcloud_topic'], cloud_callback, qos_profile_sensor_data)
    image_subscription = app.node.create_subscription(
        Image, app.settings['color_image_topic'], image_callback, qos_profile_sensor_data)
    info_subscription = app.node.create_subscription(
        CameraInfo, app.settings['color_camera_info_topic'], info_callback, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + app.settings['cloud_timeout']
        while rclpy.ok() and time.monotonic() < deadline:
            if event.wait(.1):
                break
        pair = find_pair()
        if pair is None:
            raise TimeoutError(f'Could not capture color/cloud data and camera intrinsics on '
                f'{app.settings["color_image_topic"]}, {app.settings["pointcloud_topic"]}, '
                f'{app.settings["color_camera_info_topic"]}: {last_reason[0]}')
    finally:
        app.node.destroy_subscription(subscription)
        app.node.destroy_subscription(image_subscription)
        app.node.destroy_subscription(info_subscription)
    msg, image_msg, camera_info = pair
    image = decode_color_image(image_msg)
    if (camera_info.width, camera_info.height) != (image.shape[1], image.shape[0]):
        raise ValueError('CameraInfo dimensions do not match the raw color image')
    when = Time.from_msg(msg.header.stamp)
    base_from_camera = matrix_from_transform(app.tf_buffer.lookup_transform(
        app.base_frame, app.input_frame, when, timeout=Duration(seconds=3.)).transform)
    points = decode_xyz(msg)
    if msg.header.frame_id != app.input_frame:
        camera_from_source = matrix_from_transform(app.tf_buffer.lookup_transform(
            app.input_frame, msg.header.frame_id, when, timeout=Duration(seconds=3.)).transform)
        points = transform_points(points, camera_from_source)
    projection_points = points
    if camera_info.header.frame_id != app.input_frame:
        camera_from_projection = matrix_from_transform(app.tf_buffer.lookup_transform(
            camera_info.header.frame_id, app.input_frame, when,
            timeout=Duration(seconds=3.)).transform)
        projection_points = transform_points(points, camera_from_projection)
    intrinsics = dict(matrix=np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3),
                      distortion=np.asarray(camera_info.d, dtype=np.float64),
                      distortion_model=camera_info.distortion_model)
    return (points, (msg.header.stamp.sec, msg.header.stamp.nanosec), base_from_camera,
            image, projection_points, intrinsics)


def decode_color_image(msg):
    """Convert a ROS Image message into an RGB array without cv_bridge."""
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}
    if msg.encoding not in channels:
        raise ValueError(f'Unsupported color image encoding {msg.encoding!r}; expected RGB/BGR 8-bit')
    count = channels[msg.encoding]
    if msg.step < msg.width * count or len(msg.data) < msg.step * msg.height:
        raise ValueError('Malformed color Image data/step')
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = rows[:, :msg.width * count].reshape(msg.height, msg.width, count)
    if msg.encoding.startswith('bgr'):
        return np.ascontiguousarray(pixels[:, :, 2::-1])
    return np.ascontiguousarray(pixels[:, :, :3])

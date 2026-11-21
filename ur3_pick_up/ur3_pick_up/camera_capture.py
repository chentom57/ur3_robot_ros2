"""Capture a post-home sensor sample; never reuse an old cached cloud."""
import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from .anygrasp_input import matrix_from_transform
from .live_input import decode_xyz, transform_points


def capture_cloud(app):
    event = threading.Event()
    samples = []
    last_reason = ['no messages received']
    started = app.node.get_clock().now().nanoseconds

    def callback(msg):
        if event.is_set():
            return
        stamp = msg.header.stamp
        captured = stamp.sec * 1_000_000_000 + stamp.nanosec
        age = (app.node.get_clock().now().nanoseconds - captured) / 1e9
        if not msg.header.frame_id or captured < started or not 0 <= age <= app.settings['cloud_max_age']:
            last_reason[0] = f'cloud frame={msg.header.frame_id}, age={age:.3f}s; require a fresh post-home sample'
            return
        samples.append(msg)
        event.set()

    subscription = app.node.create_subscription(
        PointCloud2, app.settings['pointcloud_topic'], callback, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + app.settings['cloud_timeout']
        while rclpy.ok() and time.monotonic() < deadline:
            if event.wait(.1):
                break
        if not samples:
            raise TimeoutError(f'PointCloud2 capture failed on {app.settings["pointcloud_topic"]}: {last_reason[0]}')
    finally:
        app.node.destroy_subscription(subscription)
    msg = samples[0]
    when = Time.from_msg(msg.header.stamp)
    base_from_camera = matrix_from_transform(app.tf_buffer.lookup_transform(
        app.base_frame, app.input_frame, when, timeout=Duration(seconds=3.)).transform)
    points = decode_xyz(msg)
    if msg.header.frame_id != app.input_frame:
        camera_from_source = matrix_from_transform(app.tf_buffer.lookup_transform(
            app.input_frame, msg.header.frame_id, when, timeout=Duration(seconds=3.)).transform)
        points = transform_points(points, camera_from_source)
    return points, (msg.header.stamp.sec, msg.header.stamp.nanosec), base_from_camera

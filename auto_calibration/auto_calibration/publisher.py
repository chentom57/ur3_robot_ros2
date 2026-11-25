#!/usr/bin/env python3
"""Publish saved hand-eye calibration, optionally attaching a camera root frame."""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Transform, TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from transforms3d.quaternions import quat2mat, mat2quat
from easy_handeye2.handeye_calibration import load_calibration
from easy_handeye2 import claim_local_resource


class CalibrationTransformBroadcaster:
    """Publish exactly one replaceable static edge, including for late subscribers.

    Some Jazzy StaticTransformBroadcaster versions ignore repeat child IDs.
    Use the public ROS publisher API instead of mutating its private cache.
    """
    def __init__(self, node):
        self.publisher = node.create_publisher(
            TFMessage, '/tf_static', QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST))

    def sendTransform(self, transform):
        self.publisher.publish(TFMessage(transforms=[transform]))




def transform_matrix(transform):
    q = transform.rotation
    matrix = np.eye(4)
    matrix[:3, :3] = quat2mat([q.w, q.x, q.y, q.z])
    matrix[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
    return matrix


def root_calibration(calibration, root_to_tracking):
    """T_parent_root = T_parent_tracking * inverse(T_root_tracking)."""
    matrix = transform_matrix(calibration) @ np.linalg.inv(transform_matrix(root_to_tracking))
    result = Transform()
    result.translation.x, result.translation.y, result.translation.z = map(float, matrix[:3, 3])
    w, x, y, z = map(float, mat2quat(matrix[:3, :3]))
    result.rotation.w, result.rotation.x, result.rotation.y, result.rotation.z = w, x, y, z
    return result


class HandeyePublisher(Node):
    def __init__(self):
        super().__init__('handeye_publisher')
        self.declare_parameter('name', '')
        self.declare_parameter('tracking_root_frame', 'camera_link')
        name = self.get_parameter('name').value
        root = self.get_parameter('tracking_root_frame').value
        self.get_logger().info(f'Loading the calibration with name {name}')
        calibration = load_calibration(name)
        parameters = calibration.parameters
        parent = (parameters.robot_effector_frame if parameters.calibration_type == 'eye_in_hand'
                  else parameters.robot_base_frame)
        child = parameters.tracking_base_frame
        transform = calibration.transform
        self.calibration_name = name
        self.internal_transform = None
        self.frame_signature = (parameters.calibration_type, parameters.robot_base_frame,
                                parameters.robot_effector_frame, parameters.tracking_base_frame)
        self.publisher_lock = claim_local_resource(root or child)
        if root and root != child:
            if root == parent or root.startswith('/'):
                raise ValueError('tracking_root_frame must be a camera root, not the robot parent')
            buffer = Buffer()
            listener = TransformListener(buffer, self)
            try:
                deadline = time.monotonic() + 10.0
                while rclpy.ok() and not buffer.can_transform(root, child, Time()):
                    if time.monotonic() >= deadline:
                        raise RuntimeError(
                            f'No camera-internal TF {root} -> {child}. Keep the camera running '
                            'and stop the old calibration publisher before restarting this node.'
                        )
                    rclpy.spin_once(self, timeout_sec=0.1)
                internal = buffer.lookup_transform(root, child, Time()).transform
                self.internal_transform = internal
                transform = root_calibration(transform, internal)
                child = root
            finally:
                listener.unregister()
        # Also detect a legacy publisher started before the lock was introduced.
        deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.get_node_names().count(self.get_name()) > 1:
            raise RuntimeError('Another handeye_publisher is running. Stop that publisher '
                               'before restarting; robot bringup need not be restarted.')
        self.broadcaster = CalibrationTransformBroadcaster(self)
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform = transform
        self.broadcaster.sendTransform(message)
        self.published_parent, self.published_child = parent, child
        self.reload_service = self.create_service(
            Trigger, f"/easy_handeye2/{name}/reload_calibration", self.reload_calibration)
        self.get_logger().info(
            f'Published saved calibration TF: {parent} -> {child} '
            f'(calibration tracking frame: {parameters.tracking_base_frame})'
        )


    def reload_calibration(self, request, response):
        """Update the same static edge in all listeners; never add a second publisher."""
        try:
            calibration = load_calibration(self.calibration_name)
            p = calibration.parameters
            signature = (p.calibration_type, p.robot_base_frame,
                         p.robot_effector_frame, p.tracking_base_frame)
            if signature != self.frame_signature:
                raise ValueError('Calibration frames/type changed; restart this publisher explicitly')
            transform = calibration.transform
            if self.internal_transform is not None:
                transform = root_calibration(transform, self.internal_transform)
            matrix = transform_matrix(transform)
            if not np.isfinite(matrix).all():
                raise ValueError('Calibration contains nonfinite values')
            message = TransformStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.published_parent
            message.child_frame_id = self.published_child
            message.transform = transform
            self.broadcaster.sendTransform(message)
            response.success = True
            response.message = (f'Reloaded {self.calibration_name}: '
                                f'{self.published_parent} -> {self.published_child}')
            self.get_logger().info(response.message)
        except Exception as exc:
            response.success = False
            response.message = f'Calibration reload failed; previous TF retained: {exc}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HandeyePublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
            node.publisher_lock.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

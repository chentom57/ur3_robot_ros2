from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    arg_name = DeclareLaunchArgument('name')

    # Calibration only needs robot-base->tool and camera->marker separately.
    # Never bridge them using a fake static TF: RealSense owns optical parents,
    # and long-lived listeners retain static transforms after a publisher exits.

    handeye_server = Node(package='easy_handeye2', executable='handeye_server', name='handeye_server', parameters=[{
        'name': LaunchConfiguration('name'),
        'calibration_type': LaunchConfiguration('calibration_type'),
        'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
        'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
        'robot_base_frame': LaunchConfiguration('robot_base_frame'),
        'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
    }])

    handeye_rqt_calibrator = Node(package='easy_handeye2', executable='rqt_calibrator.py',
                                  name='handeye_rqt_calibrator',
                                  parameters=[{
                                      'name': LaunchConfiguration('name'),
                                      'calibration_type': LaunchConfiguration('calibration_type'),
                                      'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
                                      'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
                                      'robot_base_frame': LaunchConfiguration('robot_base_frame'),
                                      'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
                                  }])

    return LaunchDescription([
        arg_name,
        DeclareLaunchArgument('calibration_type', choices=['eye_in_hand', 'eye_on_base']),
        DeclareLaunchArgument('tracking_base_frame'),
        DeclareLaunchArgument('tracking_marker_frame'),
        DeclareLaunchArgument('robot_base_frame'),
        DeclareLaunchArgument('robot_effector_frame'),
        handeye_server,
        handeye_rqt_calibrator,
    ])

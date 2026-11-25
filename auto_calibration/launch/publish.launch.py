from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name'),
        DeclareLaunchArgument('tracking_root_frame', default_value='camera_link'),
        Node(package='auto_calibration', executable='handeye_publisher',
             name='handeye_publisher', parameters=[{
                 'name': LaunchConfiguration('name'),
                 'tracking_root_frame': LaunchConfiguration('tracking_root_frame'),
             }]),
    ])

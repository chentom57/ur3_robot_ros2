"""Launch the UR3 model, MoveIt, and table with UR mock ros2_control hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    virtual_table_enabled = LaunchConfiguration("virtual_table_enabled")
    virtual_table_z = LaunchConfiguration("virtual_table_z")
    launch_rviz = LaunchConfiguration("launch_rviz")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur3_bringup"), "launch", "bringup.launch.py"]
            )
        ),
        launch_arguments={
            "robot_ip": "127.0.0.1",
            "use_mock_hardware": "true",
            "launch_camera": "false",
            "launch_rviz": launch_rviz,
            "virtual_table_enabled": virtual_table_enabled,
            "virtual_table_z": virtual_table_z,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "virtual_table_enabled",
                default_value="true",
                choices=["true", "false"],
                description="Add the shared virtual table collision slab",
            ),
            DeclareLaunchArgument(
                "virtual_table_z",
                default_value="-0.01",
                description="Virtual table top height in base_link, in meters",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                choices=["true", "false"],
                description="Launch MoveIt RViz",
            ),
            bringup,
        ]
    )

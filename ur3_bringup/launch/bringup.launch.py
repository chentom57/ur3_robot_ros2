from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur3",
            "robot_ip": robot_ip,
            "use_mock_hardware": use_mock_hardware,
            "launch_rviz": "false",
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={"ur_type": "ur3", "launch_rviz": "true"}.items(),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip", default_value="192.168.86.7", description="UR3 controller IP"
            ),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                choices=["true", "false"],
                description="Use mock hardware instead of connecting to the physical UR3",
            ),
            driver,
            moveit,
            camera,
        ]
    )

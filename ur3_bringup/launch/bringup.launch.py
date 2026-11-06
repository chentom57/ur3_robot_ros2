from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    virtual_table_enabled = LaunchConfiguration("virtual_table_enabled")
    virtual_table_z = LaunchConfiguration("virtual_table_z")
    launch_camera = LaunchConfiguration("launch_camera")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_camera_condition = PythonExpression(
        [
            "'",
            launch_camera,
            "' == 'true' or ('",
            launch_camera,
            "' == 'auto' and '",
            use_mock_hardware,
            "' == 'false')",
        ]
    )

    driver = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
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
        ],
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur3",
            "launch_rviz": launch_rviz,
            "virtual_table_enabled": virtual_table_enabled,
            "virtual_table_z": virtual_table_z,
        }.items(),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        condition=IfCondition(launch_camera_condition),
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
            DeclareLaunchArgument(
                "virtual_table_enabled",
                default_value="true",
                choices=["true", "false"],
                description="Add the shared virtual table collision plane to MoveIt",
            ),
            DeclareLaunchArgument(
                "virtual_table_z",
                default_value="-0.01",
                description="Virtual table height in base_link, in meters",
            ),
            DeclareLaunchArgument(
                "launch_camera",
                default_value="auto",
                choices=["auto", "true", "false"],
                description="Launch RealSense automatically by hardware mode, or force true/false",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                choices=["true", "false"],
                description="Launch MoveIt RViz",
            ),
            driver,
            moveit,
            camera,
        ]
    )

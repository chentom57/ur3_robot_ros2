"""Integrate upstream UR MoveIt with the project's scene and RViz display."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    # Upstream starts nodes from process-exit handlers. Capture our settings
    # before including it so its launch_rviz=false cannot affect our RViz later.
    ur_type = LaunchConfiguration('ur_type').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context)
    table_enabled = LaunchConfiguration('virtual_table_enabled').perform(context)
    table_z = float(LaunchConfiguration('virtual_table_z').perform(context))
    config = (MoveItConfigsBuilder(robot_name='ur', package_name='ur_moveit_config')
              .robot_description_semantic(Path('srdf') / 'ur.srdf.xacro', {'name': ur_type})
              .to_moveit_configs())
    upstream = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py'])),
        launch_arguments={
            'ur_type': ur_type,
            'launch_rviz': 'false',
            # Also suppress the extra table in a previously customized local driver.
            'virtual_table_enabled': 'false',
        }.items(),
    )
    table = Node(
        package='ur3_moveit_scene', executable='virtual_table', name='ur3_virtual_table',
        condition=IfCondition(table_enabled),
        parameters=[{'base_frame': 'base_link', 'table_z': table_z}],
        output='screen',
    )
    wait_description = Node(package='ur_robot_driver', executable='wait_for_robot_description',
                            name='wait_project_rviz_description', output='log')
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2_moveit',
        condition=IfCondition(launch_rviz),
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('ur3_bringup'), 'config', 'moveit.rviz'])],
        parameters=[config.robot_description, config.robot_description_semantic,
                    config.robot_description_kinematics, config.planning_pipelines,
                    config.joint_limits, {'use_sim_time': False}],
        output='log',
    )
    return [
        upstream,
        table,
        RegisterEventHandler(OnProcessExit(target_action=wait_description, on_exit=[rviz])),
        wait_description,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur3', choices=['ur3']),
        DeclareLaunchArgument('launch_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('virtual_table_enabled', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('virtual_table_z', default_value='-0.01'),
        OpaqueFunction(function=launch_setup),
    ])

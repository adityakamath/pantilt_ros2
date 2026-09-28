#!/usr/bin/env python3
"""Launch robot_state_publisher only, for visualization; no hardware or controllers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch robot_state_publisher with the URDF expanded from xacro."""
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('pt_description'),
            'urdf',
            'pantilt.urdf.xacro',
        ]),
        ' pantilt_config:=', LaunchConfiguration('pantilt_config'),
        ' camera_config:=', LaunchConfiguration('camera_config'),
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[robot_description],
        name='robot_state_publisher',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('pantilt_config', default_value='pt101', choices=['pt100', 'pt101']),
        DeclareLaunchArgument('camera_config', default_value='gemini2', choices=['gemini2', 'oakd_s2']),
        robot_state_publisher_node,
    ])

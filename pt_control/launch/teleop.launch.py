#!/usr/bin/env python3
"""Launch joy_teleop: /joy to position commands on /pantilt_controller/commands."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Launch joy_teleop with the PT100 axis/button mapping config."""
    teleop_config = PathJoinSubstitution(
        [FindPackageShare('pt_control'), 'config', 'teleop_config.yaml']
    )

    teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        output='screen',
        parameters=[teleop_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from a simulator instead of system time.',
        ),
        teleop_node,
    ])

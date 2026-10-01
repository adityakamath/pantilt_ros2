#!/usr/bin/env python3
"""Pan-tilt control plus the selected Gemini 2 or OAK-D S2 camera driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_arg_as_bool(context, name: str) -> bool:
    """Resolve a launch argument as a strict boolean."""
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise RuntimeError(
        f"[pantilt.launch.py] Launch argument '{name}' must be true/false or 1/0, got {value!r}."
    )


def launch_setup(context):
    """Include control and the selected real camera; MuJoCo supplies its own camera."""
    use_mock     = LaunchConfiguration('use_mock').perform(context)
    sim          = _launch_arg_as_bool(context, 'sim')
    enable_camera = _launch_arg_as_bool(context, 'enable_camera')
    mujoco_gui   = _launch_arg_as_bool(context, 'mujoco_gui')
    mujoco_scene = LaunchConfiguration('mujoco_scene')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    if sim:
        # Sim implies sim time and mock hardware, so no serial port is assumed
        use_sim_time = 'true'
        use_mock = use_mock or 'true'

    hw_type = 'mujoco' if sim else 'real'

    pt_control_args = {
        'sts_serial_port': LaunchConfiguration('sts_serial_port'),
        'use_mock': use_mock,
        'diagnostics': LaunchConfiguration('diagnostics'),
        'pantilt_config': LaunchConfiguration('pantilt_config'),
        'camera_config': LaunchConfiguration('camera_config'),
        'use_sim_time': use_sim_time,
        'ros2_control_hardware_type': hw_type,
    }
    if hw_type == 'mujoco':
        pt_control_args['mujoco_headless'] = 'false' if mujoco_gui else 'true'
        pt_control_args['mujoco_scene'] = mujoco_scene

    pantilt_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('pt_control'),
                'launch',
                'pantilt.launch.py'
            ])
        ),
        launch_arguments=pt_control_args.items()
    )

    actions = [pantilt_control_launch]

    camera_config = LaunchConfiguration('camera_config').perform(context)
    # Construct only the selected camera include, and only when streaming is enabled.
    # A conditional include alone can still be inspected by ROS launch argument discovery.
    if hw_type == 'real' and enable_camera:
        if camera_config == 'gemini2' and _launch_arg_as_bool(context, 'octomap'):
            raise RuntimeError('Gemini 2 octomap is not integrated; use octomap:=false.')
        camera_args = {
            'pointcloud': LaunchConfiguration('pointcloud'),
            'camera_fps': LaunchConfiguration('camera_fps'),
            'octomap': LaunchConfiguration('octomap'),
        }
        if camera_config == 'gemini2':
            for name in ('serial_number', 'usb_port'):
                camera_args[name] = LaunchConfiguration(name)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pt_bringup'), 'launch',
                'gemini2.launch.py' if camera_config == 'gemini2' else 'oakd.launch.py',
            ])),
            launch_arguments=camera_args.items(),
        ))
    # mujoco: nothing more to start, pt_control's ros2_control_node hosts the simulation

    return actions


def generate_launch_description():
    """Declare launch arguments and include pt_control's and pt_bringup's own launch files."""
    declared_arguments = [
        DeclareLaunchArgument(
            'camera_config', default_value='gemini2', choices=['gemini2', 'oakd_s2'],
            description='Camera geometry and real driver selection.',
        ),
        DeclareLaunchArgument(
            'enable_camera', default_value='true',
            description='Start the selected real camera driver and streaming pipeline. '
                        'False preserves camera geometry in the URDF; simulation is unaffected.',
        ),
        DeclareLaunchArgument('serial_number', default_value='', description='Gemini 2 serial selector.'),
        DeclareLaunchArgument('usb_port', default_value='', description='Gemini 2 USB port selector.'),
        DeclareLaunchArgument(
            'sts_serial_port',
            default_value='',
            description='Serial port override; empty string means use urdf_config.yaml value',
        ),
        DeclareLaunchArgument(
            'use_mock',
            default_value='',
            description='Mock mode override (true/false); empty string means use urdf_config.yaml value',
        ),
        DeclareLaunchArgument(
            'diagnostics',
            default_value='false',
            description='Launch motor diagnostics node',
        ),
        DeclareLaunchArgument(
            'pantilt_config',
            default_value='pt101',
            description='Pan-tilt mesh variant: "pt100" or "pt101" (pt101 is recommended and default)',
        ),
        DeclareLaunchArgument('camera_fps', default_value='15',
                              description='RGB/depth frame rate for either camera: 5, 10, 15, or 30 Hz.'),
        DeclareLaunchArgument(
            'pointcloud',
            default_value='false',
            description='Enable RGBD point cloud pipeline.',
        ),
        DeclareLaunchArgument(
            'octomap',
            default_value='false',
            description='Run octomap_server on the OAK-D point cloud. Requires pointcloud:=true '
                        '(validated in oakd.launch.py, not here).',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from a simulator instead of system time.',
        ),
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Run against MuJoCo instead of real hardware: forces use_sim_time/use_mock, '
                        'and skips real camera drivers (the simulated camera comes from pt_mujoco).',
        ),
        DeclareLaunchArgument(
            'mujoco_gui',
            default_value='false',
            description='[sim only] Launch with the MuJoCo Simulate viewer attached (headless otherwise).',
        ),
        DeclareLaunchArgument(
            'mujoco_scene',
            default_value='flat',
            description='[sim only] flat, none, or scene MJCF path.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

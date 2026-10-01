#!/usr/bin/env python3
"""Launch Orbbec's Gemini 2 driver and attach its sensor tree to the pan-tilt."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in ('true', 'false', '1', '0'):
        raise RuntimeError(f'{name} must be true/false or 1/0, got {value!r}')
    return value in ('true', '1')


def launch_setup(context):
    if not _boolean(context, 'enable_camera'):
        return []
    pointcloud = 'true' if _boolean(context, 'pointcloud') else 'false'
    fps = LaunchConfiguration('camera_fps').perform(context).strip()
    if fps not in ('5', '10', '15', '30'):
        raise RuntimeError('camera_fps must be 5, 10, 15, or 30 for the real camera')
    if _boolean(context, 'octomap'):
        raise RuntimeError('Gemini 2 octomap is not integrated; use octomap:=false. '
                           'pointcloud:=true publishes /gemini2/depth_registered/points.')
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('orbbec_camera'), 'launch', 'gemini2.launch.py',
        ])),
        launch_arguments={
            'camera_name': 'gemini2',
            'serial_number': LaunchConfiguration('serial_number'),
            'usb_port': LaunchConfiguration('usb_port'),
            'enable_color': 'true',
            'color_width': '640',
            'color_height': '360',
            'color_fps': fps,
            'enable_depth': 'true',
            'depth_width': '640',
            'depth_height': '400',
            'depth_fps': fps,
            'enable_ir': 'false',
            'enable_accel': 'true',
            'enable_gyro': 'true',
            'enable_sync_output_accel_gyro': 'true',
            'depth_registration': 'true',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': pointcloud,
            'publish_tf': 'true',
        }.items(),
    )
    # Convert depth directly in both point-cloud modes; a cloud is not needed for a 2D scan.
    # image_transport loads every installed plugin per topic; keep only the useful ones
    actions = [
        SetParameter('color.image_raw.enable_pub_plugins',
                     ['image_transport/raw', 'image_transport/compressed']),
        SetParameter('depth.image_raw.enable_pub_plugins',
                     ['image_transport/raw', 'image_transport/compressedDepth']),
        driver, Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='gemini2_depth_to_scan',
        output='log',
        parameters=[PathJoinSubstitution([
            FindPackageShare('pt_bringup'), 'config', 'gemini2_depth_to_scan.yaml',
        ])],
        remappings=[
            ('depth', '/gemini2/depth/image_raw'),
            ('depth_camera_info', '/gemini2/depth/camera_info'),
            ('scan', '/gemini2/scan'),
        ],
    )]
    if pointcloud == 'true':
        actions.append(ComposableNodeContainer(
            name='gemini2_cloudini_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[ComposableNode(
                package='pt_bringup',
                plugin='pt_bringup::PCLCompressorNode',
                name='gemini2_cloudini_compressor',
                parameters=[PathJoinSubstitution([
                    FindPackageShare('pt_bringup'), 'config', 'gemini2_pcl.yaml',
                ])],
            )],
            output='both',
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_camera', default_value='true',
                              description='Start the camera driver and streaming helpers.'),
        DeclareLaunchArgument('pointcloud', default_value='false',
                              description='Publish one colored Orbbec point cloud and compress it with Cloudini.'),
        DeclareLaunchArgument('camera_fps', default_value='15',
                              description='RGB/depth frame rate: 5, 10, 15, or 30 Hz.'),
        DeclareLaunchArgument('octomap', default_value='false',
                              description='Not integrated for Gemini 2; must remain false.'),
        DeclareLaunchArgument('serial_number', default_value='',
                              description='Orbbec device serial; empty selects the first device.'),
        DeclareLaunchArgument('usb_port', default_value='',
                              description='Optional Orbbec USB port selector.'),
        OpaqueFunction(function=launch_setup),
    ])

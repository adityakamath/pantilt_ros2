#!/usr/bin/env python3
"""Launch the OAK-D S2 camera with depthai_ros_driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterFile


def _launch_arg_as_bool(context, name: str) -> bool:
    """Resolve a launch argument as a strict boolean."""
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise RuntimeError(
        f"[oakd.launch.py] Launch argument '{name}' must be true/false or 1/0, got {value!r}."
    )


def generate_launch_description():
    """Declare arguments and defer camera node setup to launch_setup via OpaqueFunction."""
    declared_arguments = [
        DeclareLaunchArgument(
            'pointcloud',
            default_value='false',
            description='Layer oakd_vio_pcl.yaml on top of oakd_vio.yaml (depth aligned to '
                         'RGB + point cloud) instead of oakd_vio.yaml alone (depth unaligned, '
                         'no point cloud). Both publish /oak/scan via depth_to_scan. Also '
                         'gates cloudini compression, which needs the point cloud that only '
                         'exists in this mode. Required for octomap:=true.',
        ),
        DeclareLaunchArgument(
            'octomap',
            default_value='false',
            description='Run octomap_server on /oak/rgbd/points to build a persistent 3D '
                         'octree. Only takes effect when pointcloud:=true (no point cloud to '
                         'consume otherwise).',
        ),
        DeclareLaunchArgument(
            'tf_parent_frame',
            default_value='tilt_link',
            description="TF frame the OAK-D S2 is mounted to. Default 'tilt_link' is correct "
                         "for pantilt_ros2; override when mounting the camera elsewhere "
                         "(e.g. directly on a host robot without the pan-tilt).",
        ),
    ]

    def launch_setup(context, *_args, **_kwargs):
        """Start the OAK-D composable node container, layering oakd_vio_pcl.yaml on top of
        oakd_vio.yaml when pointcloud:=true."""
        log_level = 'info'
        if context.environment.get('DEPTHAI_DEBUG') == '1':
            log_level = 'debug'

        pointcloud = _launch_arg_as_bool(context, 'pointcloud')
        octomap = _launch_arg_as_bool(context, 'octomap')
        tf_parent_frame = LaunchConfiguration('tf_parent_frame').perform(context)

        config_dir = os.path.join(get_package_share_directory('pt_bringup'), 'config')
        base_params_file = ParameterFile(os.path.join(config_dir, 'oakd_vio.yaml'), allow_substs=True)
        oak_parameters = [base_params_file]
        if pointcloud:
            pcl_overlay_file = ParameterFile(os.path.join(config_dir, 'oakd_vio_pcl.yaml'), allow_substs=True)
            oak_parameters.append(pcl_overlay_file)
        oak_parameters.append({
            'driver': {
                'i_tf_parent_frame': tf_parent_frame,
                'i_tf_camera_model': 'OAK-D-S2',
                'i_tf_base_frame': 'oak_link',
            }
        })

        # Build composable node list - always include OAK-D driver
        composable_nodes = [
            ComposableNode(
                package='depthai_ros_driver',
                plugin='depthai_ros_driver::Driver',
                name='oak',
                parameters=oak_parameters,
            ),
            # Slices /oak/stereo/image_raw into /oak/scan - runs regardless of pointcloud.
            ComposableNode(
                package='depthimage_to_laserscan',
                plugin='depthimage_to_laserscan::DepthImageToLaserScanROS',
                name='depth_to_scan',
                parameters=[
                    os.path.join(
                        get_package_share_directory('pt_bringup'),
                        'config',
                        'depthimage_to_laserscan.yaml',
                    )
                ],
                remappings=[
                    ('depth', '/oak/stereo/image_raw'),
                    ('depth_camera_info', '/oak/stereo/camera_info'),
                    ('scan', '/oak/scan'),
                ],
            ),
        ]

        # cloudini compression and octomap need /oak/rgbd/points, which only exists when
        # pointcloud:=true.
        if pointcloud:
            composable_nodes.append(
                ComposableNode(
                    package='pt_bringup',
                    plugin='pt_bringup::PCLCompressorNode',
                    name='cloudini_compressor',
                    parameters=[pcl_overlay_file],
                )
            )
            if octomap:
                composable_nodes.append(
                    ComposableNode(
                        package='octomap_server',
                        plugin='octomap_server::OctomapServer',
                        name='octomap_server',
                        parameters=[
                            os.path.join(
                                get_package_share_directory('pt_bringup'),
                                'config',
                                'octomap.yaml',
                            )
                        ],
                        remappings=[
                            ('cloud_in', '/oak/rgbd/points'),
                        ],
                    )
                )

        return [
            ComposableNodeContainer(
                name='oak_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=composable_nodes,
                arguments=['--ros-args', '--log-level', log_level],
                output='both',
            ),
        ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

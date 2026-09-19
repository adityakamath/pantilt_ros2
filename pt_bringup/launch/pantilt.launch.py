#!/usr/bin/env python3
"""
Main bringup launch file for the complete Pan Tilt 100 system: mechanism + camera.

This launch file includes:
    - pt_control's pantilt.launch.py (robot control stack)
    - pt_bringup's oakd.launch.py (OAK-D camera) - real hardware only; sim:=true (MuJoCo)
      skips it, mujoco_ros2_control's own ros2_control_node is self-contained
It forwards relevant launch arguments to each included launch file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_arg_as_bool(context, name: str) -> bool:
    """Resolve a launch argument as a strict boolean."""
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ("true", "1"):
        return True
    if value in ("false", "0"):
        return False
    raise RuntimeError(
        f"[pantilt.launch.py] Launch argument '{name}' must be true/false or 1/0, got {value!r}."
    )


def launch_setup(context):
    """Include pt_control's control stack, plus either oakd (real) or nothing more (sim - MuJoCo is self-contained)."""
    use_mock     = LaunchConfiguration("use_mock").perform(context)
    sim          = _launch_arg_as_bool(context, "sim")
    mujoco_gui   = LaunchConfiguration("mujoco_gui").perform(context)
    mujoco_scene = LaunchConfiguration("mujoco_scene")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)

    if sim:
        # Running in sim implies sim time; keep any explicit use_mock override, otherwise
        # force it so the STS plugin (unused in sim, but still resolved by xacro defaults)
        # never assumes a real serial port is present.
        use_sim_time = "true"
        use_mock = use_mock or "true"

    hw_type = "mujoco" if sim else "real"

    pt_control_args = {
        "sts_serial_port": LaunchConfiguration("sts_serial_port"),
        "use_mock": use_mock,
        "diagnostics": LaunchConfiguration("diagnostics"),
        "pantilt_config": LaunchConfiguration("pantilt_config"),
        "use_sim_time": use_sim_time,
        "ros2_control_hardware_type": hw_type,
    }
    if hw_type == "mujoco":
        pt_control_args["mujoco_headless"] = "false" if mujoco_gui.lower() in ("true", "1") else "true"
        pt_control_args["mujoco_scene"] = mujoco_scene

    pantilt_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("pt_control"),
                "launch",
                "pantilt.launch.py"
            ])
        ),
        launch_arguments=pt_control_args.items()
    )

    actions = [pantilt_control_launch]

    if hw_type == "real":
        oakd_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("pt_bringup"),
                    "launch",
                    "oakd.launch.py"
                ])
            ),
            launch_arguments={
                "pointcloud": LaunchConfiguration("pointcloud"),
                "octomap": LaunchConfiguration("octomap"),
                "tf_parent_frame": LaunchConfiguration("tf_parent_frame"),
            }.items()
        )
        actions.append(oakd_launch)
    # hw_type == "mujoco": nothing more to start - mujoco_ros2_control's own
    # ros2_control_node (started by pt_control/launch/pantilt.launch.py above) hosts the
    # MuJoCo simulation itself, no separate simulator process, bridge, or entity spawn needed.

    return actions


def generate_launch_description():
    """Declare launch arguments and include pt_control's and pt_bringup's own launch files."""
    declared_arguments = [
        DeclareLaunchArgument(
            "sts_serial_port",
            default_value="",
            description="Serial port override; empty string means use urdf_config.yaml value",
        ),
        DeclareLaunchArgument(
            "use_mock",
            default_value="",
            description="Mock mode override (true/false); empty string means use urdf_config.yaml value",
        ),
        DeclareLaunchArgument(
            "diagnostics",
            default_value="false",
            description="Launch motor diagnostics node",
        ),
        DeclareLaunchArgument(
            "pantilt_config",
            default_value="pt101",
            description='Pan-tilt mesh variant: "pt100" or "pt101" (pt101 is recommended and default)',
        ),
        DeclareLaunchArgument(
            "pointcloud",
            default_value="false",
            description="Enable RGBD point cloud pipeline.",
        ),
        DeclareLaunchArgument(
            "octomap",
            default_value="false",
            description="Run octomap_server on the OAK-D point cloud. Requires pointcloud:=true "
                        "(validated in oakd.launch.py, not here).",
        ),
        DeclareLaunchArgument(
            "tf_parent_frame",
            default_value="tilt_link",
            description="TF frame the OAK-D S2 is mounted to; see oakd.launch.py.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock from a simulator instead of system time.",
        ),
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="Run against MuJoCo instead of real hardware: forces use_sim_time/use_mock, "
                        "and skips oakd (the simulated camera comes from pt_mujoco).",
        ),
        DeclareLaunchArgument(
            "mujoco_gui",
            default_value="false",
            description="[sim only] Launch with the MuJoCo Simulate viewer attached (headless otherwise).",
        ),
        DeclareLaunchArgument(
            "mujoco_scene",
            default_value="flat",
            description="[sim only] flat, none, or scene MJCF path.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

#!/usr/bin/env python3
"""
Smoke tests for pt_description's URDF xacro files (the MJCF lives in pt_mujoco).

Pure xacro-processing + XML-structure checks: no ROS graph, no rclpy, no nodes. Runs `xacro`
as a subprocess and inspects its output - mirrors so_arm_description's test_urdf_xacro.py in
so_arm_ros2 (the two robots share the same first two joints, so their test structure should
too).
"""

import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest

_SHARE = get_package_share_directory('pt_description')

_CONFIGS = ('pt100', 'pt101')
_MOVABLE_JOINTS = ('shoulder_pan_joint', 'tilt_joint')
_HARDWARE_PLUGINS = {
    'real': 'sts_hardware_interface/STSHardwareInterface',
    'mujoco': 'mujoco_ros2_control/MujocoSystemInterface',
    'gazebo': 'gz_ros2_control/GazeboSimSystem',
}


def _run_xacro(*args):
    result = subprocess.run(
        ['xacro', *args], capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, f'xacro failed for {args}:\n{result.stderr}'
    return result.stdout


def _process_urdf(pantilt_config, **mappings):
    xacro_file = os.path.join(_SHARE, 'urdf', 'pantilt.urdf.xacro')
    args = [xacro_file, f'pantilt_config:={pantilt_config}']
    args += [f'{key}:={value}' for key, value in mappings.items()]
    return ET.fromstring(_run_xacro(*args))


def _mesh_paths(root):
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename')
        if filename and filename.startswith('package://pt_description/'):
            yield filename.removeprefix('package://pt_description/')


# ── URDF xacro ──────────────────────────────────────────────────────────────

@pytest.mark.parametrize('pantilt_config', _CONFIGS)
class TestUrdfXacroDefaults:

    def test_processes_to_valid_xml(self, pantilt_config):
        root = _process_urdf(pantilt_config)
        assert root.tag == 'robot'

    def test_movable_joints_present_with_valid_limits(self, pantilt_config):
        root = _process_urdf(pantilt_config)
        joints = {j.get('name'): j for j in root.findall('joint')}
        for name in _MOVABLE_JOINTS:
            assert name in joints, f'{pantilt_config}: missing joint {name}'
            limit = joints[name].find('limit')
            assert limit is not None, f'{pantilt_config}: {name} has no <limit>'
            lower, upper = float(limit.get('lower')), float(limit.get('upper'))
            assert lower < upper, f'{pantilt_config}: {name} lower={lower} >= upper={upper}'

    def test_ros2_control_block_covers_every_movable_joint(self, pantilt_config):
        root = _process_urdf(pantilt_config)
        ros2_control = root.find('ros2_control')
        assert ros2_control is not None
        rc_joints = {j.get('name') for j in ros2_control.findall('joint')}
        assert rc_joints == set(_MOVABLE_JOINTS)

    def test_referenced_meshes_exist_on_disk(self, pantilt_config):
        root = _process_urdf(pantilt_config)
        paths = list(_mesh_paths(root))
        assert paths, f'{pantilt_config}: no mesh references found'
        for rel_path in paths:
            full_path = os.path.join(_SHARE, rel_path)
            assert os.path.isfile(full_path), f'{pantilt_config}: missing mesh {full_path}'

    def test_command_interface_is_position_only(self, pantilt_config):
        root = _process_urdf(pantilt_config)
        ros2_control = root.find('ros2_control')
        for joint in ros2_control.findall('joint'):
            cmd_interfaces = {c.get('name') for c in joint.findall('command_interface')}
            assert cmd_interfaces == {'position'}, (
                f'{pantilt_config}: {joint.get("name")} command interfaces {cmd_interfaces} '
                "!= {'position'}"
            )

    def test_parent_child_chain(self, pantilt_config):
        """pantilt_base_link -> shoulder_link -> tilt_link -> oak_link."""
        root = _process_urdf(pantilt_config)
        parent_of = {j.find('child').get('link'): j.find('parent').get('link')
                     for j in root.findall('joint')}
        assert parent_of['shoulder_link'] == 'pantilt_base_link'
        assert parent_of['tilt_link'] == 'shoulder_link'
        assert parent_of['oak_link'] == 'tilt_link'


@pytest.mark.parametrize('pantilt_config', _CONFIGS)
@pytest.mark.parametrize('hardware_type', ('real', 'mujoco', 'gazebo'))
def test_ros2_control_hardware_plugin_matches_type(pantilt_config, hardware_type):
    root = _process_urdf(pantilt_config, ros2_control_hardware_type=hardware_type)
    plugin = root.find('ros2_control/hardware/plugin')
    assert plugin is not None, f'{pantilt_config}/{hardware_type}: no <plugin> emitted'
    assert plugin.text == _HARDWARE_PLUGINS[hardware_type]


# ── servo tuning owned by the module ────────────────────────────────────────

@pytest.mark.parametrize('pantilt_config', _CONFIGS)
def test_internal_tuning_defaults_are_the_slow_smooth_ones(pantilt_config):
    """65/50/0 come from pantilt.joints.xacro's macro defaults, for standalone and any host."""
    root = _process_urdf(pantilt_config)
    joints = root.find('ros2_control').findall('joint')
    assert {j.get('name') for j in joints} == set(_MOVABLE_JOINTS)
    for joint in joints:
        params = {p.get('name'): p.text.strip() for p in joint.findall('param')}
        assert (params['internal_max_vel'], params['internal_max_acc'], params['internal_acc_coeff']) == \
            ('65', '50', '0'), joint.get('name')

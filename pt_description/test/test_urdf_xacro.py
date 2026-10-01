#!/usr/bin/env python3
"""Smoke tests for the URDF xacro files: runs xacro as a subprocess, no ROS graph."""

import os
import math
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
        assert 'meshes/gemini2.stl' in paths
        assert 'meshes/tilt_joint_gemini2.stl' in paths
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
        """The camera-specific frame is directly mounted to tilt_link."""
        root = _process_urdf(pantilt_config)
        parent_of = {j.find('child').get('link'): j.find('parent').get('link')
                     for j in root.findall('joint')}
        assert parent_of['shoulder_link'] == 'pantilt_base_link'
        assert parent_of['tilt_link'] == 'shoulder_link'
        assert parent_of['gemini2_link'] == 'tilt_link'


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


@pytest.mark.parametrize('pantilt_config', _CONFIGS)
@pytest.mark.parametrize('hardware_type,unlimited', [('real', True), ('mujoco', True), ('gazebo', False)])
def test_velocity_limit_is_unlimited_except_where_the_simulator_enforces_it(pantilt_config, hardware_type, unlimited):
    """joy_teleop's absolute jumps must not be rate-limited on real hardware or in MuJoCo."""
    root = _process_urdf(pantilt_config, ros2_control_hardware_type=hardware_type)
    for name in _MOVABLE_JOINTS:
        velocity = float(root.find(f"joint[@name='{name}']/limit").get('velocity'))
        assert (velocity >= 1e6) == unlimited, (hardware_type, name, velocity)



@pytest.mark.parametrize('pantilt_config', _CONFIGS)
@pytest.mark.parametrize('camera_config', ('oakd_s2', 'gemini2'))
def test_camera_mesh_selection(pantilt_config, camera_config):
    root = _process_urdf(pantilt_config, camera_config=camera_config)
    tilt = root.find("link[@name='tilt_link']")
    camera_frame = 'gemini2_link' if camera_config == 'gemini2' else 'oak_link'
    model_frame = f'{camera_frame}_model_origin'
    for role in ('visual', 'collision'):
        path = tilt.find(f'{role}/geometry/mesh').get('filename')
        assert path.endswith(f'/tilt_joint_{camera_config}.stl')
    camera = root.find(f"link[@name='{model_frame}']/visual/geometry/mesh")
    assert camera.get('filename').endswith(f'/{camera_config}.stl')
    assert root.find(f"link[@name='{camera_frame}']") is not None
    mount_joint = root.find(f"joint[@name='{camera_frame}_center_joint']")
    assert mount_joint.find('parent').get('link') == 'tilt_link'
    origin = mount_joint.find('origin')
    xyz = [float(value) for value in origin.get('xyz').split()]
    rpy = [float(value) for value in origin.get('rpy').split()]
    assert xyz == pytest.approx(
        [-0.0306, -0.0068941 if camera_config == 'gemini2' else -0.012165,
         -0.000225 if camera_config == 'gemini2' else 0.0]
    )
    assert rpy == pytest.approx([
        math.pi / 2 if camera_config == 'gemini2' else -math.pi / 2,
        0.0,
        -math.pi / 2,
    ])
    assert (root.find("link[@name='oak_link']") is not None) == (camera_config == 'oakd_s2')
    for mesh in _mesh_paths(root):
        assert os.path.isfile(os.path.join(_SHARE, mesh))


@pytest.mark.parametrize('mesh_name', ('gemini2.stl', 'tilt_joint_gemini2.stl'))
def test_gemini_stl_defers_color_to_urdf(mesh_name):
    """Binary STL color extensions can override the URDF material in mesh viewers."""
    import struct
    from pathlib import Path

    data = (Path(_SHARE) / 'meshes' / mesh_name).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84 + count * 50
    assert b'COLOR=' not in data[:80]
    assert all(struct.unpack_from('<H', data, 84 + i * 50 + 48)[0] == 0
               for i in range(count))

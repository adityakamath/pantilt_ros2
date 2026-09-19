"""Launch-file structure checks for the simulation arguments: no ROS graph, no nodes started."""
import importlib.util
from pathlib import Path
import re

from launch.actions import DeclareLaunchArgument
import pytest
import yaml

REPOSITORY = Path(__file__).resolve().parents[2]


def declared(package, filename='pantilt.launch.py'):
    path = REPOSITORY / package / 'launch' / filename
    spec = importlib.util.spec_from_file_location(f'{package}_{path.stem}', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    arguments = [entry for entry in module.generate_launch_description().entities
                 if isinstance(entry, DeclareLaunchArgument)]
    return {argument.name: ''.join(part.text for part in argument.default_value or [])
            for argument in arguments}


def test_control_launch_declares_the_mujoco_arguments():
    arguments = declared('pt_control')
    assert arguments['ros2_control_hardware_type'] == 'real'
    assert arguments['mujoco_scene'] == 'flat'
    assert arguments['mujoco_model'] == ''
    assert arguments['mujoco_headless'] == 'false'


def test_bringup_selects_the_viewer_with_mujoco_gui_and_defaults_to_headless():
    arguments = declared('pt_bringup')
    assert arguments['sim'] == 'false'
    assert arguments['mujoco_gui'] == 'false'
    assert arguments['mujoco_scene'] == 'flat'
    assert 'gui' not in arguments


@pytest.mark.parametrize('name', ['mujoco_ros2_control_plugins.yaml'])
def test_simulation_config_lives_in_pt_mujoco(name):
    assert (REPOSITORY / 'pt_mujoco/config' / name).is_file()
    assert not (REPOSITORY / 'pt_control/config' / name).exists()


def test_control_launch_loads_the_plugin_config_from_pt_mujoco():
    source = (REPOSITORY / 'pt_control/launch/pantilt.launch.py').read_text()
    assert '{pkg_mujoco}/config/mujoco_ros2_control_plugins.yaml' in source
    assert 'pt_mujoco.build_mujoco_models' in source


def test_servo_profile_keys_are_set_in_range_and_match_the_modules_defaults():
    config = yaml.safe_load((REPOSITORY / 'pt_control/config/urdf_config.yaml').read_text())
    macro = (REPOSITORY / 'pt_description/urdf/pantilt.joints.xacro').read_text()
    for key in ('internal_max_vel', 'internal_max_acc', 'internal_acc_coeff'):
        assert isinstance(config[key], int) and 0 <= config[key] <= 254, key
        # The explicit tunable values start out equal to the module's built-in defaults.
        assert re.search(rf'^\s*{key}:={config[key]}\s*$', macro, re.M), key


def test_control_launch_passes_the_servo_profile_to_the_urdf():
    source = (REPOSITORY / 'pt_control/launch/pantilt.launch.py').read_text()
    for key in ('internal_max_vel', 'internal_max_acc', 'internal_acc_coeff'):
        assert f"{key}:={{_cfg[\"{key}\"]}}" in source, key


def test_sim_launch_slices_the_simulated_depth_into_oak_scan_like_the_real_bringup():
    source = (REPOSITORY / 'pt_control/launch/pantilt.launch.py').read_text()
    assert "executable='depthimage_to_laserscan_node'" in source
    assert "('depth', '/oak/stereo/image_raw')" in source and "('scan', '/oak/scan')" in source
    assert (REPOSITORY / 'pt_mujoco/config/mujoco_depth_to_scan.yaml').is_file()

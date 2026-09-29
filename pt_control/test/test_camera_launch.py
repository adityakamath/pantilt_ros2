"""Exercise real-camera routing and Gemini arguments without starting hardware."""
import importlib.util
import math
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest

ROOT = Path(__file__).resolve().parents[2]


def load(filename):
    spec = importlib.util.spec_from_file_location(filename, ROOT / 'pt_bringup/launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context_for(module, **overrides):
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(overrides)
    return context


def resolved_arguments(action, context):
    return {name: value if isinstance(value, str) else perform_substitutions(context, [value])
            for name, value in action.launch_arguments}


@pytest.mark.parametrize('camera,parent,filename', [
    ('gemini2', 'oak_link', 'gemini2.launch.py'),
    ('oakd_s2', 'tilt_link', 'oakd.launch.py'),
])
def test_real_bringup_selects_exactly_one_camera(camera, parent, filename, monkeypatch):
    module = load('pantilt.launch.py')
    monkeypatch.setattr(module, 'FindPackageShare', lambda package: str(ROOT / package))
    context = context_for(module, camera_config=camera, serial_number='test-serial')
    actions = module.launch_setup(context)
    assert len(actions) == 2
    assert all(isinstance(action, IncludeLaunchDescription) for action in actions)
    source = actions[1].launch_description_source
    source.get_launch_description(context)
    assert source.location.endswith(filename)
    arguments = resolved_arguments(actions[1], context)
    assert arguments['tf_parent_frame'] == parent
    assert ('serial_number' in arguments) == (camera == 'gemini2')
    if camera == 'gemini2':
        assert arguments['serial_number'] == 'test-serial'
        assert arguments['camera_fps'] == '15'


@pytest.mark.parametrize('camera', ['gemini2', 'oakd_s2'])
def test_simulation_skips_all_real_drivers(camera):
    module = load('pantilt.launch.py')
    context = context_for(module, camera_config=camera, sim='true')
    actions = module.launch_setup(context)
    assert len(actions) == 1
    assert resolved_arguments(actions[0], context)['ros2_control_hardware_type'] == 'mujoco'


def test_parent_and_calibration_overrides_are_forwarded():
    module = load('pantilt.launch.py')
    context = context_for(module, tf_parent_frame='host_mount', camera_mount_xyz='0.1 0.2 0.3')
    arguments = resolved_arguments(module.launch_setup(context)[1], context)
    assert arguments['tf_parent_frame'] == 'host_mount'
    assert arguments['camera_mount_xyz'] == '0.1 0.2 0.3'


@pytest.mark.parametrize('pointcloud', ['true', 'false'])
def test_gemini_streams_and_pointcloud_selection(pointcloud):
    module = load('gemini2.launch.py')
    context = context_for(module, pointcloud=pointcloud, serial_number='test-serial')
    actions = module.launch_setup(context)
    assert len(actions) == (3 if pointcloud == 'true' else 2)
    driver, mount = actions[0], actions[-1]
    if pointcloud == 'true':
        from launch_ros.actions import ComposableNodeContainer
        assert isinstance(actions[1], ComposableNodeContainer)
        compressor = actions[1]._ComposableNodeContainer__composable_node_descriptions[0]
        assert perform_substitutions(context, compressor.package) == 'pt_bringup'
        assert perform_substitutions(context, compressor.node_plugin) == 'pt_bringup::PCLCompressorNode'
    arguments = resolved_arguments(driver, context)
    assert arguments['camera_name'] == 'gemini2'
    assert arguments['serial_number'] == 'test-serial'
    for name in ('enable_color', 'enable_depth', 'enable_accel', 'enable_gyro',
                 'enable_sync_output_accel_gyro', 'publish_tf', 'depth_registration'):
        assert arguments[name] == 'true'
    assert arguments['enable_point_cloud'] == 'false'
    assert arguments['color_width'] == '640'
    assert arguments['color_height'] == '360'
    assert arguments['depth_width'] == '640'
    assert arguments['depth_height'] == '400'
    assert arguments['color_fps'] == arguments['depth_fps'] == '15'
    assert arguments['enable_colored_point_cloud'] == pointcloud
    assert isinstance(mount, Node)
    command = []
    for part in mount.cmd:
        value = perform_substitutions(context, part)
        if value == '--ros-args':
            break
        command.append(value)
    assert command[command.index('--frame-id') + 1] == 'oak_link'
    assert command[command.index('--child-frame-id') + 1] == 'gemini2_link'
    assert float(command[command.index('--roll') + 1]) == pytest.approx(math.pi)


def test_mount_transform_can_be_owned_by_another_robot():
    module = load('gemini2.launch.py')
    context = context_for(module, publish_mount_tf='false')
    assert len(module.launch_setup(context)) == 1


@pytest.mark.parametrize('overrides', [
    {'octomap': 'true'}, {'pointcloud': 'maybe'}, {'publish_mount_tf': 'maybe'},
    {'tf_parent_frame': 'gemini2_link'}, {'tf_parent_frame': ''},
    {'camera_mount_xyz': '0 0'}, {'camera_mount_rpy': 'nan 0 0'},
    {'camera_mount_xyz': 'bad input here'},
])
def test_invalid_or_unsupported_camera_settings_fail(overrides):
    module = load('gemini2.launch.py')
    with pytest.raises(RuntimeError):
        module.launch_setup(context_for(module, **overrides))


def test_unsupported_gemini_octomap_fails_before_control_starts():
    module = load('pantilt.launch.py')
    with pytest.raises(RuntimeError, match='octomap'):
        module.launch_setup(context_for(module, octomap='true'))


@pytest.mark.parametrize('camera', ['gemini2', 'oakd_s2'])
@pytest.mark.parametrize('enabled', ['false', '0'])
def test_disabled_camera_keeps_geometry_without_loading_camera_launch(camera, enabled, monkeypatch):
    module = load('pantilt.launch.py')

    def control_only(package):
        assert package == 'pt_control', f'Unexpected camera dependency: {package}'
        return str(ROOT / package)

    monkeypatch.setattr(module, 'FindPackageShare', control_only)
    context = context_for(module, camera_config=camera, enable_camera=enabled,
                          pointcloud='true', octomap='true')
    actions = module.launch_setup(context)
    assert len(actions) == 1
    arguments = resolved_arguments(actions[0], context)
    assert arguments['camera_config'] == camera
    assert arguments['ros2_control_hardware_type'] == 'real'
    assert 'enable_camera' not in arguments  # Geometry generation is independent of streaming.


def test_enable_camera_defaults_to_true_and_rejects_invalid_values():
    module = load('pantilt.launch.py')
    assert context_for(module).launch_configurations['enable_camera'] == 'true'
    with pytest.raises(RuntimeError, match='enable_camera'):
        module.launch_setup(context_for(module, enable_camera='maybe'))


def test_gemini_wrapper_resolves_only_orbbec(monkeypatch):
    from launch import LaunchDescription
    from launch_ros.substitutions import FindPackageShare

    module = load('gemini2.launch.py')
    context = context_for(module)
    requests = []

    def selected_only(self, package):
        requests.append(package)
        assert package == 'orbbec_camera', f'Unselected camera dependency: {package}'
        return '/unused/orbbec_camera'

    monkeypatch.setattr(FindPackageShare, 'find', selected_only)
    driver = module.launch_setup(context)[0]
    source = driver.launch_description_source
    monkeypatch.setattr(source, '_get_launch_description', lambda path: LaunchDescription())
    source.get_launch_description(context)
    assert requests == ['orbbec_camera']


def test_oak_wrapper_constructs_depthai_without_orbbec(monkeypatch):
    module = load('oakd.launch.py')
    context = context_for(module)

    def own_config_only(package):
        assert package == 'pt_bringup', f'Unexpected package lookup: {package}'
        return str(ROOT / package)

    monkeypatch.setattr(module, 'get_package_share_directory', own_config_only)
    monkeypatch.setattr(module, 'ComposableNodeContainer', lambda **kwargs: kwargs)
    setup = module.generate_launch_description().entities[-1]
    container = setup.execute(context)[0]
    packages = {perform_substitutions(context, node.package)
                for node in container['composable_node_descriptions']}
    assert 'depthai_ros_driver' in packages
    assert 'orbbec_camera' not in packages


@pytest.mark.parametrize('filename', ['gemini2.launch.py', 'oakd.launch.py'])
def test_camera_only_launch_skips_all_dependencies_when_disabled(filename, monkeypatch):
    module = load(filename)
    context = context_for(module, enable_camera='false', pointcloud='true', octomap='true')

    def unexpected(*args, **kwargs):
        pytest.fail('Disabled camera must not resolve driver dependencies or construct nodes')

    for name in ('FindPackageShare', 'get_package_share_directory', 'ComposableNodeContainer', 'Node'):
        if hasattr(module, name):
            monkeypatch.setattr(module, name, unexpected)
    setup = module.generate_launch_description().entities[-1]
    assert setup.execute(context) == []


@pytest.mark.parametrize('pointcloud', ['false', 'true'])
def test_oak_shared_rate_overrides_rgb_and_depth(pointcloud, monkeypatch):
    module = load('oakd.launch.py')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(ROOT / package))
    context = context_for(module, camera_fps='10', pointcloud=pointcloud)
    container = module.generate_launch_description().entities[-1].execute(context)[0]
    driver = container._ComposableNodeContainer__composable_node_descriptions[0]
    params = driver.parameters[-1]
    flat = {''.join(perform_substitutions(context, [part]) for part in key): value
            for key, value in params.items()}
    assert flat['rgb.i_fps'] == flat['stereo.i_fps'] == 10.0


@pytest.mark.parametrize('camera', ['gemini2.launch.py', 'oakd.launch.py'])
def test_real_camera_rejects_unsupported_rate(camera):
    module = load(camera)
    context = context_for(module, camera_fps='3')
    with pytest.raises(RuntimeError, match='camera_fps'):
        if camera == 'gemini2.launch.py':
            module.launch_setup(context)
        else:
            module.generate_launch_description().entities[-1].execute(context)

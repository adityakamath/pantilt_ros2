"""Physics and portable model checks, runnable without ROS."""
import importlib.util
import math
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import pytest

mujoco = pytest.importorskip('mujoco')
np = pytest.importorskip('numpy')
yaml = pytest.importorskip('yaml')
xacro = pytest.importorskip('xacro')
ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT.parent / 'pt_description'
CONTROL = ROOT.parent / 'pt_control'
sys.path.insert(0, str(ROOT))
from pt_mujoco import build_mujoco_models as builder  # noqa: E402
from pt_mujoco.simulation import Simulation  # noqa: E402

VARIANTS = ('pt100', 'pt101')
JOINTS = ('shoulder_pan_joint', 'tilt_joint')


def snapshot(variant):
    return ROOT / 'mjcf' / f'{variant}_oakd_s2.xml'


def runtime_from_file(path, settle=True):
    runtime = Simulation(mujoco.MjModel.from_xml_path(str(path)))
    runtime.reset(.5 if settle else 0.)
    return runtime


def payload_urdf(variant, description=DESCRIPTION):
    return builder.payload_urdf(variant, {'pt_description': description}, CONTROL)


@pytest.fixture
def workspace(tmp_path, monkeypatch):
    """Isolate source edits so tests exercise regeneration, not committed snapshots."""
    description = tmp_path / 'pt_description'
    for folder in ('urdf', 'meshes'):
        shutil.copytree(DESCRIPTION / folder, description / folder)
    simulation = tmp_path / 'pt_mujoco'
    for folder in ('mjcf', 'config'):
        shutil.copytree(ROOT / folder, simulation / folder)
    control = tmp_path / 'pt_control'
    shutil.copytree(CONTROL / 'config', control / 'config')
    monkeypatch.setattr(builder, 'SIM_PACKAGE', simulation)
    return SimpleWorkspace(description, simulation, control)


class SimpleWorkspace:
    def __init__(self, description, simulation, control):
        self.description, self.simulation, self.control = description, simulation, control

    def build_spec(self, variant='pt101'):
        return builder.build_robot_spec(variant, self.description, control_dir=self.control)


@pytest.mark.parametrize('variant', VARIANTS)
def test_models_settle_and_preserve_interfaces_and_mass(variant):
    runtime = runtime_from_file(snapshot(variant))
    model, data = runtime.model, runtime.data
    assert np.isfinite(data.qpos).all()
    assert model.nu == 2 and model.njnt == 2
    assert np.abs(data.qpos).max() < 1e-3
    assert model.camera('oak_rgb').id >= 0
    masses = {link.get('name'): float(link.find('inertial/mass').get('value'))
              for link in payload_urdf(variant).findall('link') if link.find('inertial/mass') is not None}
    assert model.body_mass.sum() == pytest.approx(sum(masses.values()))


@pytest.mark.parametrize('variant', VARIANTS)
def test_pan_tilt_and_optical_axes(variant):
    runtime = runtime_from_file(snapshot(variant))
    model, data = runtime.model, runtime.data
    camera = model.camera('oak_rgb').id
    assert np.dot(-data.cam_xmat[camera].reshape(3, 3)[:, 2], [1, 0, 0]) > .999
    # The physical camera is mounted upside down: image up is world -Z, right is world +Y.
    assert np.dot(data.cam_xmat[camera].reshape(3, 3)[:, 1], [0, 0, -1]) > .999
    assert np.dot(data.cam_xmat[camera].reshape(3, 3)[:, 0], [0, 1, 0]) > .999
    runtime.command(.6, .3)
    runtime.step(2000)
    for name, goal in zip(JOINTS, (.6, .3)):
        assert abs(data.qpos[model.joint(name).qposadr[0]] - goal) < .03
    assert np.isfinite(data.qpos).all()
    assert np.dot(-data.cam_xmat[camera].reshape(3, 3)[:, 2], [1, 0, 0]) < .95


@pytest.mark.parametrize('variant', VARIANTS)
@pytest.mark.parametrize('joint,target', [(0, .6), (0, -.6), (1, .3), (1, -.3)])
def test_step_response_settles_with_bounded_overshoot(variant, joint, target):
    runtime = runtime_from_file(snapshot(variant))
    goal = [0., 0.]
    goal[joint] = target
    runtime.command(*goal)
    peak = 0.
    for _ in range(1000):
        runtime.step()
        peak = max(peak, runtime.positions()[joint] * math.copysign(1, target))
    assert abs(runtime.positions()[joint] - target) < .01
    # The BAM-identified servo overshoots a little (about 4% pan, 8% tilt); it must stay small.
    assert peak - abs(target) < .10 * abs(target)


@pytest.mark.parametrize('variant', VARIANTS)
def test_generated_assets_are_current_and_portable(tmp_path, variant):
    spec = importlib.util.spec_from_file_location('builder', ROOT / 'pt_mujoco/build_mujoco_models.py')
    fresh = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fresh)
    output = fresh.build(variant, tmp_path / snapshot(variant).name, absolute=True)
    expected = ET.parse(output).getroot()
    actual = ET.parse(snapshot(variant)).getroot()
    for root, directory in [(expected, tmp_path), (actual, ROOT / 'mjcf')]:
        for mesh in root.iter('mesh'):
            mesh.set('file', str((directory / mesh.attrib['file']).resolve()))
        for node in root.iter():
            node.text = (node.text or '').strip()
            node.tail = ''
    assert ET.tostring(expected) == ET.tostring(actual)
    assert not actual.findall('.//include')
    assert not any(Path(mesh.attrib['file']).is_absolute() for mesh in ET.parse(snapshot(variant)).getroot().iter('mesh'))
    mujoco.MjModel.from_xml_path(str(output))


def _urdf_transform(origin):
    """Independent matrix implementation of URDF's fixed-axis RPY convention."""
    transform = np.eye(4)
    if origin is None:
        return transform
    transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    roll, pitch, yaw = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    transform[:3, :3] = rz @ ry @ rx
    return transform


@pytest.mark.parametrize('variant', VARIANTS)
@pytest.mark.parametrize('pan,tilt', [(0., 0.), (.6, -.4), (-.7, .5)])
def test_payload_frames_and_visual_meshes_match_urdf(variant, pan, tilt):
    urdf = payload_urdf(variant)
    transforms = {'base_footprint': np.eye(4)}
    remaining = list(urdf.findall('joint'))
    while remaining:
        progress = False
        for joint in list(remaining):
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                continue
            local = _urdf_transform(joint.find('origin'))
            angle = {'shoulder_pan_joint': pan, 'tilt_joint': tilt}.get(joint.get('name'), 0.)
            if angle:
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                axis /= np.linalg.norm(axis)
                x, y, z = axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                motion = np.eye(4)
                motion[:3, :3] = np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew @ skew)
                local = local @ motion
            transforms[joint.find('child').get('link')] = transforms[parent] @ local
            remaining.remove(joint)
            progress = True
        assert progress, 'Disconnected URDF joint tree'
    runtime = runtime_from_file(snapshot(variant), settle=False)
    model, data = runtime.model, runtime.data
    for name, value in zip(JOINTS, (pan, tilt)):
        data.qpos[model.joint(name).qposadr[0]] = value
    mujoco.mj_forward(model, data)
    base_inverse = np.linalg.inv(transforms['pantilt_base_link'])
    for name in ['pantilt_base_link', 'shoulder_link', 'tilt_link', 'oak_link', 'oak_link_model_origin']:
        expected = base_inverse @ transforms[name]
        body = model.body(name).id
        actual = np.eye(4)
        actual[:3, :3] = data.xmat[body].reshape(3, 3)
        actual[:3, 3] = data.xpos[body] - data.xpos[runtime.payload.base]
        assert np.allclose(actual, expected, atol=1e-8), name
    # Compare compiled mesh bounds with the original STL vertices transformed through the
    # URDF. This catches wrong visual offsets as well as frame-only errors.
    for link_name, mesh_name in [('tilt_link', 'tilt_joint_oakd_s2'), ('oak_link_model_origin', 'oakd_s2')]:
        visual = urdf.find(f"link[@name='{link_name}']/visual")
        transform = transforms[link_name] @ _urdf_transform(visual.find('origin'))
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        records = np.fromfile(DESCRIPTION / 'meshes' / (mesh_name + '.stl'), dtype=dtype, offset=84)
        vertices = records['vertices'].reshape(-1, 3).astype(float)
        expected_vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
        mesh_id = model.mesh(mesh_name).id
        geom_id = next(i for i in range(model.ngeom)
                       if model.geom_dataid[i] == mesh_id and model.geom_group[i] == 2)
        start, count = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
        compiled = model.mesh_vert[start:start+count]
        actual_vertices = compiled @ data.geom_xmat[geom_id].reshape(3, 3).T + data.geom_xpos[geom_id]
        assert np.allclose(actual_vertices.min(axis=0) - data.xpos[runtime.payload.base],
                           expected_vertices.min(axis=0) - transforms['pantilt_base_link'][:3, 3], atol=2e-7)
        assert np.allclose(actual_vertices.max(axis=0) - data.xpos[runtime.payload.base],
                           expected_vertices.max(axis=0) - transforms['pantilt_base_link'][:3, 3], atol=2e-7)


def test_tilt_joint_height_comes_from_the_urdf_not_the_mjcf(workspace):
    urdf_z = float(payload_urdf('pt101').find("joint[@name='tilt_joint']/origin").get('xyz').split()[2])
    assert urdf_z == pytest.approx(0.0541441)
    assert workspace.build_spec().compile().body('tilt_link').pos[2] == pytest.approx(urdf_z)


def test_limits_regenerate_from_edited_configuration_and_urdf(workspace):
    motors = workspace.control / 'config/urdf_config.yaml'
    data = yaml.safe_load(motors.read_text())
    data['sts3215_max_vel_steps'] = 2000
    motors.write_text(yaml.safe_dump(data))
    output = workspace.simulation.parent / 'first.xml'
    builder.build('pt101', output, description_dir=workspace.description, control_dir=workspace.control)
    root = ET.parse(output).getroot()
    expected = int(2000 * .85) * 2 * math.pi / 4096
    for name in JOINTS:
        assert float(root.find(f"custom/numeric[@name='velocity_limit_{name}']").get('data')) == pytest.approx(expected, abs=1e-5)


def test_config_overrides_velocity_and_position_limits(workspace):
    config = workspace.control / 'config/pantilt_config.yaml'
    settings = yaml.safe_load(config.read_text())
    limits = settings['controller_manager']['ros__parameters'].setdefault('joint_limits', {})
    for name, speed, low, high in [('shoulder_pan_joint', .7, -.4, .5), ('tilt_joint', .9, -.6, .3)]:
        limits.setdefault(name, {}).update(has_velocity_limits=True, max_velocity=speed, has_position_limits=True,
                                        min_position=low, max_position=high)
    config.write_text(yaml.safe_dump(settings))
    output = workspace.simulation.parent / 'limits.xml'
    builder.build('pt101', output, description_dir=workspace.description, control_dir=workspace.control)
    root = ET.parse(output).getroot()
    for name, speed, low, high in [('shoulder_pan_joint', .7, -.4, .5), ('tilt_joint', .9, -.6, .3)]:
        assert float(root.find(f"custom/numeric[@name='velocity_limit_{name}']").get('data')) == speed
        assert root.find(f"actuator/general[@name='{name}']").get('ctrlrange') == f'{low} {high}'
        assert root.find(f".//worldbody//joint[@name='{name}']").get('range') == f'{low} {high}'


def test_robot_scene_boundary_and_native_composition(tmp_path):
    robot = builder.build_robot_spec('pt101')
    model = robot.compile()
    assert robot.geom('floor') is None
    assert not list(robot.lights) and not list(robot.textures)
    assert robot.material('groundplane') is None
    world = builder.compose_scene(robot, 'flat')
    combined = world.compile()
    assert world.geom('floor') is not None
    assert combined.ngeom == model.ngeom + 1
    assert combined.nu == model.nu
    assert combined.body_mass.sum() == pytest.approx(model.body_mass.sum())
    # Attachment must serialize to independently loadable XML, including defaults.
    path = tmp_path / 'composed.xml'
    path.write_text(world.to_xml())
    assert mujoco.MjModel.from_xml_path(str(path)).actuator('tilt_joint').id >= 0


def test_custom_scene_resolves_its_own_assets(tmp_path):
    folder = tmp_path / 'scene'
    assets = folder / 'assets'
    assets.mkdir(parents=True)
    (assets / 'tetra.obj').write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n')
    scene = folder / 'world.xml'
    scene.write_text('<mujoco><compiler meshdir="assets"/><asset><mesh name="fixture" file="tetra.obj"/></asset>'
                     '<worldbody><geom name="fixture_geom" type="mesh" mesh="fixture" pos="3 0 0"/></worldbody></mujoco>')
    path = builder.build('pt100', tmp_path / 'different_directory' / 'model.xml', scene=scene)
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.geom('fixture_geom').id >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'floor') == -1


def test_scene_none_builds_the_bare_robot(tmp_path):
    model = mujoco.MjModel.from_xml_path(str(builder.build('pt101', tmp_path / 'bare.xml', scene='none')))
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'floor') == -1
    assert model.nu == 2


def test_payload_mass_changes_and_new_camera_inertia(workspace):
    module = workspace.description / 'urdf/pantilt.module.xacro'
    module.write_text(module.read_text().replace('<mass value="1.0"/>', '<mass value="1.3"/>'))
    module = workspace.description / 'urdf/oakd_s2.module.xacro'
    # Adding missing inertial data later must replace the massless frame automatically.
    module.write_text(module.read_text().replace(
        '<link name="${camera_name}_model_origin">',
        '<link name="${camera_name}_model_origin"><inertial><mass value="0.2"/>'
        '<inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>'))
    model = workspace.build_spec().compile()
    assert model.body('pantilt_base_link').mass[0] == pytest.approx(1.3)
    assert model.body('oak_link_model_origin').mass[0] == pytest.approx(.2)


def test_urdf_inertial_origin_and_tensor_are_preserved(workspace):
    module = workspace.description / 'urdf/pantilt.module.xacro'
    text = module.read_text()
    old = ('<mass value="0.119226"/>\n        <origin xyz="-9.07886e-05 0.0590972 -0.0172669" rpy="0 0 0"/>\n'
           '        <inertia ixx="5.94278e-05" ixy="0" ixz="0" iyy="5.89975e-05" iyz="0" izz="3.13712e-05"/>')
    assert old in text
    module.write_text(text.replace(old, '<mass value="0.119226"/>\n        <origin xyz="0.01 -0.02 0.03" rpy="0 0 0"/>\n'
                                        '        <inertia ixx="0.012" ixy="0.001" ixz="0.002" iyy="0.013" iyz="0.001" izz="0.014"/>'))
    model = workspace.build_spec().compile()
    body = model.body('shoulder_link').id
    assert np.allclose(model.body_ipos[body], [.01, -.02, .03])
    orientation = np.empty(9)
    mujoco.mju_quat2Mat(orientation, model.body_iquat[body])
    orientation = orientation.reshape(3, 3)
    tensor = np.array([[.012, .001, .002], [.001, .013, .001], [.002, .001, .014]])
    assert np.allclose(orientation @ np.diag(model.body_inertia[body]) @ orientation.T, tensor)


def test_profile_and_urdf_effort_own_actuation(workspace):
    module = workspace.description / 'urdf/pantilt.control.xacro'
    module.write_text(module.read_text().replace('value="2.942"', 'value="1.5"'))
    config = workspace.simulation / 'config/mujoco.yaml'
    settings = yaml.safe_load(config.read_text())
    settings['actuators']['pantilt'].update(position_gain=30., velocity_gain=.5, damping=.4, armature=.05, frictionloss=.02)
    config.write_text(yaml.safe_dump(settings))
    model = workspace.build_spec().compile()
    for name in JOINTS:
        actuator = model.actuator(name)
        joint = model.joint(name)
        assert np.allclose(actuator.forcerange, [-1.5, 1.5])
        assert actuator.gainprm[0] == 30. and actuator.biasprm[1] == -30. and actuator.biasprm[2] == -.5
        assert joint.damping[0] == pytest.approx(.4)
        assert joint.armature[0] == pytest.approx(.05) and joint.frictionloss[0] == pytest.approx(.02)


@pytest.mark.parametrize('failure', ['armature', 'effort', 'moving_inertia'])
def test_inconsistent_parameters_fail_generation(workspace, failure):
    if failure == 'armature':
        config = workspace.simulation / 'config/mujoco.yaml'
        settings = yaml.safe_load(config.read_text())
        settings['actuators']['pantilt']['armature'] = -1
        config.write_text(yaml.safe_dump(settings))
    elif failure == 'effort':
        module = workspace.description / 'urdf/pantilt.control.xacro'
        module.write_text(module.read_text().replace('value="2.942"', 'value="0"'))
    else:
        module = workspace.description / 'urdf/pantilt.module.xacro'
        text = module.read_text()
        start = text.index('<link name="tilt_link">')
        end = text.index('</inertial>', start) + len('</inertial>')
        module.write_text(text[:start] + '<link name="tilt_link">' + text[end:])
    with pytest.raises(ValueError):
        workspace.build_spec()


def test_unknown_variant_is_rejected():
    with pytest.raises(ValueError):
        builder.build_robot_spec('pt102')


@pytest.mark.parametrize('variant', VARIANTS)
@pytest.mark.parametrize('joint', JOINTS)
@pytest.mark.parametrize('sign', (1, -1))
def test_instant_jump_to_a_limit_barely_penetrates_the_end_stop(variant, joint, sign):
    """ros2_control's limiter deactivates the controller when the measured position passes the URDF limit."""
    model = mujoco.MjModel.from_xml_path(str(snapshot(variant)))
    data = mujoco.MjData(model)
    limit = model.joint(joint).range[1]
    data.ctrl[model.actuator(joint).id] = sign * limit
    peak = 0.
    for _ in range(2000):
        mujoco.mj_step(model, data)
        peak = max(peak, sign * data.qpos[model.joint(joint).qposadr[0]])
    assert peak - limit < .002, peak - limit


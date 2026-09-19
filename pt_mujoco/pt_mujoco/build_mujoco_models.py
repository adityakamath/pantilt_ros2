#!/usr/bin/env python3
"""Generate portable PT100/PT101 pan-tilt MJCFs from shared xacro and pt_description's URDF.

Also used by ROS launch: --absolute keeps mesh paths valid in temporary files.
Frames, mesh origins, inertias and limits are synced from the URDF, so no offset is copied
into the MJCF by hand.

Standalone, no install needed: `python3 -m pt_mujoco.build_mujoco_models --variant pt101
--output /tmp/pt.xml`, run from this package's root dir (-m puts the cwd on sys.path).
After `pip install -e .` (or a colcon build), the same tool is also `build_mujoco_models`
on PATH / `ros2 run pt_mujoco build_mujoco_models`.
"""
import argparse
from contextlib import contextmanager
import os
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
from pt_mujoco.mujoco_parameters import positive, sync_payload_parameters
import yaml
import xacro
import xacro.substitution_args

from pt_mujoco.paths import package_share

VARIANTS = ('pt100', 'pt101')
SIM_PACKAGE = package_share('pt_mujoco')


@contextmanager
def package_paths(packages):
    """Resolve the known packages without requiring a ROS Python installation."""
    packages = {'pt_mujoco': SIM_PACKAGE, **packages}
    original = xacro.substitution_args._eval_find
    xacro.substitution_args._eval_find = lambda name: str(packages[name]) if name in packages else original(name)
    try:
        yield
    finally:
        xacro.substitution_args._eval_find = original


def set_origin(element, origin):
    """Translate URDF fixed-axis Rz(yaw) Ry(pitch) Rx(roll) to MJCF quaternion."""
    xyz = origin.get('xyz', '0 0 0') if origin is not None else '0 0 0'
    rpy = origin.get('rpy', '0 0 0') if origin is not None else '0 0 0'
    r, p, y = [float(value) / 2 for value in rpy.split()]
    cr, cp, cy = math.cos(r), math.cos(p), math.cos(y)
    sr, sp, sy = math.sin(r), math.sin(p), math.sin(y)
    quat = (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)
    element.pos = list(map(float, xyz.split()))
    element.alt.type = mujoco.mjtOrientation.mjORIENTATION_QUAT
    element.quat = quat


def control_package():
    # Optional checkout/share discovery; installed ROS launch passes this explicitly.
    return package_share('pt_control')


def payload_urdf(variant, packages, control):
    """Expand pt_description's standalone URDF the way the real robot does, in mock mode."""
    motor = yaml.safe_load((control / 'config/urdf_config.yaml').read_text())
    with package_paths(packages):
        doc = xacro.process_file(str(packages['pt_description'] / 'urdf/pantilt.urdf.xacro'), mappings={
            **{key: str(value).lower() if isinstance(value, bool) else str(value) for key, value in motor.items()},
            'pantilt_config': variant, 'use_mock': 'true'})
    return ET.fromstring(doc.toxml())


def sync_payload_geometry(spec, urdf):
    """Use the URDF as the single source for payload frames, joint axes and mesh origins."""
    joints = [('shoulder_pan_joint', 'shoulder_link'), ('tilt_joint', 'tilt_link'),
              ('oak_link_center_joint', 'oak_link'),
              ('oak_link_model_origin_joint', 'oak_link_model_origin')]
    for joint_name, body_name in joints:
        joint = urdf.find(f"joint[@name='{joint_name}']")
        body = spec.body(body_name)
        if joint is None or body is None:
            raise ValueError(f'Missing payload frame: {joint_name} / {body_name}')
        set_origin(body, joint.find('origin'))
        mj_joint = spec.joint(joint_name)
        if mj_joint is not None:
            mj_joint.axis = list(map(float, joint.find('axis').get('xyz').split()))
            limit = joint.find('limit')
            mj_joint.range = [float(limit.get('lower')), float(limit.get('upper'))]
    meshes = {mesh.name: Path(mesh.file).name for mesh in spec.meshes if mesh.file}
    for name in ('pantilt_base_link', 'shoulder_link', 'tilt_link', 'oak_link_model_origin'):
        body = spec.body(name)
        link = urdf.find(f"link[@name='{name}']")
        for geom in body.geoms:
            filename = meshes.get(geom.meshname)
            role = 'collision' if geom.classname.name == 'collision' else 'visual'
            matches = [entry for entry in link.findall(role)
                       if entry.find('geometry/mesh') is not None
                       and Path(entry.find('geometry/mesh').get('filename')).name == filename]
            if len(matches) != 1:
                raise ValueError(f'Ambiguous or missing URDF {role}: {name}/{filename}')
            set_origin(geom, matches[0].find('origin'))
            spec.mesh(geom.meshname).scale = list(map(float, matches[0].find('geometry/mesh').get('scale', '1 1 1').split()))


def sync_velocity_limits(spec, urdf, control):
    """Embed the configured joint limits so the viewer needs only the generated XML."""
    settings = yaml.safe_load((control / 'config/pantilt_config.yaml').read_text())
    payload_limits = settings['controller_manager']['ros__parameters'].get('joint_limits', {})
    for actuator in spec.actuators:
        name = actuator.target
        # Real-mode URDF <limit velocity> is deliberately 1e6; use the
        # hardware's actual max_velocity parameter, not that sentinel.
        param = urdf.find(f".//ros2_control/joint[@name='{name}']/param[@name='max_velocity']")
        if param is None:
            raise ValueError(f'Missing hardware velocity limit for {name}')
        joint_limit = urdf.find(f"joint[@name='{name}']/limit")
        limit = min(float(param.text), float(joint_limit.get('velocity')))
        configured = payload_limits.get(name, {})
        if configured.get('has_velocity_limits', False):
            limit = min(limit, float(configured['max_velocity']))
        hardware_joint = urdf.find(f".//ros2_control/joint[@name='{name}']")
        low = max(float(joint_limit.get('lower')),
                  float(hardware_joint.find("param[@name='min_position']").text))
        high = min(float(joint_limit.get('upper')),
                   float(hardware_joint.find("param[@name='max_position']").text))
        if configured.get('has_position_limits', False):
            low = max(low, float(configured['min_position']))
            high = min(high, float(configured['max_position']))
        if not math.isfinite(low) or not math.isfinite(high) or low >= high:
            raise ValueError(f'Invalid position limits for {name}: {low}, {high}')
        spec.joint(name).range = [low, high]
        actuator.inheritrange = 0
        actuator.ctrllimited = True
        actuator.ctrlrange = [low, high]
        if not math.isfinite(limit) or limit <= 0:
            raise ValueError(f'Invalid command limits: velocity_limit_{name}')
        spec.add_numeric(name='velocity_limit_' + name, data=[limit])


def build_robot_spec(variant, description_dir=None, *, control_dir=None):
    """Return an editable native spec with absolute assets for future scene composition."""
    if variant not in VARIANTS:
        raise ValueError(f'Unknown variant: {variant}')
    control = Path(control_dir).resolve() if control_dir else control_package()
    packages = {'pt_description': Path(description_dir).resolve() if description_dir else package_share('pt_description')}
    with package_paths(packages):
        doc = xacro.process_file(str(SIM_PACKAGE / 'mjcf/pt.mjcf.xacro'),
                                 mappings={'pantilt_config': variant})
    # MuJoCo parses includes and maintains model references; no custom XML assembly.
    spec = mujoco.MjSpec.from_string(doc.toxml())
    urdf = payload_urdf(variant, packages, control)
    sync_payload_geometry(spec, urdf)
    sync_velocity_limits(spec, urdf, control)
    sync_payload_parameters(spec, urdf, yaml.safe_load((SIM_PACKAGE / 'config/mujoco.yaml').read_text()), set_origin)
    camera = spec.camera('oak_rgb')
    if camera is not None:
        # The OAK-D is mounted upside down: the camera axes follow the oak_link frame (right = its
        # -Y, up = its +Z), which is rolled 180 degrees, so the image is upside down like the real one.
        camera.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
        camera.alt.xyaxes = [0, -1, 0, 0, 0, 1]
    configure_physics(spec)
    return spec


def configure_physics(spec):
    settings = yaml.safe_load((SIM_PACKAGE / 'config/mujoco.yaml').read_text())['physics']
    spec.option.timestep = positive(settings['timestep'], 'physics timestep')
    iterations = positive(settings['iterations'], 'solver iterations')
    if int(iterations) != iterations:
        raise ValueError('solver iterations must be an integer')
    spec.option.iterations = int(iterations)
    integrators = {'implicitfast': mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
                   'implicit': mujoco.mjtIntegrator.mjINT_IMPLICIT,
                   'Euler': mujoco.mjtIntegrator.mjINT_EULER, 'RK4': mujoco.mjtIntegrator.mjINT_RK4}
    spec.option.integrator = integrators[settings['integrator']]


def resolve_assets(spec):
    """Resolve external scene assets before native attachment changes their context."""
    for assets, directory in ((spec.meshes, spec.meshdir), (spec.textures, spec.texturedir)):
        root = Path(spec.modelfiledir or '.') / directory
        for asset in assets:
            if asset.file:
                asset.file = str((root / asset.file).resolve())
    for texture in spec.textures:
        texture.cubefiles = [str((Path(spec.modelfiledir or '.') / spec.texturedir / name).resolve())
                             if name else '' for name in texture.cubefiles]
    spec.meshdir = ''
    spec.texturedir = ''


def compose_scene(robot, scene='flat'):
    """One payload in a separate scene. Native attachment owns references and assets.

    The simulation profile owns global physics options; scenes own world geometry,
    visual settings and lighting.
    """
    if scene is False or scene is None or scene == 'none':
        return robot
    if scene is True:
        scene = 'flat'
    bundled = SIM_PACKAGE / 'mjcf/scenes' / f'{scene}.xml'
    path = bundled if bundled.is_file() else Path(scene).expanduser().resolve()
    world = mujoco.MjSpec.from_file(str(path))
    resolve_assets(world)
    configure_physics(world)
    # A named root default stays valid when serialized below the scene's default.
    robot.default.name = 'pt_robot'
    world.attach(robot, prefix='', frame=world.worldbody.add_frame(name='pt_spawn'))
    return world


def build_spec(variant, description_dir=None, scene='flat', *, control_dir=None):
    return compose_scene(build_robot_spec(variant, description_dir, control_dir=control_dir), scene)


def build(variant, output, absolute=False, description_dir=None, scene=True, *, control_dir=None):
    output = Path(output).resolve()
    spec = build_spec(variant, description_dir=description_dir, scene=scene, control_dir=control_dir)
    for mesh in [*spec.meshes, *spec.textures]:
        if mesh.file:
            path = Path(mesh.file).resolve()
            mesh.file = str(path) if absolute else os.path.relpath(path, output.parent).replace(os.sep, '/')
            # Let MuJoCo compile portable paths without changing process cwd.
            if not absolute:
                spec.assets[mesh.file] = path.read_bytes()
    for texture in spec.textures:
        files = []
        for name in texture.cubefiles:
            if name and not absolute:
                path = Path(name).resolve()
                name = os.path.relpath(path, output.parent).replace(os.sep, '/')
                spec.assets[name] = path.read_bytes()
            files.append(name)
        texture.cubefiles = files
    output.parent.mkdir(parents=True, exist_ok=True)
    spec.compile()
    content = '<!-- Generated by pt_mujoco.build_mujoco_models using MjSpec; edit sources, not this file. -->\n'
    output.write_text(content + spec.to_xml())
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=VARIANTS)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--absolute', action='store_true')
    parser.add_argument('--control-package', type=Path, help='Directory containing control config/ (data only)')
    parser.add_argument('--description-package', type=Path, help='pt_description source/share directory')
    parser.add_argument('--scene', default='flat', help='flat, none, or a scene MJCF path')
    args = parser.parse_args()
    if bool(args.variant) != bool(args.output):
        parser.error('--variant and --output must be used together')
    if args.variant:
        build(args.variant, args.output, args.absolute, args.description_package, scene=args.scene, control_dir=args.control_package)
    else:
        for variant in VARIANTS:
            print(build(variant, SIM_PACKAGE / 'mjcf' / f'{variant}_oakd_s2.xml', args.absolute, args.description_package,
                        scene=args.scene, control_dir=args.control_package))


if __name__ == '__main__':
    main()

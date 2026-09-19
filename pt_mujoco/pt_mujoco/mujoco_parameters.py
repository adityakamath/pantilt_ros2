"""Pan-tilt parameter ownership and URDF-to-MjSpec physical conversion."""
from types import SimpleNamespace

import mujoco
import numpy as np


def positive(value, name, allow_zero=False):
    value = float(value)
    if not np.isfinite(value) or value < 0 or (value == 0 and not allow_zero):
        raise ValueError(f'{name} must be finite and {"nonnegative" if allow_zero else "positive"}')
    return value


def pose(origin, set_origin):
    target = SimpleNamespace(alt=SimpleNamespace())
    set_origin(target, origin)
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, np.array(target.quat))
    return np.array(target.pos), rotation.reshape(3, 3)


def set_inertia(body, mass, center, inertia):
    """Explicit principal inertia; reject invalid sources instead of silently repairing."""
    positive(mass, f'{body.name} mass')
    values, axes = np.linalg.eigh(inertia)
    if not np.isfinite(values).all() or values.min() <= 0 or values.max() > values.sum() - values.max() + 1e-12:
        raise ValueError(f'Nonphysical inertia for {body.name}; check the URDF')
    if np.linalg.det(axes) < 0:
        axes[:, 0] *= -1
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, axes.ravel())
    body.explicitinertial = True
    body.mass, body.ipos, body.iquat, body.inertia = mass, center, quaternion, values


def sync_payload_parameters(spec, urdf, simulation, set_origin):
    """URDF owns inertias and effort limits; YAML owns the servo approximation."""
    # No mass is inferred from CAD volume or retained from the MJCF. Frames without URDF
    # inertia stay massless; a moving link without one is an error.
    for body in spec.bodies:
        if body.name == 'world':
            continue
        link = urdf.find(f"link[@name='{body.name}']")
        element = link.find('inertial') if link is not None else None
        for geom in body.geoms:
            geom.mass, geom.density = 0, 0
        if element is None:
            if list(body.joints):
                raise ValueError(f'Missing URDF inertia for moving body {body.name}')
            body.explicitinertial = True
            body.mass, body.ipos, body.inertia = 0, [0, 0, 0], [0, 0, 0]
    for link in urdf.findall('link'):
        element = link.find('inertial')
        if element is None:
            continue
        body = spec.body(link.get('name'))
        if body is None:
            raise ValueError(f"URDF inertia has no MuJoCo body: {link.get('name')}")
        mass = positive(element.find('mass').get('value'), link.get('name') + ' mass')
        center, rotation = pose(element.find('origin'), set_origin)
        raw = element.find('inertia')
        xx, yy, zz, xy, xz, yz = [float(raw.get(k)) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')]
        inertia = rotation @ np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]) @ rotation.T
        set_inertia(body, mass, center, inertia)
    settings = simulation['actuators']['pantilt']
    for actuator in spec.actuators:
        name = actuator.target
        joint = spec.joint(name)
        joint.armature = positive(settings['armature'], name + ' armature', True)
        joint.frictionloss = positive(settings['frictionloss'], name + ' frictionloss', True)
        effort = positive(urdf.find(f"joint[@name='{name}']/limit").get('effort'), name + ' effort')
        actuator.forcelimited = True
        actuator.forcerange = [-effort, effort]
        gain = positive(settings['position_gain'], 'position_gain')
        actuator.gainprm[0], actuator.biasprm[1] = gain, -gain
        # MjSpec position shortcut stores a positive bias[2] as dampratio
        # until compilation resolves it against the effective joint inertia.
        actuator.biasprm[2] = positive(settings['damping_ratio'], 'damping_ratio')

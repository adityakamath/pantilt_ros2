"""ROS/GUI-independent commands, stepping, reset and state for the pan-tilt payload."""
from types import MappingProxyType

import mujoco
import numpy as np

PAYLOAD = ('shoulder_pan_joint', 'tilt_joint')


def finite_vector(value, size, label):
    value = np.asarray(value, dtype=float)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError(f'{label} must contain {size} finite values')
    return value


class PayloadBindings:
    """Resolve the payload's named elements once against a compiled model."""
    def __init__(self, model):
        self.model = model

        def required(kind, name):
            result = mujoco.mj_name2id(model, kind, name)
            if result < 0:
                raise ValueError(f'Missing payload element: {name}')
            return result

        self.base = required(mujoco.mjtObj.mjOBJ_BODY, 'pantilt_base_link')
        self.actuator_ids = MappingProxyType({name: required(mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in PAYLOAD})
        self.joint_ids = MappingProxyType({name: required(mujoco.mjtObj.mjOBJ_JOINT, name) for name in PAYLOAD})
        for name in PAYLOAD:
            if model.actuator_trnid[self.actuator_ids[name], 0] != self.joint_ids[name]:
                raise ValueError(f'Actuator/joint mismatch: {name}')
        self.sensor_ids = self._named_ids(mujoco.mjtObj.mjOBJ_SENSOR, model.nsensor)
        self.camera_ids = self._named_ids(mujoco.mjtObj.mjOBJ_CAMERA, model.ncam)

    def _named_ids(self, kind, count):
        return MappingProxyType({name: i for i in range(count)
                                 if (name := mujoco.mj_id2name(self.model, kind, i))})

    def numeric(self, name):
        result = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_NUMERIC, name)
        if result < 0:
            raise ValueError(f'Missing payload metadata: {name}; regenerate the model')
        return self.model.numeric(result).data.copy()


class PayloadControl:
    """Common command semantics. Limits constrain commands, not physical state."""
    def __init__(self, model, bindings=None):
        self.model = model
        self.bindings = bindings or PayloadBindings(model)
        self.payload_limits = {name: float(self.bindings.numeric('velocity_limit_' + name)[0]) for name in PAYLOAD}
        if any(not np.isfinite(v) or v <= 0 for v in self.payload_limits.values()):
            raise ValueError('Payload velocity limits must be positive and finite')
        self.previous_targets = {name: float(np.clip(0, *model.actuator_ctrlrange[self.bindings.actuator_ids[name]]))
                                 for name in PAYLOAD}

    def action(self, value):
        return np.clip(finite_vector(value, len(PAYLOAD), 'action'), -1, 1)

    def integrate_payload(self, data, rates, dt):
        """Form requested targets from normalized rates; physics enforces them separately."""
        if not np.isfinite(dt) or dt < 0:
            raise ValueError('Target interval must be finite and nonnegative')
        rates = finite_vector(rates, len(PAYLOAD), 'payload rates')
        for rate, (name, limit) in zip(np.clip(rates, -1, 1), self.payload_limits.items()):
            actuator = self.bindings.actuator_ids[name]
            data.ctrl[actuator] = np.clip(data.ctrl[actuator] + rate * limit * dt,
                                          *self.model.actuator_ctrlrange[actuator])

    def apply(self, data, requested, dt):
        if not np.isfinite(dt) or dt < 0:
            raise ValueError('Control interval must be finite and nonnegative')
        requested = finite_vector(requested, self.model.nu, 'actuator commands')
        for name, limit in self.payload_limits.items():
            actuator = self.bindings.actuator_ids[name]
            low, high = self.model.actuator_ctrlrange[actuator]
            previous = self.previous_targets[name]
            target = np.clip(requested[actuator], max(low, previous-limit*dt), min(high, previous+limit*dt))
            data.ctrl[actuator] = target
            self.previous_targets[name] = float(target)

    def reset_targets(self, data):
        self.previous_targets = {name: float(np.clip(data.ctrl[self.bindings.actuator_ids[name]],
                                                    *self.model.actuator_ctrlrange[self.bindings.actuator_ids[name]]))
                                 for name in PAYLOAD}
        for name, value in self.previous_targets.items():
            data.ctrl[self.bindings.actuator_ids[name]] = value


class Simulation:
    """Model/data owner (no rendering or ROS); step(count, action) holds a normalized [pan, tilt] rate, step(count) uses data.ctrl."""
    def __init__(self, model):
        self.model = model
        self.data = mujoco.MjData(model)
        self.payload = PayloadBindings(model)
        self.control = PayloadControl(model, self.payload)
        self.ready = False
        self._requested = self.data.ctrl.copy()
        self._applied = self.data.ctrl.copy()

    def steps_for(self, seconds):
        if not np.isfinite(seconds) or seconds <= 0:
            raise ValueError('Control interval must be finite and positive')
        ratio = seconds / self.model.opt.timestep
        if abs(ratio - round(ratio)) > 1e-8 or round(ratio) < 1:
            raise ValueError('Control interval must be an integer multiple of the physics timestep')
        return round(ratio)

    def reset(self, settle_seconds=.5):
        if not np.isfinite(settle_seconds) or settle_seconds < 0:
            raise ValueError('Settling duration must be finite and nonnegative')
        mujoco.mj_resetData(self.model, self.data)
        self.control.reset_targets(self.data)
        self._requested = self.data.ctrl.copy()
        self._applied = self.data.ctrl.copy()
        self.ready = True
        # Settling uses the same bounded controls and stepping as normal execution.
        self.step(round(settle_seconds / self.model.opt.timestep))
        self.data.time = 0.
        mujoco.mj_forward(self.model, self.data)
        return self.observation()

    def command(self, pan, tilt):
        """Request absolute joint targets in radians; they are slew-limited and clipped to range."""
        for name, target in zip(PAYLOAD, finite_vector([pan, tilt], len(PAYLOAD), 'joint targets')):
            self.data.ctrl[self.payload.actuator_ids[name]] = target
        self._requested[:] = self.data.ctrl

    def stop(self):
        """Hold the current payload targets; do not teleport state."""
        self._requested = self.data.ctrl.copy()
        self._applied = self.data.ctrl.copy()
        self.control.reset_targets(self.data)

    def step(self, count=1, action=None):
        if not self.ready:
            raise RuntimeError('Call reset() before stepping')
        if not isinstance(count, (int, np.integer)) or isinstance(count, bool) or count < 0:
            raise ValueError('Physics step count must be a nonnegative integer')
        action = self.control.action(action) if action is not None else None
        if not np.isfinite(self.data.ctrl).all():
            raise ValueError('Actuator commands must be finite')
        changed = self.data.ctrl != self._applied
        self._requested[changed] = self.data.ctrl[changed]
        for _ in range(count):
            if action is not None:
                self.control.integrate_payload(self.data, action, self.model.opt.timestep)
                self._requested = self.data.ctrl.copy()
            self.control.apply(self.data, self._requested, self.model.opt.timestep)
            mujoco.mj_step(self.model, self.data)
        self._applied = self.data.ctrl.copy()
        mujoco.mj_forward(self.model, self.data)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all() or not np.isfinite(self.data.sensordata).all():
            raise FloatingPointError('Non-finite MuJoCo state')

    def positions(self):
        """Measured joint angles in radians as [pan, tilt]."""
        return np.array([self.data.qpos[self.model.joint(name).qposadr[0]] for name in PAYLOAD])

    def observation(self):
        return {'qpos': self.data.qpos.copy(), 'qvel': self.data.qvel.copy(),
                'sensors': self.data.sensordata.copy()}

    def info(self):
        return {'sim_time': float(self.data.time), 'actuator_commands': self.data.ctrl.copy(),
                'contacts': int(self.data.ncon), 'warning_count': sum(int(w.number) for w in self.data.warning)}

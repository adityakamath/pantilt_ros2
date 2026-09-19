"""Held-key motion, release, focus and limit checks without a GUI."""
from pathlib import Path
import sys

import pytest

mujoco = pytest.importorskip('mujoco')
np = pytest.importorskip('numpy')
glfw = pytest.importorskip('glfw')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pt_mujoco.mujoco_preview import HeldKeys, KeyboardControl  # noqa: E402


@pytest.fixture(scope='module')
def model():
    return mujoco.MjModel.from_xml_path(str(ROOT / 'mjcf/pt101_oakd_s2.xml'))


@pytest.mark.parametrize('key,name,sign', [(glfw.KEY_LEFT, 'shoulder_pan_joint', 1),
    (glfw.KEY_RIGHT, 'shoulder_pan_joint', -1), (glfw.KEY_DOWN, 'tilt_joint', -1),
    (glfw.KEY_UP, 'tilt_joint', 1)])
def test_continuous_motion_release_and_limits(model, key, name, sign):
    data = mujoco.MjData(model)
    control = KeyboardControl(model)
    actuator = model.actuator(name).id
    for _ in range(6):
        control.update(data, {key}, 1 / 60)
    assert data.ctrl[actuator] == pytest.approx(sign * control.payload_limits[name] * .1)
    control.update(data, set(), 1.)
    assert data.ctrl[actuator] == pytest.approx(sign * control.payload_limits[name] * .1)
    for _ in range(600):
        control.update(data, {key}, 1 / 60)
    low, high = model.actuator_ctrlrange[actuator]
    assert data.ctrl[actuator] == pytest.approx(high if sign > 0 else low)


def test_opposite_keys_cancel_and_axes_are_independent(model):
    data = mujoco.MjData(model)
    control = KeyboardControl(model)
    control.update(data, {glfw.KEY_LEFT, glfw.KEY_RIGHT, glfw.KEY_UP}, .01)
    assert data.ctrl[model.actuator('shoulder_pan_joint').id] == 0
    assert data.ctrl[model.actuator('tilt_joint').id] == pytest.approx(control.payload_limits['tilt_joint'] * .01)


def test_focus_loss_clears_held_keys_and_other_events_forward():
    keys = HeldKeys()
    forwarded = []
    keys.previous_key = lambda *args: forwarded.append(args)
    keys.record(glfw.KEY_UP, glfw.PRESS)
    keys.on_key(None, ord('W'), 0, glfw.PRESS, 0)
    assert forwarded[0][1] == ord('W')
    assert keys.snapshot() == {glfw.KEY_UP}
    keys.on_focus(None, False)
    assert not keys.snapshot()
    assert keys.events.get() == 'focus_lost'


def test_reset_pause_are_edges_not_repeat():
    keys = HeldKeys()
    for key in [ord('X'), ord('P')]:
        for action in [glfw.PRESS, glfw.REPEAT, glfw.RELEASE]:
            keys.on_key(None, key, 0, action, 0)
        assert keys.events.get_nowait() == key
        assert keys.events.empty()


def test_bootstrap_installs_release_and_focus_callbacks(monkeypatch):
    keys, window = HeldKeys(), object()
    installed = {}
    monkeypatch.setattr(glfw, 'get_current_context', lambda: window)
    monkeypatch.setattr(glfw, 'get_key', lambda w, k: glfw.PRESS if k == glfw.KEY_UP else glfw.RELEASE)
    monkeypatch.setattr(keys, 'install_callbacks',
                        lambda w: installed.update(key=keys.on_key, focus=keys.on_focus))
    keys.bootstrap(glfw.KEY_UP)
    assert keys.snapshot() == {glfw.KEY_UP}
    installed['key'](window, glfw.KEY_UP, 0, glfw.RELEASE, 0)
    assert not keys.snapshot()


def test_slider_commands_slew_at_the_payload_limit(model):
    control = KeyboardControl(model)
    data = mujoco.MjData(model)
    tilt = model.actuator('tilt_joint').id
    data.ctrl[tilt] = 1.5
    previous = 0.
    for _ in range(20):
        control.control.apply(data, data.ctrl.copy(), .002)
        assert abs(data.ctrl[tilt] - previous) <= control.payload_limits['tilt_joint'] * .002 + 1e-10
        previous = data.ctrl[tilt]
        data.ctrl[tilt] = 1.5


def test_motor_limit_is_baked_into_xml(model):
    expected = 2890 * 2 * np.pi / 4096
    for name in ('shoulder_pan_joint', 'tilt_joint'):
        assert model.numeric('velocity_limit_' + name).data[0] == pytest.approx(expected, abs=1e-5)

"""Command semantics, stepping and reset checks."""
from pathlib import Path
import sys

import pytest

mujoco = pytest.importorskip('mujoco')
np = pytest.importorskip('numpy')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pt_mujoco.simulation import Simulation  # noqa: E402


@pytest.fixture(scope='module')
def model():
    return mujoco.MjModel.from_xml_path(str(ROOT / 'mjcf/pt101_oakd_s2.xml'))


def same(a, b):
    for key in ('qpos', 'qvel', 'ctrl', 'sensordata'):
        np.testing.assert_allclose(getattr(a, key), getattr(b, key), atol=1e-12, rtol=0)
    assert a.time == pytest.approx(b.time)


def test_batching_reset_and_slider_goal(model):
    a, b = Simulation(model), Simulation(model)
    a.reset()
    b.reset()
    action = [1, -1]
    a.step(20, action)
    for _ in range(20):
        b.step(action=action)
    same(a.data, b.data)
    pan = a.payload.actuator_ids['shoulder_pan_joint']
    a.data.ctrl[pan] = .8
    start = a.control.previous_targets['shoulder_pan_joint']
    a.step(3)
    assert a.data.ctrl[pan] == pytest.approx(start + 3 * model.opt.timestep * a.control.payload_limits['shoulder_pan_joint'])
    a.stop()
    held = a.data.ctrl[pan]
    a.step(4)
    assert a.data.ctrl[pan] == held
    a.reset()
    b.reset()
    same(a.data, b.data)
    a.step(20, action)
    b.step(20, action)
    same(a.data, b.data)


def test_command_moves_to_the_requested_angles_and_clips_to_range(model):
    sim = Simulation(model)
    sim.reset()
    sim.command(.5, -.4)
    sim.step(1000)
    assert sim.positions() == pytest.approx([.5, -.4], abs=.01)
    sim.command(5., -5.)
    sim.step(2000)
    low, high = model.actuator_ctrlrange[sim.payload.actuator_ids['shoulder_pan_joint']]
    assert sim.positions()[0] == pytest.approx(high, abs=.01)
    assert sim.positions()[1] == pytest.approx(model.actuator_ctrlrange[sim.payload.actuator_ids['tilt_joint']][0], abs=.01)


def test_slew_limit_bounds_the_target_rate(model):
    sim = Simulation(model)
    sim.reset()
    sim.command(1.5, 1.5)
    limit = sim.control.payload_limits['tilt_joint']
    previous = 0.
    for _ in range(50):
        sim.step()
        target = sim.data.ctrl[sim.payload.actuator_ids['tilt_joint']]
        assert target - previous <= limit * model.opt.timestep + 1e-10
        previous = target


def test_invalid_execution_inputs(model):
    sim = Simulation(model)
    with pytest.raises(RuntimeError):
        sim.step()
    sim.reset(0)
    for count in (-1, .5, True):
        with pytest.raises(ValueError):
            sim.step(count)
    for interval in (0, np.nan, .003):
        with pytest.raises(ValueError):
            sim.steps_for(interval)
    with pytest.raises(ValueError):
        sim.step(action=[0, np.nan])
    with pytest.raises(ValueError):
        sim.step(action=[0, 0, 0])
    with pytest.raises(ValueError):
        sim.command(np.nan, 0)


def test_models_without_payload_actuators_are_rejected():
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body name="pantilt_base_link"><geom size=".1"/></body></worldbody></mujoco>')
    with pytest.raises(ValueError):
        Simulation(model)

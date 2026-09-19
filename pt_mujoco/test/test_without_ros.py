"""Exercise standalone entry points with ROS Python packages blocked."""
import os
from pathlib import Path
import subprocess
import sys

import pytest


def test_without_ros(tmp_path):
    package = Path(__file__).resolve().parents[1]
    environment = {key: os.environ[key] for key in ('HOME', 'PATH', 'TMPDIR', 'SYSTEMROOT') if key in os.environ}
    # Isolated mode ignores PYTHONPATH, so a mujoco/xacro that comes only from a ROS or user
    # install is not visible; that needs pip-installed requirements (as in CI), not a failure.
    probe = subprocess.run([sys.executable, '-I', '-c', 'import mujoco, xacro'], env=environment, capture_output=True)
    if probe.returncode:
        pytest.skip('mujoco and xacro are not pip-installed for isolated Python (see requirements.txt)')
    program = '''
import importlib.abc
import sys
from pathlib import Path

class NoROS(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'rclpy', 'rclcpp', 'ament_index_python',
                                      'launch', 'launch_ros', 'rosbag2_py'}:
            raise AssertionError('Unexpected ROS import: ' + fullname)
sys.meta_path.insert(0, NoROS())
sys.path.insert(0, sys.argv[1])
import mujoco
from pt_mujoco.build_mujoco_models import build
from pt_mujoco.simulation import Simulation
for variant in ('pt100', 'pt101'):
    path = build(variant, Path(sys.argv[2]) / (variant + '.xml'), absolute=True)
    sim = Simulation(mujoco.MjModel.from_xml_path(str(path)))
    sim.reset()
    sim.command(.3, -.2)
    sim.step(200)
    assert sim.data.time > 0
    sim.stop()
    sim.reset()
    assert sim.data.time == 0
print('Both variants generated, stepped, stopped and reset without ROS')
'''
    result = subprocess.run([sys.executable, '-I', '-c', program, str(package), str(tmp_path)],
                            cwd=tmp_path, env=environment, capture_output=True, text=True, timeout=90)
    assert result.returncode == 0, result.stdout + result.stderr

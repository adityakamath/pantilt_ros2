"""Drive the viewer's main loop with a fake passive viewer, so no display is needed."""
from contextlib import contextmanager
from pathlib import Path
import sys

import pytest

mujoco = pytest.importorskip('mujoco')
pytest.importorskip('glfw')
np = pytest.importorskip('numpy')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pt_mujoco import mujoco_preview  # noqa: E402


class FakeViewer:
    def __init__(self, frames):
        self.frames = frames
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        self.texts = []
        self.synced = 0

    @contextmanager
    def lock(self):
        yield

    def is_running(self):
        self.frames -= 1
        return self.frames >= 0

    def set_texts(self, texts):
        self.texts = texts

    def sync(self):
        self.synced += 1

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


def test_main_loop_steps_the_model_and_reports_state(monkeypatch, capsys):
    viewer = FakeViewer(frames=5)
    captured = {}

    def launch_passive(model, data, key_callback=None):
        captured['data'] = data
        captured['callback'] = key_callback
        return viewer

    monkeypatch.setattr(mujoco.viewer, 'launch_passive', launch_passive)
    monkeypatch.setattr(mujoco_preview.time, 'sleep', lambda seconds: None)
    monkeypatch.setattr(sys, 'argv', ['mujoco_preview', '--model', str(ROOT / 'mjcf/pt100_oakd_s2.xml')])
    mujoco_preview.main()
    assert viewer.synced == 5
    assert captured['data'].time > 0
    assert callable(captured['callback'])
    text = viewer.texts[0][2]
    assert 'pt100_oakd_s2' in text and 'pan' in text and 'RUNNING' in text
    assert 'Native viewer ready' in capsys.readouterr().out
    assert not viewer.opt.geomgroup[3]

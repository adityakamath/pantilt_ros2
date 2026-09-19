"""The MJCF xacro sources expand to valid XML with meshes that exist, without ROS discovery."""
from contextlib import contextmanager
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import xacro
import xacro.substitution_args

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pt_mujoco.paths import package_share  # noqa: E402

CONFIGS = ('pt100', 'pt101')


@contextmanager
def package_paths():
    packages = {'pt_mujoco': ROOT, 'pt_description': package_share('pt_description')}
    original = xacro.substitution_args._eval_find
    xacro.substitution_args._eval_find = lambda name: str(packages[name]) if name in packages else original(name)
    try:
        yield
    finally:
        xacro.substitution_args._eval_find = original


def process(config, standalone):
    with package_paths():
        doc = xacro.process_file(str(ROOT / 'mjcf/pantilt.mjcf.xacro'),
                                 mappings={'pantilt_config': config, 'standalone': standalone})
    return ET.fromstring(doc.toxml())


@pytest.mark.parametrize('config', CONFIGS)
@pytest.mark.parametrize('standalone', ('true', 'false'))
def test_mjcf_processes_to_valid_xml(config, standalone):
    root = process(config, standalone)
    assert root.tag == 'mujoco'
    assert root.find('worldbody') is not None or standalone == 'false'


@pytest.mark.parametrize('config', CONFIGS)
def test_mjcf_referenced_meshes_exist_on_disk(config):
    meshes = process(config, 'true').findall('asset/mesh')
    assert meshes, f'{config}: no <mesh> assets found in MJCF'
    for mesh in meshes:
        assert mesh.get('file') and Path(mesh.get('file')).is_file(), \
            f'{config}: missing MJCF mesh {mesh.get("file")}'

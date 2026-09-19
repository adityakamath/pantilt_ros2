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


def process(config, entry='pt.mjcf.xacro'):
    with package_paths():
        doc = xacro.process_file(str(ROOT / 'mjcf' / entry), mappings={'pantilt_config': config})
    return ET.fromstring(doc.toxml())


@pytest.mark.parametrize('config', CONFIGS)
def test_entry_file_fixes_the_payload_to_the_world(config):
    root = process(config)
    assert root.tag == 'mujoco'
    bodies = root.findall('worldbody/body')
    assert [body.get('name') for body in bodies] == ['pantilt_base_link']
    assert root.find('worldbody/geom') is None and not root.findall('asset/texture')


@pytest.mark.parametrize('config', CONFIGS)
def test_library_alone_adds_no_worldbody_or_scene(config):
    root = process(config, 'pantilt.mjcf.xacro')
    assert root.find('worldbody') is None
    assert not root.findall('asset/texture') and root.find('light') is None


@pytest.mark.parametrize('config', CONFIGS)
def test_mjcf_referenced_meshes_exist_on_disk(config):
    meshes = process(config).findall('asset/mesh')
    assert meshes, f'{config}: no <mesh> assets found in MJCF'
    for mesh in meshes:
        assert mesh.get('file') and Path(mesh.get('file')).is_file(), \
            f'{config}: missing MJCF mesh {mesh.get("file")}'


def test_variants_use_their_own_body_meshes():
    names = {config: {Path(mesh.get('file')).name for mesh in process(config).findall('asset/mesh')} for config in CONFIGS}
    assert 'pantilt_base_100.stl' in names['pt100'] and 'pantilt_base_101.stl' in names['pt101']
    assert 'pantilt_base_101.stl' not in names['pt100']

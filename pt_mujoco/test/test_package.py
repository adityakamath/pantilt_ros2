"""Package boundaries and data lookup without ROS discovery."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pt_mujoco.paths import package_share  # noqa: E402


def test_package_metadata():
    package = ET.parse(ROOT / 'package.xml').getroot()
    assert package.findtext('name') == 'pt_mujoco'
    assert package.findtext('export/build_type') == 'ament_python'
    depends = [entry.text for entry in package if 'depend' in entry.tag]
    assert 'pt_description' in depends
    assert not any(name.startswith('lekiwi') for name in depends)


def test_sibling_description_is_found_in_a_checkout():
    assert (package_share('pt_description') / 'urdf/pantilt.urdf.xacro').is_file()


def test_share_override(monkeypatch, tmp_path):
    monkeypatch.setenv('PT_DESCRIPTION_SHARE', str(tmp_path))
    assert package_share('pt_description') == tmp_path.resolve()
    monkeypatch.setenv('PT_DESCRIPTION_SHARE', str(tmp_path / 'missing'))
    with pytest.raises(FileNotFoundError):
        package_share('pt_description')

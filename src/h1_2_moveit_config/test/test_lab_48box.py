import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[2]
DIRECTORY = ROOT / 'h1_2_description/models/lab_48box'
CONTRACT = json.loads((DIRECTORY / 'PROVENANCE.json').read_text())


def test_original_mesh_scale_and_collision_envelope_match():
    data = (DIRECTORY / 'meshes/48box.stl').read_bytes()
    assert hashlib.sha256(data).hexdigest() == CONTRACT['source_sha256']
    records = np.frombuffer(data, offset=84, dtype=np.dtype([
        ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')]))
    assert len(records) == 61074
    model = ET.parse(DIRECTORY / 'model.sdf').getroot().find('model')
    visual = model.find('./link/visual')
    pose = np.array([float(value) for value in visual.findtext('pose').split()])
    scale = np.array([float(value) for value in visual.findtext('geometry/mesh/scale').split()])
    assert scale == pytest.approx([.00085] * 3)
    vertices = records['vertices'].reshape(-1, 3) * scale
    vertices = vertices @ Rotation.from_euler('xyz', pose[3:]).as_matrix().T + pose[:3]
    dimensions = np.array([float(value) for value in model.findtext('./link/collision/geometry/box/size').split()])
    assert dimensions == pytest.approx(CONTRACT['model_size_xyz_m'])
    assert vertices.min(0) == pytest.approx(-dimensions / 2, abs=1e-9)
    assert vertices.max(0) == pytest.approx(dimensions / 2, abs=1e-9)
    assert CONTRACT['fit_scaling'] == .85


def test_payload_is_free_dynamic_and_inertia_is_positive():
    model = ET.parse(DIRECTORY / 'model.sdf').getroot().find('model')
    assert model.findtext('static') == 'false'
    assert model.findtext('./link/gravity') == 'true'
    assert not model.findall('joint') and not model.findall('plugin')
    mass = float(model.findtext('./link/inertial/mass'))
    assert mass == pytest.approx(.2 * .85 ** 3)
    dimensions = np.array(CONTRACT['model_size_xyz_m'])
    for axis, key in enumerate(('ixx', 'iyy', 'izz')):
        expected = mass * (sum(dimensions ** 2) - dimensions[axis] ** 2) / 12
        assert float(model.findtext('./link/inertial/inertia/' + key)) == pytest.approx(expected)
    for key in ('ixy', 'ixz', 'iyz'):
        assert float(model.findtext('./link/inertial/inertia/' + key)) == 0


def test_contact_sensor_is_observational_and_tracks_envelope():
    model = ET.parse(DIRECTORY / 'model.sdf').getroot().find('model')
    sensor = model.find('./link/sensor')
    assert sensor.get('type') == 'contact'
    assert sensor.findtext('contact/collision') == model.find('./link/collision').get('name')
    assert sensor.findtext('contact/topic') == '/lab_48box/contacts'
    assert not model.findall('plugin')


@pytest.mark.parametrize('name', ['box48_left', 'box48_right'])
def test_spawn_on_table_without_overlapping_centrifuge(name):
    centre = np.array(CONTRACT['table_spawn_world_xyz'][name])
    size = np.array(CONTRACT['model_size_xyz_m'])
    assert centre[0] - size[0] / 2 > .2
    assert centre[0] + size[0] / 2 < 1.0
    assert abs(centre[1]) + size[1] / 2 < .6
    assert abs(centre[1]) - size[1] / 2 > .225
    assert centre[2] - size[2] / 2 - CONTRACT['table_top_m'] == pytest.approx(.001)


def test_original_size_fit_is_not_claimed():
    short, long = sorted(CONTRACT['original_model_size_xyz_m'][:2])
    assert short > CONTRACT['bucket_inner_short_long_m'][0]
    assert long > CONTRACT['bucket_inner_short_long_m'][1]
    assert not CONTRACT['fits_bucket_upright_original_size']
    assert not CONTRACT['robot_loading_validated']
    fitted_short, fitted_long = sorted(CONTRACT['model_size_xyz_m'][:2])
    assert .0009 < (CONTRACT['bucket_inner_short_long_m'][0] - fitted_short) / 2 < .001
    assert fitted_long < CONTRACT['bucket_inner_short_long_m'][1]


@pytest.mark.parametrize('scene,demo,press,payload', [
    ('incubator', 'false', 'false', 'true'),
    ('centrifuge', 'false', 'true', 'true'),
    ('centrifuge', 'false', 'false', 'invalid'),
])
def test_payload_launch_rejects_unsafe_combinations(scene, demo, press, payload):
    from launch import LaunchContext
    path = ROOT / 'h1_2_moveit_config/launch/lab_scene.launch.py'
    spec = importlib.util.spec_from_file_location('lab_scene_payload_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(scene=scene, allow_open_lid_demo=demo,
                                         enable_robot_press=press, with_48box=payload)
    with pytest.raises(ValueError):
        module._scene_actions(context)

import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


DIRECTORY = Path(__file__).resolve().parents[2] / 'h1_2_description/models/centrifuge'


def model():
    return ET.parse(DIRECTORY / 'model.sdf').getroot().find('model')


@pytest.mark.parametrize('side,expected', [
    ('left', '582f79f41b20066c0c2925ede6bbdb108ac2f7c3469e94679b3188d859013675'),
    ('right', '99366066850fb451a1c5cd1891647874c1bb81e9e9f766b2fe0ff7e4d3f4e0ec'),
])
def test_bucket_is_independent_passive_body(side, expected):
    scene = model()
    name = 'bucket_' + side
    link = scene.find(f"./link[@name='{name}']")
    joint = scene.find(f"./joint[@name='{name}_swing_joint']")
    assert link is not None and joint is not None
    assert joint.get('type') == 'revolute'
    assert joint.findtext('parent') == 'rotor' and joint.findtext('child') == name
    assert joint.findtext('axis/xyz') == '0 -1 0'
    assert float(joint.findtext('axis/limit/lower')) < 0
    assert float(joint.findtext('axis/limit/upper')) > math.radians(80)
    assert float(joint.findtext('axis/dynamics/damping')) > 0
    assert joint.find('axis/dynamics/spring_stiffness') is None
    assert float(joint.findtext('axis/limit/effort')) == 0
    assert link.findtext('gravity') == 'true'
    assert link.findtext('self_collide') == 'true'
    assert scene.find(f"./link[@name='rotor']/visual[@name='{name}_visual']") is None
    assert hashlib.sha256((DIRECTORY/'meshes'/(name+'.stl')).read_bytes()).hexdigest() == expected
    assert link.findtext('visual/pose') == link.findtext('collision/pose')
    assert link.findtext('visual/geometry/mesh/uri') == link.findtext('collision/geometry/mesh/uri')
    for plugin in scene.findall('plugin'):
        if 'JointStatePublisher' not in plugin.get('name', ''):
            assert name not in ET.tostring(plugin, encoding='unicode')


@pytest.mark.parametrize('side', ['left', 'right'])
def test_mass_tensor_and_hanging_zero(side):
    link = model().find(f"./link[@name='bucket_{side}']")
    mass = float(link.findtext('inertial/mass'))
    centre = [float(value) for value in link.findtext('inertial/pose').split()[:3]]
    assert .5 < mass < .6
    assert abs(centre[0]) < 1e-8 and abs(centre[1]) < 1e-6 and centre[2] < -.01
    tensor = link.find('inertial/inertia')
    diagonal = [float(tensor.findtext(key)) for key in ('ixx','iyy','izz')]
    cross = [float(tensor.findtext(key)) for key in ('ixy','ixz','iyz')]
    assert min(diagonal) > 0
    assert 2*max(diagonal) < sum(diagonal)
    assert max(abs(value) for value in cross) < 1e-6
    assert float(link.findtext('pose').split()[2]) == pytest.approx(.2375)


def test_both_passive_angles_published_and_default_lid_interlock_retained():
    scene = model()
    publisher = scene.find("./plugin[@name='gz::sim::systems::JointStatePublisher']")
    names = [node.text for node in publisher.findall('joint_name')]
    assert set(names) == {'lid_hinge_joint','rotor_spin_joint',
                          'bucket_left_swing_joint','bucket_right_swing_joint'}
    assert scene.find("./plugin[@name='CentrifugeInterlock']/allow_open_lid_demo") is None
    assert scene.find("./joint[@name='assembly_world_joint']").get('type') == 'fixed'


@pytest.mark.parametrize('button,ordinate', [('lid', .13), ('rotor', -.13)])
def test_front_buttons_face_robot_with_matching_collisions(button, ordinate):
    assembly = model().find("./link[@name='assembly']")
    contract = json.loads((DIRECTORY / 'LID_MOTION.json').read_text())
    normal = contract['button_outward_normal_model']
    centres = {}
    for part in ('base_', '', 'mark_'):
        visual = assembly.find(f"./visual[@name='{button}_button_{part}visual']")
        pose = [float(value) for value in visual.findtext('pose').split()]
        centres[part] = pose[:3]
        assert pose[1] == pytest.approx(ordinate)
        assert pose[3:] == pytest.approx(contract['button_rpy_rad'])
        assert [math.sin(pose[4]), 0, math.cos(pose[4])] == pytest.approx(normal)
        assert normal[0] < -.97
        if part != 'mark_':
            collision = assembly.find(f"./collision[@name='{button}_button_{part}collision']")
            assert collision.findtext('pose') == visual.findtext('pose')
            assert ET.tostring(collision.find('geometry')).strip() == ET.tostring(visual.find('geometry')).strip()
    assert centres[''] == pytest.approx(contract[f'{button}_button_model_m'])
    assert centres[''][2] + .815 == pytest.approx(contract['button_cap_world_height_m'])
    for part, offset in [('base_', -.006), ('mark_', .0055)]:
        assert centres[part] == pytest.approx(
            [centre + offset * axis for centre, axis in zip(centres[''], normal)])
    assert abs(ordinate) + .017 < .22
    assert abs(ordinate) - .017 > .08846


def test_front_buttons_have_separate_robot_approaches():
    contract = json.loads((DIRECTORY / 'LID_MOTION.json').read_text())
    left = contract['lid_button_model_m']
    right = contract['rotor_button_model_m']
    assert left[0] == right[0] and left[2] == right[2]
    assert math.dist(left, right) == pytest.approx(.26)
    assert contract['button_spacing_m'] == pytest.approx(.26)

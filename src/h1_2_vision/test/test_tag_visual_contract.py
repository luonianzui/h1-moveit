import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET


WORKSPACE = Path(__file__).resolve().parents[3]
MODELS = WORKSPACE / 'src/h1_2_description/models'
EXPECTED_PHYSICAL_HASHES = {
    'green_cube': '10718cdeb56a7f8172f1ef8a9587db13d7df48aa393e4817989e34b41696ca72',
    'incubator': '13701430a7d5c0c490591075540edf6d0248279b325e50268e2543280f5dc454',
}


def physical_hash(path):
    root = ET.parse(path).getroot()
    for parent in root.iter():
        for child in list(parent):
            # Observational sensors do not alter mass, joints, collision
            # geometry, friction, or the latch dynamics covered by this hash.
            if child.tag in ('visual', 'sensor'):
                parent.remove(child)
    for element in root.iter():
        element.text = (element.text or '').strip()
        element.tail = ''
        element.attrib = dict(sorted(element.attrib.items()))
    return hashlib.sha256(ET.tostring(root, encoding='utf-8')).hexdigest()


def test_visual_only_change_preserves_sealed_physical_structure():
    for model, expected in EXPECTED_PHYSICAL_HASHES.items():
        assert physical_hash(MODELS / model / 'model.sdf') == expected


def test_all_declared_tag_visuals_are_planes_without_collisions():
    cube = ET.parse(MODELS / 'green_cube/model.sdf').getroot()
    incubator = ET.parse(MODELS / 'incubator/model.sdf').getroot()
    visuals = [value for root in (cube, incubator)
               for value in root.findall('.//visual')
               if value.attrib.get('name', '').startswith('apriltag_')]
    assert {value.attrib['name'] for value in visuals} == {
        'apriltag_10_cabinet_top_visual',
        'apriltag_11_door_outer_visual',
        'apriltag_12_cabinet_top_visual',
        'apriltag_20_top_visual',
        'apriltag_21_positive_x_visual',
        'apriltag_22_negative_x_visual',
        'apriltag_23_top_reacquisition_visual',
    }
    assert all(value.find('./geometry/plane') is not None for value in visuals)
    assert not any('apriltag' in collision.attrib.get('name', '')
                   for root in (cube, incubator)
                   for collision in root.findall('.//collision'))

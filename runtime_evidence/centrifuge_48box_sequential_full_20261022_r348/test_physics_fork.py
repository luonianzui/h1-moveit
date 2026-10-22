import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]


def test_only_world_timing_changes():
    current = ET.parse(EVIDENCE / 'inputs/lab_fast_world.sdf').getroot()
    original = ET.parse(ROOT / 'src/h1_2_description/worlds/lab_scene_world.sdf').getroot()
    assert current.find('./world/physics/max_step_size').text == '0.0005'
    for field in ('max_step_size', 'real_time_update_rate', 'real_time_factor'):
        current.find('./world/physics/' + field).text = original.find('./world/physics/' + field).text
    assert ET.tostring(current) == ET.tostring(original)


def test_fresh_relay_only_changes_step_labels_and_nonfunctional_comments():
    original = (ROOT / 'src/h1_2_moveit_demo/scripts/h1_robot_joint_state_relay_829.py').read_text()
    current = (EVIDENCE / 'inputs/robot_joint_state_relay_05ms.py').read_text()
    import re
    expected = original.replace('SOURCE_STEP_NS = 1_000_000', 'SOURCE_STEP_NS = 500_000').replace('1 ms', '0.5 ms').replace('1MS_GRID', '05MS_GRID')
    expected = re.sub(r'\n[ \t]*#[^\n]*', '', expected)
    assert current == expected


def test_launch_binds_unique_relay_and_self_hash():
    source = (EVIDENCE / 'inputs/gazebo_effort.launch.py').read_text()
    assert str(EVIDENCE / 'inputs/robot_joint_state_relay_05ms.py') in source
    assert 'robot_joint_state_relay_default_sha256 = _sha256_file(' in source
    assert "'--self-sha256', robot_joint_state_relay_sha256" in source


def test_old_model_hashes_preserved():
    expected = {'incubator':'44ff23fc7077f0c232e2e69ff9d73aba336140d6a7951968c9c4241a97e73db9',
                'centrifuge':'f017dfcf10b0e2c583e80bb6f625ad5998c7815f8fa69e3c1eaa7134a3684a55',
                'lab_48box':'dc582ab7ef80536e28836c743f9671b2709d54143cb1d3c2a87a0e3c5f9bf1ec'}
    for name, digest in expected.items():
        assert hashlib.sha256((ROOT / 'src/h1_2_description/models' / name / 'model.sdf').read_bytes()).hexdigest() == digest


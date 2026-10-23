from pathlib import Path
import xml.etree.ElementTree as element_tree

from contact_phase_policy import bounded_motion


EVIDENCE = Path(__file__).resolve().parent


def test_only_retrieval_empty_approach_gets_medium_rate_with_original_tracking_gate():
    assert bounded_motion('paired_retrieval_approach', .8) == (.4, .002)
    assert bounded_motion('paired_retrieval_approach', .05) == (.05, .002)
    assert bounded_motion('paired_load_approach', .8) == (.8, .015)
    assert bounded_motion('paired_retrieval_transfer', .25) == (.25, .015)


def test_model_velocity_limit_and_default_runtime_fault_guard_are_preserved():
    tree = element_tree.parse(EVIDENCE / 'inputs/robot_dual.urdf').getroot()
    for hand in ('left', 'right'):
        for number in (1, 2):
            joint = tree.find("./joint[@name='" + hand + '_dex1_finger_' + str(number) + "_joint']")
            assert float(joint.find('limit').get('velocity')) == .2
    source = (EVIDENCE / 'execute_loading.py').read_text()
    from speed_source import restore_speed_source
    source = restore_speed_source('execute_loading.py', source)
    assert 'if velocity_violation(value) and self.velocity_fault is None:' in source
    assert "raise RuntimeError('Effort gripper velocity fault: '" in source

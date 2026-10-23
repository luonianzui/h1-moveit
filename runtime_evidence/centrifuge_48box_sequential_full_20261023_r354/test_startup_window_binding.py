from pathlib import Path


EVIDENCE = Path(__file__).resolve().parent


def test_gate_is_after_actual_settling_and_before_static_leg_action():
    source = (EVIDENCE / 'execute_cycle.py').read_text()
    start = source.index('    def prepare_open(self):')
    section = source[start:source.index('    def return_neutral(self):', start)]
    assert section.index("self.record('passive_posture_settled'") < section.index('validate_startup_window(self.sim)')
    assert section.index('validate_startup_window(self.sim)') < section.index('self.initialize_leg_posture()')
    assert 'validate_startup_window(0)' not in section


def test_original_static_leg_action_limits_are_not_changed():
    source = (EVIDENCE / 'execute_cycle.py').read_text()
    start = source.index('    def initialize_leg_posture(self):')
    section = source[start:source.index('    def receive_leg_posture_state', start)]
    assert 'position=.005, velocity=.03' in section
    assert 'goal.goal_time_tolerance.sec = 10' in section
    assert 'targets, measured = leg_posture_targets' in section
    assert 'assert response.error_code == 0' in section


def test_preparation_outputs_are_current_and_parent_failure_is_protected():
    source = (EVIDENCE / 'prepare_candidate.sh').read_text()
    assert 'evidence="${workspace}/runtime_evidence/' + EVIDENCE.name + '"' in source
    assert "'centrifuge_48box_empty_retreat_20261013_r246'" in (EVIDENCE / 'verify_dual_protection.py').read_text()

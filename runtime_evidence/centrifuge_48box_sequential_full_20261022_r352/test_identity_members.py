from pathlib import Path

from identity_members import VALIDATION_FILES, is_identity_member


def test_sealer_own_output_never_becomes_self_reference():
    assert not is_identity_member(Path('seal.log'))
    assert not is_identity_member(Path('launch_dispatch.log'))
    assert not is_identity_member(Path('execution.log'))


def test_completed_validation_logs_are_explicit_members():
    assert all(is_identity_member(Path(name)) for name in VALIDATION_FILES)
    assert is_identity_member(Path('tests_revision3.log'))
    assert not is_identity_member(Path('unknown_running_test.log'))


def test_sources_and_contracts_remain_sealed():
    assert all(is_identity_member(Path(name)) for name in (
        'execute_cycle.py', 'supervise_completion.py', 'build_scene.sh',
        'FULL_CYCLE_PLAN.md', 'protected_before.json', 'control_fork_audit.json'))


def test_manifest_report_and_postrun_media_are_not_preexecution_members():
    assert all(not is_identity_member(Path(name)) for name in (
        'identity.json', 'REPORT.md', 'SHA256SUMS', 'full_cycle.mp4', 'final_scene.png'))


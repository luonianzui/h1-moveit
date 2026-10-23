import ast
import hashlib
import json
from pathlib import Path

from contact_phase_policy import bounded_motion
from dual_motion import common_arm_timing
from probe_medium_retrieval import actual_approach_fixture
from inherited_source_identity import restore_inherited_source


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_bucket_axis_exit_full_20261015_r282'


def test_actual_path_retiming_preserves_every_point_and_twenty_ms_floor():
    actual, samples = actual_approach_fixture()
    old = common_arm_timing(actual, samples, .13)
    candidate = common_arm_timing(actual, samples, bounded_motion('paired_retrieval_approach', .8)[0])
    assert candidate['points'] == old['points']
    assert candidate['groups'] == old['groups']
    assert candidate['duration_sim_s'] < old['duration_sim_s'] - 8
    assert all(later - earlier >= .020 - 1e-12
               for earlier, later in zip(candidate['times'], candidate['times'][1:]))


def test_actual_executor_planner_grip_and_geometry_only_have_verified_identity_bindings():
    records = {item['relative']: item for item in json.loads((EVIDENCE / 'derivation.json').read_text())['records']}
    for name in ('execute_dual.py', 'dual_worker.py', 'dual_motion.py', 'execute_loading.py',
                 'table_bucket_clearance.py', 'table_clearance_audit.py', 'paired_setdown_braking.py'):
        assert hashlib.sha256((PARENT / name).read_bytes()).hexdigest() == records[name]['parent_sha256']
        content = (EVIDENCE / name).read_bytes()
        content = restore_inherited_source(name, content.decode()).encode()
        assert hashlib.sha256(content).hexdigest() == records[name]['derived_sha256']


def test_supervisor_uses_bounded_terminal_retries_and_ordered_cleanup():
    source = (EVIDENCE / 'supervise_completion.py').read_text()
    ast.parse(source)
    assert "restore_with_retries(camera, 'terminal_overhead')" in source
    assert 'finish_before_retirement(inspect_terminal, finish_video, retire_scene,' in source
    assert 'lambda: recorder_alive(identity)' in source
    assert "run/completion_finalization.json" in source


def test_preparation_and_build_bind_only_the_exclusive_candidate_output():
    prepare = (EVIDENCE / 'prepare_candidate.sh').read_text()
    build = (EVIDENCE / 'build_scene.sh').read_text()
    assert 'evidence="${workspace}/runtime_evidence/' + EVIDENCE.name + '"' in prepare
    assert 'evidence="' + str(EVIDENCE) + '"' in build
    assert 'install_lab_scene_48box_sequential_full_20261023_r356/local_setup.bash' in prepare

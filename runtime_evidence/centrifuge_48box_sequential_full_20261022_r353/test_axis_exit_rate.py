from acceleration_source import restore_acceleration, restore_acceleration_bytes
import hashlib
from test_return_height_inverse import undo_return_height
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from test_frame_source_inverse import undo_frame_integration

from dual_motion import common_arm_timing
from sequential_cycle import SequentialCycle


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261019_r306'
CHANGE = "        if phase == 'seq_retrieval_retrieve_axis_exit':\n            return min(requested, .03), .002\n"


@pytest.mark.parametrize('requested,expected', [(.01, .01), (.03, .03), (.13, .03), (.25, .03), (.8, .03)])
def test_axis_exit_uses_original_pilot_precision(requested, expected):
    cycle = SimpleNamespace(holders=SimpleNamespace(rows={'left': dict(payload='left')}))
    assert SequentialCycle.motion_limits(cycle, 'seq_retrieval_retrieve_axis_exit', requested) == (expected, .002)


def test_only_declared_runtime_change_from_sealed_parent():
    identity_file = PARENT / 'identity.json'
    assert hashlib.sha256(identity_file.read_bytes()).hexdigest() == '1ba2ef37c4168f0244b3f5872ffe6b358426ea125195b49f596dfa8798ed4e84'
    manifest = json.loads(identity_file.read_text())['sha256']
    for name in ('sequential_cycle.py', 'sequential_planning.py', 'sequential_force.py',
                 'sequential_geometry.py', 'sequential_support.py', 'sequential_worker.py',
                 'release_alignment.py', 'execute_dual.py', 'loading_contact_guard.py'):
        old = (PARENT / name).read_text()
        assert hashlib.sha256(old.encode()).hexdigest() == manifest[name]
        current = restore_acceleration(name, (ROOT / name).read_text())
        if name == 'sequential_cycle.py':
            current = undo_frame_integration(current)
            assert current.count(CHANGE) == 1
            current = current.replace(CHANGE, '')
        if name == 'sequential_planning.py':
            current = undo_return_height(current)
            current = current.replace('from load_bound_reserve import InteriorLoadPlanner\n', '')
            current = current.replace('planner = InteriorLoadPlanner(request, reference)', 'planner = SinglePlanner(request, reference)')
            current = current.replace('from pilot_centering import centered_pilot_target\n', '')
            current = current.replace("    if retrieval:\n        target = centered_pilot_target(body, request['poses'], request['payload'])\n", '')
        assert current == old


def test_actual_failed_path_keeps_all_joint_samples_and_slows_axis_exit():
    request_path = PARENT / 'run/15_retrieval_transport_left_request.json'
    result_path = PARENT / 'run/15_retrieval_transport_left_result.json'
    assert hashlib.sha256(request_path.read_bytes()).hexdigest() == '330885a77acef3cc03cddd6bec1f4bfc06f2e805f716fba762ee7f4433b25f89'
    assert hashlib.sha256(result_path.read_bytes()).hexdigest() == '34c74fcb4d7e193c4b592a267afdc4992adcaec9b1ff972e9dcbfa81f1a43681'
    request, result = json.loads(request_path.read_text()), json.loads(result_path.read_text())
    stage = result['stages'][0]
    assert stage['phase'] == 'retrieve_axis_exit'
    old = common_arm_timing(request['positions'], stage['samples'], .25)
    new = common_arm_timing(request['positions'], stage['samples'], .03)
    assert old['points'] == new['points'] and old['groups'] == new['groups']
    assert new['duration_sim_s'] > old['duration_sim_s'] * 4
    for index in range(1, len(new['points'])):
        duration = new['times'][index] - new['times'][index - 1]
        assert duration >= .02 - 1e-12
        assert max(abs(new['points'][index][name] - new['points'][index - 1][name]) / duration
                   for name in new['points'][index]) <= .03 + 1e-12

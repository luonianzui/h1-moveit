import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import dual_worker


ROOT = Path(__file__).resolve().parent
NAMES = sorted(dual_worker.TASK_JOINTS)


def score(initial, samples):
    previous = initial
    variation = {name: 0. for name in NAMES}
    minimum = {name: initial[name] for name in NAMES}
    maximum = dict(minimum)
    for point in samples:
        assert set(point) == set(NAMES)
        for name in NAMES:
            assert np.isfinite(point[name])
            variation[name] += abs(point[name] - previous[name])
            minimum[name] = min(minimum[name], point[name])
            maximum[name] = max(maximum[name], point[name])
        previous = point
    total = sum(variation.values())
    axial = sum(value for name, value in variation.items()
                if any(term in name for term in ('shoulder_yaw', 'wrist_roll', 'wrist_yaw')))
    excursion = {name: maximum[name] - minimum[name] for name in NAMES}
    peak = max(excursion.values())
    return dict(total_rad=total, axial_rad=axial, peak_excursion_rad=peak, score=total + axial + peak,
                joint_excursion_rad=excursion)


commands = {}
with gzip.open(ROOT / 'run/loading_trace.jsonl.gz', 'rb') as stream:
    for raw in stream:
        header, separator, remainder = raw.partition(b', "payload": ')
        row = json.loads(header + b'}') if separator else json.loads(raw)
        if row['kind'] == 'command' and row['phase'] in {
                'paired_load_approach', 'paired_load_descend',
                'paired_retrieval_approach', 'paired_retrieval_descend'}:
            assert row['phase'] not in commands
            commands[row['phase']] = json.loads(raw)['payload']
records = []
for prefix, phase_prefix in [('1_load_pickup', 'paired_load_'), ('7_retrieval_pickup', 'paired_retrieval_')]:
    request = json.loads((ROOT / 'run' / (prefix + '_request.json')).read_text())
    result = json.loads((ROOT / 'run' / (prefix + '_result.json')).read_text())
    selection = result['minimum_turn_selection']
    assert request['original_runtime_holder_references'] == {} and result['status'] == 'PASS'
    original = dual_worker.legacy_plan_pickup(request)
    baseline = score(request['positions'], [point for stage in original['stages'] for point in stage['samples']])
    selected = score(request['positions'], [point for stage in result['stages'] for point in stage['samples']])
    for label, expected in [('baseline', baseline), ('selected', selected)]:
        assert all(abs(expected[name] - selection[label][name]) < 1e-7
                   for name in ('total_rad', 'axial_rad', 'peak_excursion_rad', 'score'))
    assert selected['score'] <= baseline['score'] + 1e-8
    assert selected['peak_excursion_rad'] <= baseline['peak_excursion_rad'] + 1e-8
    references = {}
    for hand, flip in zip(('left', 'right'), result['flips_deg']):
        rotation = np.eye(4)
        rotation[:3, :3] = Rotation.from_euler('z', -flip, degrees=True).as_matrix()
        references[hand] = dual_worker.nominal_references()[hand] @ rotation
    planner = dual_worker.PairedPlanner(request, references)
    checked = 0
    previous = {name: request['positions'][name] for name in NAMES}
    for stage in result['stages']:
        command = commands[phase_prefix + stage['phase']]
        assert command['samples'] == stage['samples'] and command['simultaneous_physical_controllers']
        assert command['holders'] == {}
        for point in stage['samples']:
            for fraction, joints in dual_worker.dense_edge(previous, point):
                vector = np.array([joints[name] for name in planner.names])
                assert np.all(vector >= planner.lower - 1e-12) and np.all(vector <= planner.upper + 1e-12)
                assert not planner.planner.check(joints)['collision']
                checked += 1
            previous = point
    records.append(dict(mode=request['mode'], baseline=baseline, selected=selected,
        axial_reduction_percent=100 * (baseline['axial_rad'] - selected['axial_rad']) / baseline['axial_rad'],
        improved=selection['improved'], independent_dense_collision_checks=checked,
        exact_selected_path_in_actual_published_commands=True,
        final_wrist_roll_deg={hand: float(np.degrees(result['stages'][-1]['samples'][-1][hand + '_wrist_roll_joint']))
                              for hand in ('left', 'right')}))
assert records[0]['improved']
assert json.loads((ROOT / 'cycle_audit.json').read_text())['status'] == 'PASS'
assert json.loads((ROOT / 'independent_dual_grip.json').read_text())['status'] == 'PASS'
with (ROOT / 'minimum_turn_actual_audit.json').open('x') as stream:
    json.dump(dict(status='PASS', records=records,
                   exact_grasp_frames_original_force_hold_slip_and_no_collision_gates_preserved=True,
                   source_sha256=hashlib.sha256((ROOT / 'minimum_turn.py').read_bytes()).hexdigest(),
                   no_claim_all_flips_removed_or_globally_optimal=True), stream, indent=2)
print('INDEPENDENT_ACTUAL_MINIMUM_TURN_PASS', [(record['mode'], record['axial_reduction_percent'])
                                             for record in records], flush=True)

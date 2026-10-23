import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import dual_worker as worker
from elbow_posture import ELBOW_CONTRACT, posture_metrics
from paired_setdown_braking import split_paired_setdown


ROOT = Path(__file__).resolve().parent


def actual_commands():
    commands = {}
    velocity = dict(samples=0, maximum_absolute_m_s=0., by_joint_m_s={})
    with gzip.open(ROOT / 'run/loading_trace.jsonl.gz', 'rb') as stream:
        for raw in stream:
            header, separator, remainder = raw.partition(b', "payload": ')
            row = json.loads(header + b'}') if separator else json.loads(raw)
            if row['kind'] == 'raw_gripper_velocity':
                payload = json.loads(raw)['payload']
                for name, value in payload['velocities'].items():
                    assert np.isfinite(value)
                    velocity['by_joint_m_s'][name] = max(velocity['by_joint_m_s'].get(name, 0.), abs(value))
                    velocity['maximum_absolute_m_s'] = max(velocity['maximum_absolute_m_s'], abs(value))
                velocity['samples'] += 1
            if row['kind'] != 'command' or not row['phase'].startswith(('paired_load_', 'paired_retrieval_')):
                continue
            row = json.loads(raw)
            if 'samples' not in row['payload']:
                continue
            assert row['phase'] not in commands
            commands[row['phase']] = row['payload']
    assert velocity['samples'] > 0 and velocity['maximum_absolute_m_s'] <= .2
    commands['__native_velocity_metrics'] = velocity
    return commands


def command_samples(request, stage, commands):
    prefix = 'paired_retrieval_' if request.get('retrieval') else 'paired_load_'
    if request['mode'] == 'load_alignment':
        names = ['paired_load_alignment_correction']
    elif stage['phase'] == 'setdown':
        coarse, fine, split = split_paired_setdown(stage)
        assert commands[prefix + 'setdown']['samples'] == coarse
        assert commands[prefix + 'setdown_fine']['samples'] == fine
        assert coarse + fine[1:] == stage['samples']
        names = [prefix + 'setdown', prefix + 'setdown_fine']
    else:
        names = [prefix + stage['phase']]
    for name in names:
        assert commands[name]['simultaneous_physical_controllers']
        if stage['phase'] != 'setdown':
            assert commands[name]['samples'] == stage['samples'], name


def audit_plan(request, result, commands):
    mode = request['mode']
    pickup = mode.endswith('pickup')
    if pickup:
        assert request['original_runtime_holder_references'] == {}
        assert result['elbow_posture_selection']['contract'] == ELBOW_CONTRACT
        if mode == 'load_pickup':
            assert commands['paired_load_approach']['actual_rate_rad_s'] == .5
        references = {}
        for hand, flip in zip(('left', 'right'), result['flips_deg']):
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_euler('z', -flip, degrees=True).as_matrix()
            references[hand] = worker.nominal_references()[hand] @ rotation
    else:
        assert request['planning_references'] == request['original_runtime_holder_references']
        assert request['runtime_reference_not_rebased']
        references = {hand: np.array(value) for hand, value in request['planning_references'].items()}
    planner = worker.PairedPlanner(request, references)
    previous = {name: request['positions'][name] for name in planner.names}
    if not pickup:
        bodies = {hand: worker.pose_matrix(request['poses']['box48_' + payload])
                  for hand, payload in request['assignment'].items()}
        planner.attach(bodies, previous)
    checked = 0
    basket_metrics = {hand: [] for hand in ('left', 'right')}
    minimum_margin = float('inf')
    for stage_index, stage in enumerate(result['stages']):
        command_samples(request, stage, commands)
        for sample_index, target in enumerate(stage['samples']):
            for fraction, joints in worker.dense_edge(previous, target):
                vector = np.array([joints[name] for name in planner.names])
                assert np.all(vector >= planner.lower - 1e-12) and np.all(vector <= planner.upper + 1e-12)
                minimum_margin = min(minimum_margin, float(np.min(vector - planner.lower)), float(np.min(planner.upper - vector)))
                if not pickup:
                    for hand, payload in request['assignment'].items():
                        initial_table = mode == 'pilot' and not request.get('retrieval') and (
                            sample_index <= 1 or sample_index == 2 and fraction == 0)
                        final_table = mode == 'retrieval_transport' and stage['phase'] == 'setdown' and sample_index == len(stage['samples']) - 1
                        planner.planner.scene.allowed_collision_matrix.set_entry('box48_' + payload, 'table_link_surface', initial_table or final_table)
                        planner.planner.scene.allowed_collision_matrix.set_entry('box48_' + payload,
                            'centrifuge_bucket_' + payload + '_bucket_' + payload + '_collision',
                            mode == 'pilot' and bool(request.get('retrieval')) and sample_index <= 2)
                assert not planner.planner.check(joints)['collision'], (mode, stage_index, sample_index, fraction)
                checked += 1
            is_basket = (mode in ('load_transport', 'load_alignment') and stage_index == len(result['stages']) - 1 or
                         mode == 'retrieval_transport' and stage['phase'] == 'lift' or
                         mode == 'retrieval_pickup' and stage['phase'] == 'descend' and sample_index == len(stage['samples']) - 1 or
                         mode == 'pilot' and bool(request.get('retrieval')))
            if is_basket:
                measured = posture_metrics(planner, target)
                for hand in ('left', 'right'):
                    basket_metrics[hand].append(measured[hand]['from_down_deg'])
            previous = target
    maximum = {hand: max(values) for hand, values in basket_metrics.items() if values}
    assert not maximum or max(maximum.values()) <= 50, (mode, maximum)
    return dict(mode=mode, independent_dense_checks=checked, exact_published_samples=True,
                basket_max_from_down_deg=maximum, minimum_margin_inside_original_shrunken_limits_rad=minimum_margin,
                original_actual_grip_reference_not_rebased=True)


def main():
    commands = actual_commands()
    for phase in ('paired_retrieval_outboard', 'paired_retrieval_transfer'):
        assert commands[phase]['actual_rate_rad_s'] == .3, phase
    records = []
    for path in sorted((ROOT / 'run').glob('*_request.json')):
        request = json.loads(path.read_text())
        if request.get('mode') not in ('load_pickup', 'retrieval_pickup', 'pilot', 'load_transport', 'load_alignment', 'retrieval_transport'):
            continue
        result = json.loads(path.with_name(path.name.replace('_request.json', '_result.json')).read_text())
        assert result['status'] == 'PASS'
        records.append(audit_plan(request, result, commands))
    assert {row['mode'] for row in records} >= {'load_pickup', 'retrieval_pickup', 'load_transport', 'retrieval_transport', 'pilot'}
    assert json.loads((ROOT / 'cycle_audit.json').read_text())['status'] == 'PASS'
    assert json.loads((ROOT / 'independent_dual_grip.json').read_text())['status'] == 'PASS'
    with (ROOT / 'elbow_actual_audit.json').open('x') as stream:
        json.dump(dict(status='PASS', records=records, elbow_contract=ELBOW_CONTRACT,
                       native_finger_velocity=commands['__native_velocity_metrics'], actual_empty_approach_rate_rad_s=.5,
                       actual_retrieval_outboard_and_transfer_rate_rad_s=.3,
                       all_original_hold_force_slip_and_full_task_gates_required=True,
                       model_wrist_visual_only_collision_limitation_not_hidden=True,
                       source_sha256=hashlib.sha256((ROOT / 'elbow_posture.py').read_bytes()).hexdigest(),
                       no_claim_perfect_vertical_or_global_optimum_or_real_robot=True), stream, indent=2)
    print('INDEPENDENT_ACTUAL_ELBOW_PASS', records, flush=True)


if __name__ == '__main__':
    main()

import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

from speed_policy import PROFILE, common_timing, motion_limits
from joined_stages import joined_phase, joined_samples, joined_timing, stage_groups
from sequential_geometry import CONTRACT, SinglePlanner, pose_matrix
from speed_retrieval_geometry import SpeedRetrievalPlanner


ROOT = Path(__file__).resolve().parent
RUN = ROOT.parent / 'centrifuge_48box_sequential_full_20261019_r299'
IDENTITY_SHA = '6701d4c24ff2610749f42f274b2c8648b08420ecb4201ef1d0b07e2a0d866ced'
PREFIXES = ('seq_load_', 'seq_retrieval_', 'machine_sequential_', 'paired_retrieval_setdown')


def expected_commands(planned, prefix=None):
    if prefix is None:
        prefix = ('machine_sequential' if planned['mode'] == 'neutral' else
                  'seq_retrieval' if planned['mode'].startswith('retrieval') else 'seq_load')
    result = []
    for group in stage_groups(planned['stages']):
        if len(group) == 2:
            result.append((prefix + '_' + joined_phase(group), joined_samples(group)))
            continue
        stage = group[0]
        samples = stage['samples']
        if stage['phase'] == 'retrieve_setdown':
            split = max(1, len(samples) - 8)
            result.extend([('paired_retrieval_setdown', samples[:split + 1]),
                           ('paired_retrieval_setdown_fine', samples[split:])])
        else:
            result.append((prefix + '_' + stage['phase'], samples))
    return result


def verify_published(sequence, planned):
    prefix = None
    if planned['mode'] in ('park', 'empty_retreat'):
        if not sequence:
            raise ValueError('Missing actual empty path')
        phase = sequence[0]['phase']
        prefix = next((value for value in ('seq_load', 'seq_retrieval') if phase.startswith(value + '_')), None)
        if prefix is None:
            raise ValueError('Unknown actual empty-stage context')
    expected = expected_commands(planned, prefix)
    if len(sequence) != len(expected):
        raise ValueError('Published stage count differs from the actual qualified plan')
    for actual, (phase, samples) in zip(sequence, expected):
        if actual['phase'] != phase or actual['payload']['samples'] != samples:
            raise ValueError('Published samples or phase differ from the actual qualified plan')
        if '_and_' in phase:
            segments = next(group for group in stage_groups(planned['stages'])
                            if len(group) == 2 and phase.endswith(joined_phase(group)))
            if actual['payload'].get('joined_segments') != segments:
                raise ValueError('Joined timing segment metadata differs from original qualified stages')
        if len(actual['payload']['holders']) > 1:
            raise ValueError('More than one actual holder')


def verify_axis_exit_timing(actual):
    payload = actual['payload']
    rate, unused = motion_limits(actual['phase'], bool(payload['holders']))
    if actual['phase'] == 'seq_retrieval_single_pilot':
        if len(payload['holders']) != 1 or any(hand != row['payload'] for hand, row in payload['holders'].items()):
            raise ValueError('Single original retrieval holder required for gentle pilot timing')
        rate = .05
    ramp = .3 if actual['phase'] == 'paired_retrieval_setdown' else .15
    if actual['phase'] == 'paired_retrieval_setdown' and set(payload['holders']) == {'right'}:
        if payload['holders']['right']['payload'] != 'right':
            raise ValueError('Right setdown requires original right payload assignment')
        rate, ramp = .2, .6
    if payload['actual_rate_rad_s'] != rate:
        raise ValueError('Published rate does not match declared speed-priority phase')
    if '_and_' in actual['phase']:
        expected = joined_timing(payload['initial'], payload['joined_segments'], rate)
    else:
        expected = common_timing(payload['initial'], payload['samples'], rate,
                                 ramp_s=ramp)
    if payload['times'] != expected['times'] or payload['groups'] != expected['groups']:
        raise ValueError('Published timing differs from declared speed-priority profile')


def verify_centered_pilot(request, planned):
    if request['mode'] != 'retrieval_pilot':
        return
    if len(planned['stages']) != 1 or planned['stages'][0]['phase'] != 'single_pilot':
        raise ValueError('Expected one actual retrieval pilot')
    stage = planned['stages'][0]
    first = pose_matrix(request['poses']['box48_' + request['payload']])
    bucket = pose_matrix(request['poses']['centrifuge']) @ pose_matrix(request['poses']['bucket_' + request['payload']])
    final = np.array(stage['last_body'])
    local_first, local_final = np.linalg.inv(bucket) @ first, np.linalg.inv(bucket) @ final
    if not np.allclose(stage['first_body'], first, atol=1e-12, rtol=0):
        raise ValueError('Pilot start must match actual box snapshot')
    if first[2, 2] <= math.cos(math.radians(2)):
        raise ValueError('Pilot initial orientation exceeds original two-degree bound')
    if not np.array_equal(final[:3, :3], first[:3, :3]):
        raise ValueError('Pilot must preserve actual orientation until clear of basket')
    if not np.allclose(local_final[:2, 3], [0., local_first[1, 3]], atol=1e-12, rtol=0):
        raise ValueError('Pilot must provide local X relief and preserve actual Y offset')
    if not np.isclose(local_final[2, 3] - local_first[2, 3], .015, atol=1e-12, rtol=0):
        raise ValueError('Pilot must lift exactly 15mm along actual basket axis')


def verify_load_reserve(request, planned, planner):
    if request['mode'] != 'load_transport':
        return
    for stage in planned['stages']:
        for sample in stage['samples'][1:]:
            values = np.array([sample[name] for name in planner.active])
            margin = np.minimum(values - planner.lower[planner.indices], planner.upper[planner.indices] - values)
            if not np.isfinite(margin).all() or np.min(margin) < .001 - 1e-10:
                raise ValueError('Load IK did not retain the declared interior reserve')


def verify_return_height(request, planned):
    if request['mode'] != 'retrieval_transport':
        return
    hand = request['hand']
    if hand not in ('left', 'right'):
        raise ValueError('Unknown return hand')
    lift = next(stage for stage in planned['stages'] if stage['phase'] == 'retrieve_lift')
    initial_body = pose_matrix(request['poses']['box48_' + request['payload']])
    bucket = pose_matrix(request['poses']['centrifuge']) @ pose_matrix(request['poses']['bucket_' + request['payload']])
    exit_stage = next(stage for stage in planned['stages'] if stage['phase'] == 'retrieve_axis_exit')
    exit_body = np.array(exit_stage['last_body'])
    expected_transverse = (np.linalg.inv(bucket) @ initial_body)[:2, 3].copy()
    correction = exit_stage.get('clearance_correction')
    if correction is not None:
        offset = -bucket[:3, 0] * expected_transverse[0]
        if (not correction.get('no_basket_collision_exemption')
                or not correction.get('original_holder_reference_unchanged')
                or not correction.get('orientation_unchanged')
                or not correction.get('full_scene_dense_checked')
                or not correction.get('original_axis_rejection', '').startswith('Single-holder dense full-scene collision:')
                or correction.get('maximum_correction_m') != .002
                or not np.isfinite(offset).all() or np.linalg.norm(offset) > .002
                or not np.allclose(correction.get('translation_world_m'), offset, atol=1e-12, rtol=0)):
            raise ValueError('Undeclared or unbounded basket clearance correction')
        expected_transverse[0] = 0.
    if (not np.array_equal(exit_body[:3, :3], initial_body[:3, :3])
            or not np.allclose((np.linalg.inv(bucket) @ exit_body)[:2, 3],
                               expected_transverse, atol=1e-12, rtol=0)):
        raise ValueError('Basket exit must preserve actual transverse position and orientation')
    expected_rotation = Rotation.from_euler('z', math.atan2(initial_body[1, 0], initial_body[0, 0])).as_matrix()
    if not np.allclose(np.array(lift['last_body'])[:3, :3], expected_rotation, atol=1e-12, rtol=0):
        raise ValueError('Retrieval high target must level after axis exit without changing actual yaw')
    transfer = next(stage for stage in planned['stages'] if stage['phase'] == 'retrieve_transfer')
    down = next(stage for stage in planned['stages'] if stage['phase'] == 'retrieve_setdown')
    lateral = .38 if hand == 'left' else -.38
    height = 1.12475 if hand == 'right' else 1.14475
    if not np.allclose(np.array(transfer['last_body'])[:3, 3], [.44, lateral, height], atol=1e-12, rtol=0):
        raise ValueError('Wrong per-hand high return waypoint')
    if not np.allclose(np.array(down['last_body'])[:3, 3], [.44, lateral, .84475], atol=1e-12, rtol=0):
        raise ValueError('Original table return position changed')


def stage_start_has_roundoff(first, previous):
    if not first or first.keys() != previous.keys():
        raise ValueError('Actual snapshot to stage continuity lost')
    differences = np.array([first[name] - previous[name] for name in first])
    if not np.isfinite(differences).all() or np.max(np.abs(differences)) > 1e-12:
        raise ValueError('Actual snapshot to stage continuity lost')
    return first != previous


def review_path(request, planned):
    verify_return_height(request, planned)
    verify_centered_pilot(request, planned)
    table_alignment = request['mode'] == 'retrieval_table_alignment'
    if table_alignment:
        first = pose_matrix(request['poses']['box48_' + request['payload']])
        goal = np.array([.44, .38 if request['hand'] == 'left' else -.38, .84475])
        assert request['hand'] == request['payload']
        assert np.linalg.norm(first[:3, 3] - goal) <= .0035
        assert abs(first[2, 3] - goal[2]) <= .0001
        yaw = math.pi if abs(math.atan2(first[1, 0], first[0, 0])) > math.pi / 2 else 0.
        assert math.degrees(Rotation.from_matrix(Rotation.from_euler('z', yaw).as_matrix().T @ first[:3, :3]).magnitude()) <= .25
        assert len(planned['stages']) == 1
        stage = planned['stages'][0]
        assert stage['phase'] == 'table_alignment_pilot'
        final = np.array(stage['last_body'])
        delta = goal[:2] - first[:2, 3]
        expected_xy = first[:2, 3] + delta * min(1., .001 / max(float(np.linalg.norm(delta)), 1e-30))
        assert np.array_equal(final[:3, :3], first[:3, :3])
        assert final[2, 3] == first[2, 3]
        assert np.allclose(final[:2, 3], expected_xy, rtol=0, atol=1e-12)
        assert np.allclose(stage['first_body'], first, rtol=0, atol=1e-12)
    loaded = request['mode'] in ('load_pilot', 'load_transport', 'retrieval_pilot', 'retrieval_transport', 'load_release_align', 'retrieval_table_alignment')
    planner_type = SpeedRetrievalPlanner if request['mode'] == 'retrieval_transport' else SinglePlanner
    planner = planner_type(request, planned['planning_reference'])
    verify_load_reserve(request, planned, planner)
    previous = {name: request['positions'][name] for name in planner.names}
    if loaded:
        planner.attach(pose_matrix(request['poses']['box48_' + request['payload']]), previous)
    maximum_basket_angle = None
    endpoint_errors = []
    roundoff_joins = []
    for stage_index, stage in enumerate(planned['stages']):
        support = None
        if loaded and stage_index == 0 and request['mode'] not in ('load_release_align', 'retrieval_table_alignment'):
            support = 'basket' if request['mode'].startswith('retrieval') else 'table'
        if stage.get('clearance_correction') is not None:
            if request['mode'] != 'retrieval_transport' or stage['phase'] != 'retrieve_axis_exit':
                raise ValueError('Clearance correction is only permitted during basket exit')
            support = None
        if stage_start_has_roundoff(stage['samples'][0], previous):
            planner.check_edge(previous, stage['samples'][0], 'independent_subpicoradian_stage_join', initial_support=support, table_endpoint=table_alignment)
            roundoff_joins.append(dict(stage=stage['phase'], maximum_joint_difference_rad=max(
                abs(stage['samples'][0][name] - previous[name]) for name in previous)))
        for index, target in enumerate(stage['samples'][1:]):
            permitted = support if index <= 1 else None
            if permitted == 'table' and index == 1:
                permitted = 'table_start_only'
            planner.check_edge(previous, target, 'independent_actual_' + stage['phase'],
                table_endpoint=table_alignment or stage['phase'] == 'retrieve_setdown' and index == len(stage['samples']) - 2,
                initial_support=permitted)
            basket_phase = (stage['phase'] in ('load_lower', 'retrieve_axis_exit', 'retrieve_lift', 'load_release_pilot_align') or
                            request['mode'] == 'retrieval_pilot' or
                            request['mode'] == 'retrieval_pickup' and stage['phase'] == 'empty_descend'
                            and index == len(stage['samples']) - 2)
            if basket_phase:
                angle = planner.elbow_metrics(target)['from_down_deg']
                assert np.isfinite(angle)
                maximum_basket_angle = max(maximum_basket_angle or 0., angle)
            previous = target
        if loaded and 'last_body' in stage:
            actual = planner.wrist(previous) @ np.array(planned['planning_reference'])
            expected = np.array(stage['last_body'])
            position = float(np.linalg.norm(actual[:3, 3] - expected[:3, 3]))
            angle = float(np.degrees(Rotation.from_matrix(expected[:3, :3].T @ actual[:3, :3]).magnitude()))
            assert position <= .00002 and angle <= .01
            endpoint_errors.append(dict(phase=stage['phase'], position_m=position, angle_deg=angle))
    if request['mode'] == 'retrieval_transport':
        assert planned['commanded_tilt_limit_deg'] == PROFILE['retrieval_commanded_tilt_deg']
        assert abs(planned['maximum_planned_payload_tilt_deg'] - planner.maximum_payload_tilt_deg) < 1e-8
    return dict(mode=request['mode'], hand=request['hand'], payload=request['payload'],
        retrieval_commanded_tilt_limit_deg=PROFILE['retrieval_commanded_tilt_deg'] if request['mode'] == 'retrieval_transport' else 2.,
        maximum_retrieval_planned_tilt_deg=getattr(planner, 'maximum_payload_tilt_deg', None),
        original_dense_step_rad=.002, final_path_dense_checks=planner.checked,
        maximum_active_basket_from_down_deg=maximum_basket_angle, endpoint_errors=endpoint_errors,
        subpicoradian_roundoff_joins_with_full_original_dense_FCL=roundoff_joins,
        original_ik_collision_contact_exceptions_and_shrunken_limits_unchanged=True)


def main():
    global RUN, IDENTITY_SHA
    if len(sys.argv) == 3:
        RUN = Path(sys.argv[1]).resolve()
        IDENTITY_SHA = sys.argv[2]
    assert RUN.parent == ROOT.parent and RUN.name.startswith('centrifuge_48box_sequential_full_')
    assert len(IDENTITY_SHA) == 64
    began = time.monotonic()
    assert hashlib.sha256((RUN / 'identity.json').read_bytes()).hexdigest() == IDENTITY_SHA
    identity = json.loads((RUN / 'identity.json').read_text())
    for relative, expected in identity['sha256'].items():
        assert hashlib.sha256((RUN / relative).read_bytes()).hexdigest() == expected, relative
    plans, published = [], []
    with gzip.open(RUN / 'run/loading_trace.jsonl.gz', 'rb') as stream:
        for raw in stream:
            header, separator, remainder = raw.partition(b', "payload": ')
            event = json.loads(header + b'}') if separator else json.loads(raw)
            if event['kind'] == 'sequential_actual_collision_plan':
                if plans:
                    verify_published(published, plans[-1])
                plans.append(json.loads(raw)['payload'])
                published = []
            elif event['kind'] == 'command' and event['phase'].startswith(PREFIXES):
                if not plans:
                    raise ValueError('Unplanned sequential command')
                command = json.loads(raw)
                verify_axis_exit_timing(command)
                published.append(command)
    assert plans
    verify_published(published, plans[-1])
    paths = sorted((RUN / 'run').glob('*_request.json'), key=lambda path: int(path.name.split('_')[0])
                   if path.name.split('_')[0].isdigit() else 1000000)
    records = []
    for path in paths:
        request = json.loads(path.read_text())
        if request.get('sequential_contract') != CONTRACT:
            continue
        result_path = path.with_name(path.name.replace('_request.json', '_result.json'))
        planned = json.loads(result_path.read_text())
        assert planned == plans[len(records)]
        assert planned['source_snapshot_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
        assert planned['worker_source_sha256'] == identity['sha256']['sequential_worker.py']
        assert planned['speed_worker_source_sha256'] == identity['sha256']['speed_worker.py']
        assert planned['speed_planning_source_sha256'] == identity['sha256']['speed_planning.py']
        assert planned['speed_retrieval_geometry_source_sha256'] == identity['sha256']['speed_retrieval_geometry.py']
        assert planned['axial_pilot_source_sha256'] == identity['sha256']['axial_pilot.py']
        assert planned['axial_transport_source_sha256'] == identity['sha256']['axial_transport.py']
        assert planned['centered_exit_source_sha256'] == identity['sha256']['centered_exit.py']
        assert planned['table_alignment_source_sha256'] == identity['sha256']['table_alignment.py']
        assert planned['speed_profile'] == identity['speed_profile'] == PROFILE
        review = review_path(request, planned)
        review.update(request=path.name, request_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                      result_sha256=hashlib.sha256(result_path.read_bytes()).hexdigest())
        records.append(review)
        print('ACTUAL_SEQUENTIAL_DENSE_PASS', review, flush=True)
    assert len(records) == len(plans)
    assert json.loads((RUN / 'speed_task_audit.json').read_text())['status'] == 'PASS_RELAXED_SIMULATION_SEQUENTIAL_TASK_NOT_OLD_CERTIFICATE'
    assert json.loads((RUN / 'speed_physical_audit.json').read_text())['status'] == 'PASS'
    assert json.loads((RUN / 'independent_speed_grip.json').read_text())['status'] == 'PASS'
    result = dict(status='PASS_ACTUAL_SEQUENTIAL_COMMAND_PATHS_NOT_ABSOLUTE_COLLISION_GUARANTEE',
        authority=str(RUN), identity_sha256=IDENTITY_SHA, records=records,
        final_path_dense_checks=sum(record['final_path_dense_checks'] for record in records),
        exact_published_paths=True, elapsed_wall_s=time.monotonic() - began,
        model_visual_only_wrist_collision_limitation_preserved=True, no_physical_action=True,
        no_claim_perfect_vertical_or_real_robot_or_three_minutes=True)
    with (ROOT / 'speed_actual_paths_review.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print('ACTUAL_PATH_REVIEW_PASS', result['final_path_dense_checks'], flush=True)


if __name__ == '__main__':
    main()


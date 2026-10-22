import copy
import math

import numpy as np
from scipy.spatial.transform import Rotation

from sequential_geometry import SinglePlanner, pose_matrix
from sequential_planning import result


def release_target(poses, payload):
    if payload not in ('left', 'right'):
        raise ValueError('Explicit single payload required')
    bucket = pose_matrix(poses['centrifuge']) @ pose_matrix(poses['bucket_' + payload])
    target = np.eye(4)
    target[:3, :3] = Rotation.from_euler('z', .309270119).as_matrix()
    target[:3, 3] = [bucket[0, 3], bucket[1, 3], 1.07525]
    if not np.isfinite(target).all():
        raise ValueError('Finite actual basket target required')
    return target


def release_error(poses, payload):
    body = pose_matrix(poses['box48_' + payload])
    target = release_target(poses, payload)
    if not np.isfinite(body).all():
        raise ValueError('Finite actual body pose required')
    position = float(np.linalg.norm(body[:3, 3] - target[:3, 3]))
    angle = math.degrees(Rotation.from_matrix(target[:3, :3].T @ body[:3, :3]).magnitude())
    return dict(position_m=position, angle_deg=angle,
                within_alignment_goal=position <= .00015 and angle <= .08,
                within_local_correction_bound=position <= .001 and angle <= 1.)


def plan_release_alignment(request, reference):
    metrics = release_error(request['poses'], request['payload'])
    if not metrics['within_local_correction_bound']:
        raise ValueError('Release correction exceeds the local one-mm/one-degree envelope')
    planner = SinglePlanner(request, reference)
    seed = {name: request['positions'][name] for name in planner.names}
    body = pose_matrix(request['poses']['box48_' + request['payload']])
    planner.attach(body, seed)
    target = release_target(request['poses'], request['payload'])
    stage = planner.stage(body, target, seed, 'load_release_pilot_align', count=21, refine=False)
    maximum_change = max(abs(sample[name] - seed[name]) for sample in stage['samples'] for name in seed)
    if maximum_change > .045:
        raise ValueError('Release correction left the local joint branch')
    return dict(result(planner, [stage], reference), release_alignment_before=metrics,
                maximum_local_joint_change_rad=maximum_change,
                original_release_gate_unchanged=True)


def align_before_release(cycle, hand, payload):
    original_holders = copy.deepcopy(cycle.holders.snapshot())
    if set(original_holders) != {hand} or original_holders[hand]['payload'] != payload:
        raise ValueError('One matching original actual holder required before release alignment')
    for attempt in range(3):
        cycle.safety()
        if cycle.holders.snapshot() != original_holders:
            raise RuntimeError('Release alignment cannot replace original holder anchors')
        metrics = release_error(cycle.poses, payload)
        cycle.record('sequential_release_alignment_measured', dict(hand=hand, payload=payload,
                     corrections_completed=attempt, metrics=metrics, original_release_gate_unchanged=True))
        if metrics['within_alignment_goal'] or attempt == 2:
            return
        if not metrics['within_local_correction_bound']:
            raise RuntimeError('Release correction exceeds the local one-mm/one-degree envelope')
        cycle.execute_stages(cycle.single_plan('load_release_align', hand, payload), 'seq_load')
        cycle.dwell(.3)

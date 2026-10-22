import copy
import math

import numpy as np
from scipy.spatial.transform import Rotation

from release_alignment import release_error, release_target
from sequential_geometry import SinglePlanner, pose_matrix
from sequential_planning import result


def correction_target(poses, payload):
    metrics = release_error(poses, payload)
    if metrics['position_m'] > .001 or metrics['angle_deg'] > 2.:
        raise ValueError('Release correction exceeds bounded two-step envelope')
    body = pose_matrix(poses['box48_' + payload])
    target = release_target(poses, payload)
    difference = Rotation.from_matrix(body[:3, :3].T @ target[:3, :3])
    if metrics['angle_deg'] > 1.:
        target[:3, :3] = body[:3, :3] @ Rotation.from_rotvec(
            difference.as_rotvec() * (.9 / metrics['angle_deg'])).as_matrix()
    step = math.degrees(Rotation.from_matrix(body[:3, :3].T @ target[:3, :3]).magnitude())
    if step > 1. + 1e-10:
        raise ValueError('Per-command angular correction exceeds original one degree')
    return metrics, target, step


def plan_release_alignment(request, reference):
    metrics, target, step = correction_target(request['poses'], request['payload'])
    planner = SinglePlanner(request, reference)
    seed = {name: request['positions'][name] for name in planner.names}
    body = pose_matrix(request['poses']['box48_' + request['payload']])
    planner.attach(body, seed)
    stage = planner.stage(body, target, seed, 'load_release_pilot_align', count=21, refine=False)
    maximum_change = max(abs(sample[name] - seed[name]) for sample in stage['samples'] for name in seed)
    if maximum_change > .045:
        raise ValueError('Release correction left the local joint branch')
    return dict(result(planner, [stage], reference), release_alignment_before=metrics,
                correction_step_deg=step, maximum_local_joint_change_rad=maximum_change,
                original_release_gate_unchanged=True, at_most_two_measured_corrections=True)


def align_before_release(cycle, hand, payload):
    holders = copy.deepcopy(cycle.holders.snapshot())
    if set(holders) != {hand} or holders[hand]['payload'] != payload:
        raise ValueError('One matching original actual holder required')
    for attempt in range(3):
        cycle.safety()
        if cycle.holders.snapshot() != holders:
            raise RuntimeError('Release correction cannot replace original holder anchors')
        metrics, target, step = correction_target(cycle.poses, payload)
        cycle.record('sequential_release_alignment_measured', dict(hand=hand, payload=payload,
                     corrections_completed=attempt, metrics=metrics, next_step_deg=step,
                     maximum_observed_error_deg=2., original_release_gate_unchanged=True))
        if metrics['within_alignment_goal']:
            return
        if attempt == 2:
            raise RuntimeError('Release alignment did not converge in two measured corrections')
        cycle.execute_stages(cycle.single_plan('load_release_align', hand, payload), 'seq_load')
        cycle.dwell(.3)

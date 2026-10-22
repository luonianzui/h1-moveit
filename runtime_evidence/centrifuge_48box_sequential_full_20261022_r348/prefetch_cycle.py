import copy
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from dual_motion import TASK_JOINTS
from sequential_geometry import CONTRACT, pose_matrix
from speed_policy import PROFILE


ROOT = Path(__file__).resolve().parent


def next_stationary_plan(phase, seconds, holders):
    if len(holders) != 1:
        return None
    hand = next(iter(holders))
    payload = holders[hand]['payload']
    if phase == hand + '_seq_load_static_grip_hold' and seconds == 10:
        return ('load_transport', hand, payload)
    if phase == 'seq_retrieval_single_pilot' and seconds == 1.:
        return ('retrieval_transport', hand, payload)
    return None


def snapshot_drift(request, positions, poses):
    joints = max(abs(positions[name] - request['positions'][name]) for name in TASK_JOINTS)
    translation = max(float(np.linalg.norm(pose_matrix(poses['box48_' + side])[:3, 3] -
        pose_matrix(request['poses']['box48_' + side])[:3, 3])) for side in ('left', 'right'))
    angle = max(math.degrees(Rotation.from_matrix(
        pose_matrix(request['poses']['box48_' + side])[:3, :3].T @
        pose_matrix(poses['box48_' + side])[:3, :3]).magnitude()) for side in ('left', 'right'))
    fingers = max(abs(positions[name] - request['positions'][name])
        for name in request['positions'] if 'dex1_finger' in name)
    return dict(joint_drift_rad=joints, pose_drift_m=translation,
        angle_drift_deg=angle, finger_drift_m=fingers)


def drift_within_bounds(values):
    pairs = (('joint_drift_rad', 'snapshot_joint_rad'), ('pose_drift_m', 'snapshot_pose_m'),
             ('angle_drift_deg', 'snapshot_angle_deg'), ('finger_drift_m', 'snapshot_finger_m'))
    return all(math.isfinite(values[name]) and values[name] <= PROFILE[limit] for name, limit in pairs)


class PlanningPrefetchMixin:
    def dwell(self, seconds):
        if not hasattr(self, 'planning_worker'):
            return super().dwell(seconds)
        candidate = next_stationary_plan(self.phase, seconds, self.holders.snapshot())
        if candidate is not None:
            if getattr(self, 'prefetch_pending', None) is not None:
                raise RuntimeError('An unconsumed speculative plan already exists')
            self.begin_stationary_plan(*candidate)
        return super().dwell(seconds)

    def begin_stationary_plan(self, mode, hand, payload):
        self.safety()
        pair = self.measured_pair()
        positions, poses = dict(pair['joints']['positions']), copy.deepcopy(pair['poses'])
        self.state.joint_positions = positions
        self.state.update()
        reference = np.linalg.inv(self.state.get_global_link_transform(hand + '_wrist_yaw_link')) @ pose_matrix(poses['box48_' + payload])
        request = dict(mode=mode, hand=hand, payload=payload, sequential_contract=CONTRACT,
            positions=positions, poses=poses, source_sim=pair['pose_stamp'], planning_reference=reference.tolist(),
            original_runtime_holder_references=self.holders.snapshot(), runtime_reference_not_rebased=True)
        self.worker_sequence += 1
        prefix = str(self.worker_sequence) + '_' + mode + '_' + hand
        source, output = [ROOT / 'run' / (prefix + suffix) for suffix in ('_request.json', '_result.json')]
        with source.open('x') as stream:
            json.dump(request, stream, indent=2, allow_nan=False)
        self.record('sequential_actual_planning_started', dict(mode=mode, hand=hand, payload=payload,
            request=str(source), source_sim=pair['pose_stamp'], prefetch_during_existing_stationary_observation=True))
        ticket = self.planning_worker.submit(source, output)
        self.prefetch_pending = dict(key=(mode, hand, payload), request=request, ticket=ticket,
            prefix=prefix, output=output, submitted_wall=time.monotonic(), submitted_sim=self.sim,
            original_observation_phase=self.phase)

    def single_plan(self, mode, hand, payload):
        pending = getattr(self, 'prefetch_pending', None)
        if pending is None:
            return super().single_plan(mode, hand, payload)
        if pending['key'] != (mode, hand, payload):
            raise RuntimeError('Stationary prefetch cannot be used by a different phase or hand')
        self.safety()
        waited = time.monotonic()
        receipt = self.planning_worker.finish(pending['ticket'], self.spin)
        self.safety()
        planned = json.loads(pending['output'].read_text())
        if (planned.get('speed_profile') != PROFILE or planned.get('status') != 'PASS_GEOMETRY_ONLY'
                or planned.get('maximum_joint_step_rad') != .002):
            raise RuntimeError('Prefetched result did not pass the unchanged geometry contract')
        if planned['original_runtime_holder_references'] != self.holders.snapshot():
            raise RuntimeError('Original holder anchors changed during stationary prefetch')
        if planned['source_snapshot_sha256'] != receipt['source_sha256']:
            raise RuntimeError('Prefetched geometry result has a different actual snapshot')
        drift = snapshot_drift(pending['request'], self.positions, self.poses)
        self.record('sequential_actual_snapshot_checked', dict(mode=mode, **drift))
        if not drift_within_bounds(drift):
            self.record('stationary_prefetch_drift_rejected', dict(mode=mode, **drift,
                no_motion_dispatched=True, original_bounds_unchanged=True))
            raise RuntimeError('Stationary prefetched snapshot drift exceeded unchanged bounds')
        with (ROOT / 'run' / (pending['prefix'] + '_worker.log')).open('x') as stream:
            json.dump(dict(reused_planner_receipt=receipt), stream)
        self.record('stationary_prefetch_consumed', dict(mode=mode, hand=hand, payload=payload,
            submitted_sim=pending['submitted_sim'], consumed_sim=self.sim,
            elapsed_wall_s=time.monotonic() - pending['submitted_wall'],
            remaining_wait_wall_s=time.monotonic() - waited,
            original_observation_phase=pending['original_observation_phase'],
            original_holder_and_snapshot_guards_passed=True))
        self.prefetch_pending = None
        self.record('sequential_actual_collision_plan', planned)
        return planned

import copy
import math
import time

import numpy as np
import rclpy

from endpoint_overlap_cycle import EndpointOverlapCycle
from execute_loading import pose_matrix
from fast_profile import RETRIEVAL_SOFT_GRASP_SECONDS
from loaded_retention import retention_metrics
from sequential_force import RETRIEVAL_FORCE_N


MAXIMUM_WAIT_WALL_S = .020
MAXIMUM_WAIT_SIM_S = .001
MAXIMUM_CALLBACKS = 64


def pending_zero_force_frame(cycle):
    hand = getattr(cycle, 'current_grasp_hand', None)
    payload = getattr(cycle, 'grasp_candidate', None)
    if (not getattr(cycle, 'remaining_guard_enabled', False) or hand not in ('left', 'right')
            or payload not in ('left', 'right') or cycle.phase != hand + '_retrieval_grasp'
            or not getattr(cycle, 'retrieval_closing', False)
            or cycle.holders.rows or cycle.order.locations.get(payload) != 'basket'):
        return None
    claim = getattr(cycle, 'preload_grasp_claim', {}) or {}
    handle = getattr(cycle, 'pair_goal_handles', {}).get(hand)
    if (handle is None or not handle.accepted or claim.get('accepted') is not True
            or claim.get('hand') != hand or claim.get('payload') != payload
            or cycle.accepted_gripper_targets.get(hand) != -.01901
            or cycle.accepted_gripper_efforts.get(hand) != RETRIEVAL_FORCE_N):
        return None
    pairs = cycle.contacts[payload]
    if not pairs or any(pair.get('force') != 0. for pair in pairs):
        return None
    entry = cycle.closing_floor_history.rows.get(payload, {})
    values = [cycle.sim, cycle.joint_time, entry.get('stamp'), entry.get('positive'),
              claim.get('accepted_common_start_sim')]
    if any(not isinstance(value, (int, float)) or not math.isfinite(value) for value in values):
        return None
    now, joint_stamp, stamp, positive, start = values
    if not (0 <= stamp-positive <= .0005 + 1e-12 and 0 <= now-stamp <= .001
            and 0 <= now-joint_stamp <= .012 and 0 <= now-start <= RETRIEVAL_SOFT_GRASP_SECONDS + .05):
        return None
    native = [cycle.positions.get(hand + f'_dex1_finger_{number}_joint', math.nan) for number in (1, 2)]
    if not all(math.isfinite(value) for value in native):
        return None
    metrics = retention_metrics(cycle.poses, [], payload)
    if not (metrics['sidewall_certified'] and metrics['vertical_inside']
            and metrics['side_overhang_m'] <= .000005
            and abs(metrics['corners_min'][2] + .03) <= .000005):
        return None
    inverse = np.linalg.inv(pose_matrix(cycle.poses['box48_' + payload]))
    fingers = set()
    for pair in pairs:
        names = (pair['first'], pair['second'])
        box = 'box48_' + payload + '::'
        number = next((number for number in (1, 2)
                       if any(name.startswith('h1_2::' + hand + f'_dex1_finger_{number}_') for name in names)), None)
        if (number is None or not any(name.startswith(box) for name in names)
                or not math.isfinite(pair['depth']) or not 0 <= pair['depth'] <= .001
                or not pair['positions']):
            return None
        local = np.asarray(pair['positions']) @ inverse[:3, :3].T + inverse[:3, 3]
        if (not np.isfinite(local).all() or np.max(np.abs(local[:, 0])) > .018
                or abs(local[:, 0].mean()) > .004
                or np.max(np.abs(np.abs(local[:, 1]) - .044070050215)) > .001
                or np.max(np.abs(local[:, 2])) > .030):
            return None
        fingers.add(number)
    return dict(hand=hand, payload=payload, sim=now, contact_stamp=stamp,
                last_positive_basket_stamp=positive, joint_stamp=joint_stamp,
                observed_fingers=sorted(fingers), claim=copy.deepcopy(claim),
                metrics=metrics, contacts=copy.deepcopy(pairs), poses=copy.deepcopy(cycle.poses),
                not_a_support_or_grip_certificate=True, no_new_arm_or_gripper_command=True)


def wait_for_observation(cycle, initial, pump, core_guard, clock=time.monotonic):
    began = clock()
    callbacks = 0
    while pending_zero_force_frame(cycle) is not None:
        core_guard()
        if clock()-began >= MAXIMUM_WAIT_WALL_S or callbacks >= MAXIMUM_CALLBACKS:
            break
        pump()
        callbacks += 1
        if not 0 <= cycle.sim-initial['sim'] <= MAXIMUM_WAIT_SIM_S + 1e-12:
            raise RuntimeError('Preload observation exceeded bounded simulation window')
        stamp = cycle.closing_floor_history.rows.get(initial['payload'], {}).get('stamp', math.nan)
        if not math.isfinite(stamp) or not 0 <= stamp-initial['contact_stamp'] <= MAXIMUM_WAIT_SIM_S + 1e-12:
            raise RuntimeError('Preload observation skipped beyond the next bounded contact frame')
    return dict(callbacks=callbacks, elapsed_wall_s=clock()-began,
                elapsed_sim_s=cycle.sim-initial['sim'],
                still_missing=pending_zero_force_frame(cycle) is not None,
                no_synthetic_contact_or_holder_created=True)


class PreloadObservationMixin:
    def safety(self):
        initial = pending_zero_force_frame(self)
        if initial is None:
            return super().safety()
        self.record('preload_observation_wait_started', initial)

        def pump():
            rclpy.spin_once(self.node, timeout_sec=0)
            self.refresh_native()

        outcome = wait_for_observation(self, initial, pump, lambda: EndpointOverlapCycle.safety(self))
        super().safety()
        self.record('preload_observation_wait_pass', dict(
            initial=initial, outcome=outcome, sim=self.sim,
            contacts=copy.deepcopy(self.contacts[initial['payload']]),
            history=copy.deepcopy(self.closing_floor_history.rows[initial['payload']]),
            poses=copy.deepcopy(self.poses), holders=self.holders.snapshot(),
            original_sequential_safety_passed_after_fresh_callbacks=True,
            not_a_support_or_grip_certificate=True))

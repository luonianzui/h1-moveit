import math

import numpy as np

from execute_loading import pose_matrix
from fast_profile import RETRIEVAL_SOFT_GRASP_SECONDS
from sequential_force import RETRIEVAL_FORCE_N


class FloorContactHistory:
    def __init__(self):
        self.rows = {}

    def observe(self, payload):
        side, stamp = payload.get('side'), payload.get('stamp')
        if side not in ('left', 'right') or not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp < 0:
            return
        previous = self.rows.get(side, {})
        if stamp < previous.get('stamp', stamp):
            self.rows[side] = dict(stamp=stamp, positive=None)
            return
        positive = previous.get('positive')
        for pair in payload['pairs']:
            names = (pair['first'], pair['second'])
            box, bucket = 'box48_' + side + '::', 'centrifuge::bucket_' + side + '::'
            matching = ((names[0].startswith(box) and names[1].startswith(bucket)) or
                        (names[1].startswith(box) and names[0].startswith(bucket)))
            if (matching and math.isfinite(pair['force']) and 0 < pair['force'] <= 40 and
                    math.isfinite(pair['depth']) and 0 <= pair['depth'] <= .001):
                positive = stamp
        self.rows[side] = dict(stamp=stamp, positive=positive)


def single_missing_floor_frame(cycle, payload, metrics):
    failure = dict(valid=False, not_a_grip_certificate=True)
    hand = getattr(cycle, 'current_grasp_hand', None)
    claim = getattr(cycle, 'preload_grasp_claim', {}) or {}
    history = getattr(cycle, 'closing_floor_history', None)
    handle = getattr(cycle, 'pair_goal_handles', {}).get(hand)
    if (hand not in ('left', 'right') or getattr(cycle, 'grasp_candidate', None) != payload or
            cycle.phase != hand + '_retrieval_grasp' or not getattr(cycle, 'retrieval_closing', False) or
            getattr(cycle.holders, 'rows', {}) or handle is None or not handle.accepted or
            claim.get('accepted') is not True or claim.get('hand') != hand or claim.get('payload') != payload or
            cycle.accepted_gripper_targets.get(hand) != -.01901 or
            cycle.accepted_gripper_efforts.get(hand) != RETRIEVAL_FORCE_N or history is None):
        return failure
    entry = history.rows.get(payload, {})
    values = [cycle.sim, cycle.joint_time, entry.get('stamp'), entry.get('positive'), claim.get('accepted_common_start_sim')]
    if any(not isinstance(value, (int, float)) or not math.isfinite(value) for value in values):
        return failure
    now, joint_stamp, contact_stamp, positive_stamp, start = values
    gap = contact_stamp - positive_stamp
    if not (0 <= gap <= .0005 + 1e-12 and 0 <= round(now * 1e9) - round(contact_stamp * 1e9) <= 1_000_000 and
            0 <= now-joint_stamp <= .012 and 0 <= now-start <= RETRIEVAL_SOFT_GRASP_SECONDS + .05):
        return failure
    native = [cycle.positions.get(hand + f'_dex1_finger_{number}_joint', math.nan) for number in (1, 2)]
    if not all(math.isfinite(value) for value in native):
        return failure
    if not (metrics['sidewall_certified'] and metrics['vertical_inside'] and
            math.isfinite(metrics['side_overhang_m']) and metrics['side_overhang_m'] <= .000005 and
            abs(metrics['corners_min'][2] + .03) <= .000005):
        return failure
    pairs = cycle.contacts[payload]
    inverse = np.linalg.inv(pose_matrix(cycle.poses['box48_' + payload]))
    fingers = set()
    for pair in pairs:
        names = pair['first'] + ' ' + pair['second']
        number = next((number for number in (1, 2) if hand + f'_dex1_finger_{number}_' in names), None)
        if (number is None or 'box48_' + payload + '::' not in names or
                not math.isfinite(pair['force']) or not 0 < pair['force'] <= 40 or
                not math.isfinite(pair['depth']) or not 0 <= pair['depth'] <= .001 or not pair['positions']):
            return failure
        local = np.asarray(pair['positions']) @ inverse[:3, :3].T + inverse[:3, 3]
        if (not np.isfinite(local).all() or np.max(np.abs(local[:, 0])) > .018 or
                abs(local[:, 0].mean()) > .004 or
                np.max(np.abs(np.abs(local[:, 1]) - .044070050215)) > .001 or
                np.max(np.abs(local[:, 2])) > .030):
            return failure
        fingers.add(number)
    if not fingers:
        return failure
    return dict(valid=True, not_a_grip_certificate=True, source='one_actual_contact_frame_floor_history',
                current_contact_stamp=contact_stamp, last_actual_positive_basket_stamp=positive_stamp,
                missing_floor_sim_s=gap, observed_fingers=sorted(fingers),
                maximum_native_distance_to_final_goal_m=max(abs(value + .01901) for value in native),
                final_endpoint_and_two_opposed_strong_capture_still_required=True,
                no_holder_created_or_anchor_rebased=True)

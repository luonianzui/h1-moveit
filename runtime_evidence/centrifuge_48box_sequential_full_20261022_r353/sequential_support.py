import math

import numpy as np

from execute_loading import midband_contacts
from sequential_force import RETRIEVAL_FORCE_N


def preload_native_status(cycle, payload, retrieval=False):
    if getattr(getattr(cycle, 'holders', None), 'rows', {}):
        return dict(valid=False, reason='another_actual_holder')
    hand = getattr(cycle, 'current_grasp_hand', None)
    claim = getattr(cycle, 'preload_grasp_claim', None)
    expected_phase = str(hand) + ('_retrieval_grasp' if retrieval else '_seq_load_grasp')
    expected_effort = RETRIEVAL_FORCE_N if retrieval else 16.
    declared = (hand in ('left', 'right') and getattr(cycle, 'grasp_candidate', None) == payload
        and cycle.phase == expected_phase and isinstance(claim, dict)
        and claim.get('accepted') is True and claim.get('hand') == hand and claim.get('payload') == payload
        and cycle.accepted_gripper_targets.get(hand) == -.01901
        and cycle.accepted_gripper_efforts.get(hand) == expected_effort)
    if not declared:
        return dict(valid=False, reason='no_accepted_current_grasp')
    stamp = cycle.joint_time
    finite_clock = (cycle.sim is not None and math.isfinite(cycle.sim) and stamp is not None and math.isfinite(stamp))
    age = abs(cycle.sim - stamp) if finite_clock else float('inf')
    names = [hand + f'_dex1_finger_{number}_joint' for number in (1, 2)]
    errors = [abs(cycle.positions.get(name, float('inf')) + .01901) for name in names]
    if not all(math.isfinite(error) and error < .00025 for error in errors) or age > .012:
        return dict(valid=False, reason='native_endpoint_or_age', native_joint_sim=stamp,
                    maximum_native_error_m=max(errors), native_age_sim_s=age)
    return dict(valid=True, native_joint_sim=stamp, maximum_native_error_m=max(errors), native_age_sim_s=age)


def actual_preload_table_continuity(cycle, payload):
    from terminal_policy import table_support_valid
    native = preload_native_status(cycle, payload)
    if not native['valid']:
        return native
    pairs = cycle.contacts[payload]
    own = cycle.current_grasp_hand + '_dex1_finger_'
    table_present = False
    for pair in pairs:
        names = pair['first'] + ' ' + pair['second']
        if ('box48_' + payload + '::' not in names or not ('table::' in names or own in names)
                or not math.isfinite(pair['depth']) or not 0 <= pair['depth'] <= .001
                or not math.isfinite(pair['force']) or not 0 <= pair['force'] <= 40):
            return dict(valid=False, reason='invalid_actual_contact')
        table_present = table_present or 'table::' in names
    continuity = table_support_valid(dict(geometry_valid=True, contacts_valid=table_present),
        cycle.sim, cycle.table_positive.get(payload))
    return dict(valid=continuity['valid'], source='original_table_force_continuity',
        native=native, table_contact_still_present=table_present, continuity=continuity,
        original_birth_pose_and_all_raw_contact_guards_still_required=True,
        no_holder_or_runtime_anchor_created_or_rebased=True)


def actual_preload_support(cycle, payload, retrieval=False):
    native = preload_native_status(cycle, payload, retrieval=retrieval)
    if not native['valid']:
        return native
    hand = cycle.current_grasp_hand
    pairs = cycle.contacts[payload]
    support_prefix = 'centrifuge::bucket_' + payload + '::' if retrieval else 'table::'
    for pair in pairs:
        names = pair['first'] + ' ' + pair['second']
        if ('box48_' + payload + '::' not in names or not
                (support_prefix in names or hand + '_dex1_finger_' in names)):
            return dict(valid=False, reason='foreign_contact')
    alias = dict(cycle.poses, **{'box48_' + hand: cycle.poses['box48_' + payload]})
    try:
        band = midband_contacts(alias, pairs, hand)
        loads = {str(number): sum(abs(float(np.dot(force, normal))) for pair in pairs
            if hand + f'_dex1_finger_{number}_' in pair['first'] + pair['second']
            for force, normal in zip(pair['force_vectors'], pair['normals'])) for number in (1, 2)}
    except (ValueError, RuntimeError):
        return dict(valid=False, reason='missing_actual_opposed_midband')
    valid = all(math.isfinite(value) and 3 <= value <= 20.5 for value in loads.values())
    return dict(valid=valid, source='gazebo_physics_contact', normal_loads_N=loads, fingers=band,
        native_joint_sim=native['native_joint_sim'], maximum_native_error_m=native['maximum_native_error_m'],
        native_age_sim_s=native['native_age_sim_s'],
        no_holder_or_runtime_anchor_created_or_rebased=True,
        original_birth_pose_and_all_raw_contact_guards_still_required=True)


def original_basket_preload_gap_valid(certificate, continuity):
    loads = certificate.get('normal_loads_N', {})
    fingers = certificate.get('fingers', {})
    gap = continuity.get('zero_force_gap_sim_s', float('inf'))
    age = certificate.get('native_age_sim_s', float('inf'))
    error = certificate.get('maximum_native_error_m', float('inf'))
    return bool(certificate.get('source') == 'gazebo_physics_contact' and
        set(loads) == set(fingers) == {'1', '2'} and
        all(math.isfinite(value) and 0 < value <= 20.5 for value in loads.values()) and
        math.isfinite(age) and 0 <= age <= .012 and math.isfinite(error) and error < .00025 and
        continuity.get('limit_sim_s') == .02 and math.isfinite(gap) and 0 <= gap <= .02)

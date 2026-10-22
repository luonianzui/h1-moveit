import math

import numpy as np
from execute_loading import pose_matrix
from table_return_goal import table_orientation_error


TERMINAL_POLICY = 'both_table_slots_verified_empty_open_grippers_hold_at_release_no_final_park'


def skip_final_parking(is_last_payload, phase):
    return is_last_payload and phase in ('retreat', 'return')


def table_terminal_metrics(poses, payload, pairs, expected_position, expected_yaw, reference_position):
    body = pose_matrix(poses['box48_' + payload])
    position_error = float(np.linalg.norm(body[:3, 3] - expected_position))
    drift = float(np.linalg.norm(body[:3, 3] - reference_position))
    angle_error = float(table_orientation_error(body, expected_yaw))
    supported = False
    contacts_valid = bool(pairs)
    no_finger_contact = True
    for pair in pairs:
        names = pair['first'] + ' ' + pair['second']
        matching = 'box48_' + payload + '::' in names and 'table::' in names
        finite = all(math.isfinite(pair.get(key, math.nan)) for key in ('depth', 'force'))
        supported = supported or bool(matching and finite and pair['force'] > 0)
        contacts_valid = contacts_valid and matching and finite and 0 <= pair['depth'] <= .001
        no_finger_contact = no_finger_contact and 'dex1' not in names
    geometry_valid = bool(np.isfinite(body).all() and position_error <= .002 and drift <= .0005 and
                          angle_error < math.radians(2) and no_finger_contact)
    return dict(geometry_valid=geometry_valid, contacts_valid=contacts_valid, supported=supported,
                no_finger_contact=no_finger_contact, position_error_m=position_error, drift_m=drift,
                orientation_error_deg=math.degrees(angle_error))


def table_support_valid(metrics, sim, last_positive_sim):
    gap = float('inf') if last_positive_sim is None else sim - last_positive_sim
    return dict(valid=bool(metrics['geometry_valid'] and metrics['contacts_valid'] and 0 <= gap <= .02),
                zero_force_gap_sim_s=gap, original_support_window_sim_s=.02)


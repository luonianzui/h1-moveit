import copy
import math
import time

import numpy as np

from sequential_geometry import SinglePlanner, pose_matrix
from sequential_planning import result
from table_return_goal import nearest_table_yaw, table_orientation_error


MODE = 'retrieval_table_alignment'
STAGE = 'table_alignment_pilot'


def alignment_metrics(poses, hand, payload):
    if hand not in ('left', 'right') or payload != hand:
        raise ValueError('Original same-side single return assignment required')
    body = pose_matrix(poses['box48_' + payload])
    goal = np.array([.44, .38 if hand == 'left' else -.38, .84475])
    error = goal - body[:3, 3]
    angle = math.degrees(table_orientation_error(body, nearest_table_yaw(body)))
    if not np.isfinite(body).all() or np.linalg.norm(error) > .0035 or abs(error[2]) > .0001 or angle > .25:
        raise ValueError('Table micro-alignment requires supported near-level bounded actual pose')
    return dict(position_error_m=float(np.linalg.norm(error)), orientation_error_deg=angle,
                height_error_m=float(error[2]), within_goal=bool(np.linalg.norm(error) <= .0015))


def correction_target(poses, hand, payload):
    metrics = alignment_metrics(poses, hand, payload)
    body = pose_matrix(poses['box48_' + payload])
    error = np.array([.44, .38 if hand == 'left' else -.38]) - body[:2, 3]
    distance = float(np.linalg.norm(error))
    target = body.copy()
    if distance > 0:
        target[:2, 3] += error * min(1., .001 / distance)
    return metrics, target


class TableAlignmentPlanner(SinglePlanner):
    def check_edge(self, first, last, phase, table_endpoint=False, initial_support=None):
        if phase != STAGE or initial_support is not None:
            raise ValueError('Only declared supported horizontal table micro-path allowed')
        return super().check_edge(first, last, phase, table_endpoint=True)


def plan_table_alignment(request, reference):
    metrics, target = correction_target(request['poses'], request['hand'], request['payload'])
    planner = TableAlignmentPlanner(request, reference)
    seed = {name: request['positions'][name] for name in planner.names}
    body = pose_matrix(request['poses']['box48_' + request['payload']])
    planner.attach(body, seed)
    stage = planner.stage(body, target, seed, STAGE, count=21, refine=False)
    change = max(abs(sample[name] - seed[name]) for sample in stage['samples'] for name in seed)
    if change > .045:
        raise ValueError('Table micro-alignment left original local joint branch')
    return dict(result(planner, [stage], reference), table_alignment_before=metrics,
                supported_horizontal_contact_pair='box48_' + request['payload'] + '/table_link_surface',
                maximum_horizontal_step_m=.001, actual_height_and_orientation_preserved=True,
                original_runtime_anchor_and_final_2mm_2deg_gate_unchanged=True,
                physical_table_support_still_required_at_dispatch=True)


def align_supported_return(cycle, hand, payload):
    holders = copy.deepcopy(cycle.holders.snapshot())
    if (set(holders) != {hand} or holders[hand]['payload'] != payload or
            cycle.order.phase != 'retrieval' or cycle.order.locations[payload] != 'held' or
            cycle.return_hands.get(payload) != hand or cycle.pair_clearance_payloads):
        raise ValueError('Original post-setdown single holder required')
    previous_error = None
    for attempt in range(3):
        cycle.safety()
        if cycle.holders.snapshot() != holders:
            raise RuntimeError('Original holder cannot change during table alignment')
        metrics = alignment_metrics(cycle.poses, hand, payload)
        pairs = [pair for pair in cycle.contacts[payload]
                 if 'table::' in pair['first'] + pair['second'] and pair.get('force', 0.) > 0.]
        if not pairs:
            raise RuntimeError('Actual positive table support required for micro-alignment')
        cycle.record('table_alignment_measured', dict(hand=hand, payload=payload, attempt=attempt,
            metrics=metrics, positive_table_pairs=copy.deepcopy(pairs), original_holders=holders,
            native_pose_sim=cycle.pose_time,
            contact_receive_age_wall_s=time.monotonic() - cycle.latest[payload + '_contacts']))
        if metrics['within_goal']:
            return
        if attempt == 2 or (previous_error is not None and metrics['position_error_m'] > previous_error + .0002):
            raise RuntimeError('Bounded table micro-alignment did not converge')
        previous_error = metrics['position_error_m']
        planned = cycle.single_plan(MODE, hand, payload)
        cycle.execute_stages(planned, 'seq_retrieval')
        cycle.dwell(.15)

import copy
import json
from pathlib import Path
from types import SimpleNamespace

from loaded_retention import retention_metrics
from preload_floor_frame import FloorContactHistory, single_missing_floor_frame
from preload_observation import pending_zero_force_frame
from sequential_support import actual_preload_support
from trace_io import open_trace


ROOT = Path(__file__).resolve().parent
history = FloorContactHistory()
latest_contacts = {}
latest_joint = None
goals = {}
claim = None
active = None
completed = []
failures = []


def snapshot(data, sim):
    hand, payload = data['hand'], data['payload']
    goal = goals[hand]
    return SimpleNamespace(remaining_guard_enabled=True, current_grasp_hand=hand, grasp_candidate=payload,
        phase=hand + '_retrieval_grasp', holders=SimpleNamespace(rows={}),
        order=SimpleNamespace(locations={payload: 'basket'}), retrieval_closing=True,
        pair_goal_handles={hand: SimpleNamespace(accepted=True)}, preload_grasp_claim=claim,
        accepted_gripper_targets={hand: goal['target']},
        accepted_gripper_efforts={hand: goal['feedforward_N']},
        sim=sim, joint_time=latest_joint['source_sim'], positions=latest_joint['positions'],
        poses=data['poses'], contacts=copy.deepcopy(latest_contacts), closing_floor_history=copy.deepcopy(history))


with open_trace(ROOT) as stream:
    for line in stream:
        row = json.loads(line)
        kind, data = row['kind'], row['payload']
        if active is not None and kind in ('command', 'button_arm_command', 'gripper_goal_accepted',
                                          'dual_holder_begin', 'coherent_grip_reference_frozen'):
            raise AssertionError('New motion or capture during uncertified observation')
        if kind == 'contact':
            history.observe(data)
            latest_contacts[data['side']] = data['pairs']
        elif kind == 'actual_full_joint_frame':
            latest_joint = data
        elif kind == 'gripper_goal_accepted':
            goals[data['side']] = data
        elif kind == 'sequential_preload_claim':
            claim = data
        elif kind == 'preload_observation_wait_started':
            assert active is None
            assert data['contacts'] == latest_contacts[data['payload']]
            cycle = snapshot(data, row['sim'])
            classified = pending_zero_force_frame(cycle)
            assert classified is not None
            assert classified['last_positive_basket_stamp'] == data['last_positive_basket_stamp']
            active = data
        elif kind == 'preload_observation_wait_pass':
            assert active is not None and data['initial'] == active
            assert not data['holders'] and data['not_a_support_or_grip_certificate']
            assert data['original_sequential_safety_passed_after_fresh_callbacks']
            payload = active['payload']
            assert data['contacts'] == latest_contacts[payload]
            assert data['history'] == history.rows[payload]
            assert 0 <= row['sim'] - active['sim'] <= .001 + 1e-12
            assert 0 <= data['history']['stamp'] - active['contact_stamp'] <= .001 + 1e-12
            assert 0 <= data['outcome']['callbacks'] <= 64
            cycle = snapshot(dict(active, poses=data['poses']), row['sim'])
            support_pairs = [pair for pair in data['contacts']
                             if 'dex1_finger_' not in pair['first'] + pair['second']]
            metrics = retention_metrics(data['poses'], support_pairs, payload)
            strong = actual_preload_support(cycle, payload, retrieval=True)
            original_frame = single_missing_floor_frame(cycle, payload, metrics)
            assert metrics['valid'] or strong['valid'] or original_frame['valid']
            completed.append(dict(sim=row['sim'], outcome=data['outcome'],
                                  positive_basket_support=metrics['valid'],
                                  strong_preload_support=strong['valid'],
                                  original_single_frame=original_frame['valid'],
                                  no_synthetic_force_or_actual_holder=True))
            active = None
        if kind == 'FAIL':
            failures.append(row)
result = dict(status='PASS' if active is None and not failures else 'FAIL',
              scope='independent_original_trace_preload_wait_not_whole_task_certificate',
              completed_waits=completed, wait_exercised_in_actual_run=bool(completed),
              unresolved_wait=active, task_failure_events=len(failures),
              original_positive_support_recomputed=True,
              no_zero_force_grip_certificate_created=True)
with (ROOT / 'preload_observation_audit.json').open('x') as output:
    json.dump(result, output, indent=2)
print(result['status'], 'OBSERVATION_WAITS', len(completed), flush=True)
assert result['status'] == 'PASS'

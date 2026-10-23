from collections import Counter
import json
from pathlib import Path

import numpy as np
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState

from execute_loading import velocity_violation, pose_matrix
from trace_io import open_trace


ROOT = Path(__file__).resolve().parent
state = RobotState(RobotModel(str(ROOT / 'inputs/robot_dual.urdf'),
                   str(ROOT.parents[1] / 'src/h1_2_moveit_config/config/h1_2.srdf')))
counts = Counter()
holders, anchors, loads, returns = {}, {}, [], []
maximum_force = maximum_velocity = 0.
faults = []
first_sim = first_wall = None
last_sim = last_wall = None
with open_trace(ROOT) as stream:
    for line in stream:
        event = json.loads(line)
        kind, data = event['kind'], event['payload']
        counts[kind] += 1
        if first_sim is None and event.get('sim') is not None:
            first_sim, first_wall = event['sim'], event['wall']
        if kind == 'dual_holder_begin':
            assert not holders and data['hand'] not in holders
            state.joint_positions = data['actual_joint_pair']['positions']
            state.update()
            body = pose_matrix(data['paired_poses']['box48_' + data['payload']])
            wrist = state.get_global_link_transform(data['hand'] + '_wrist_yaw_link')
            native_wrist = pose_matrix(data['paired_poses'][data['hand'] + '_wrist_yaw_link'])
            assert np.allclose(np.linalg.inv(wrist) @ body, data['frozen']['fk_reference'], atol=1e-10, rtol=0)
            assert np.allclose(np.linalg.inv(native_wrist) @ body, data['frozen']['native_reference'], atol=1e-10, rtol=0)
            holders[data['hand']] = data['frozen']
            anchors[(data['hand'], data['payload'], data['frozen']['source_sim'])] = data['frozen']
        elif kind == 'dual_holder_normal_release':
            assert set(holders) == {data['hand']} and holders[data['hand']]['payload'] == data['payload']
            holders.clear()
        elif kind == 'command':
            assert data['holders'] == holders and len(holders) <= 1
        elif kind == 'sequential_actual_collision_plan':
            assert data['original_runtime_holder_references'] == holders
        elif kind == 'static_grip_hold_pass':
            assert data['duration_sim_s'] >= 10
            loads.append(data['payload'])
        elif kind == 'retrieval_static_grip_hold_pass':
            assert data['duration_sim_s'] >= 10 and data['hold_after_full_lift']
        elif kind == 'table_return_pass':
            assert data['duration_sim_s'] >= 3
            returns.append(data['payload'])
        elif kind == 'raw_gripper_velocity':
            for value in data['velocities'].values():
                assert velocity_violation(value) is None
                maximum_velocity = max(maximum_velocity, abs(value))
        elif kind == 'gripper_controller':
            for value in data['output_effort']:
                assert np.isfinite(value) and abs(value) <= 18.01
                maximum_force = max(maximum_force, abs(value))
        elif kind == 'PASS' and data.get('sequential_actual_cycle') is True:
            assert data['order']['phase'] == 'complete' and not data['actual_holder_registry']
            last_sim, last_wall = event['sim'], event['wall']
        if kind.endswith('_fault') or kind == 'FAIL':
            faults.append(dict(kind=kind, sim=event['sim']))
result = json.loads((ROOT / 'run/result.json').read_text())
assert result['status'] == 'PASS' and result['sequential_actual_cycle'] and not result['paired_actual_cycle']
assert not holders and len(anchors) == 4 and not faults
assert loads == ['left', 'right'] and set(returns) == {'left', 'right'} and len(returns) == 2
assert counts['drop_acceptance_pass'] == counts['retrieval_static_grip_hold_pass'] == 2
assert counts['MACHINE_CYCLE_PASS'] == counts['terminal_table_hold_pass'] == 1
assert counts['sequential_remaining_support'] > 100
assert last_sim is not None and first_sim is not None
audit = dict(status='PASS_SEQUENTIAL_TASK_NOT_SPEED_CERTIFICATE', counts=dict(counts), actual_anchors=4,
             actual_maximum_holders=1, loads=loads, returns=returns, faults=faults,
             maximum_output_effort_N=maximum_force, maximum_native_finger_velocity_m_s=maximum_velocity,
             original_velocity_numeric_tolerance_m_s=1e-8,
             elapsed_sim_s=last_sim - first_sim, elapsed_wall_s=last_wall - first_wall)
with (ROOT / 'sequential_task_audit.json').open('x') as stream:
    json.dump(audit, stream, indent=2)
print(json.dumps(audit), flush=True)

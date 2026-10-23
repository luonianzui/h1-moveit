from collections import OrderedDict
import gzip
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from execute_loading import pose_matrix


EVIDENCE = Path(__file__).resolve().parent


def verify_bracket(pair, pose_stamp, frames):
    lower, upper = pair['lower_stamp'], pair['upper_stamp']
    assert lower <= pose_stamp <= upper
    assert upper - lower <= .012 + 1e-9
    assert lower in frames and upper in frames
    fraction = 0. if upper == lower else (pose_stamp - lower) / (upper - lower)
    assert abs(fraction - pair['fraction']) < 1e-9
    assert frames[lower].keys() == frames[upper].keys() == pair['positions'].keys()
    expected = {name:frames[lower][name] + fraction * (frames[upper][name] - frames[lower][name])
                for name in frames[lower]}
    assert all(abs(expected[name] - pair['positions'][name]) < 1e-12 for name in expected)
    return expected


def error(reference, actual):
    return (float(np.linalg.norm(reference[:3, 3] - actual[:3, 3])),
            math.degrees(Rotation.from_matrix(reference[:3, :3].T @ actual[:3, :3]).magnitude()))


def main():
    from moveit.core.robot_model import RobotModel
    from moveit.core.robot_state import RobotState
    root = EVIDENCE.parents[1]
    model = RobotModel(str(root / 'runtime_evidence/centrifuge_robot_press_20260922_r119/robot.urdf'),
                       str(root / 'src/h1_2_moveit_config/config/h1_2.srdf'))
    state = RobotState(model)
    frames = OrderedDict()
    reference = None
    checks = 0
    anchors = 0
    peaks = dict(fk_slip_m=0., native_slip_m=0., angle_deg=0., native_lag_s=0., legacy_unaligned_fk_slip_m=0.)
    with gzip.open(EVIDENCE / 'run/loading_trace.jsonl.gz', 'rt') as stream:
        for line in stream:
            event = json.loads(line)
            data = event['payload']
            if event['kind'] == 'actual_full_joint_frame':
                frames[data['source_sim']] = data['positions']
                while len(frames) > 1024:
                    frames.popitem(last=False)
            elif event['kind'] == 'coherent_guard_scope_end':
                reference = None
            elif event['kind'] == 'coherent_grip_reference_frozen':
                assert reference is None and data['current_payload_not_used_to_rebase']
                positions = verify_bracket(data['actual_joint_pair'], data['pose_sim'], frames)
                state.joint_positions = positions
                state.update()
                fk = state.get_global_link_transform(data['hand'] + '_wrist_yaw_link')
                native = pose_matrix(data['paired_poses'][data['hand'] + '_wrist_yaw_link'])
                native_reference = np.array(data['native_reference'])
                fk_reference = np.array(data['fk_reference'])
                assert np.allclose(np.linalg.inv(fk) @ native @ native_reference, fk_reference, atol=1e-10, rtol=0)
                reference = dict(hand=data['hand'], payload=data['payload'], native=native_reference, fk=fk_reference)
                anchors += 1
            elif event['kind'] in ('synchronized_grip', 'retrieval_grip'):
                assert reference is not None
                assert data['hand'] == reference['hand'] and data['payload'] == reference['payload']
                positions = verify_bracket(data['actual_joint_bracket'], data['coherent_pose_sim'], frames)
                state.joint_positions = positions
                state.update()
                fk = state.get_global_link_transform(data['hand'] + '_wrist_yaw_link').copy()
                paired = data['paired_poses']
                latest = data['latest_poses']
                native = pose_matrix(paired[data['hand'] + '_wrist_yaw_link'])
                native_now = pose_matrix(latest[data['hand'] + '_wrist_yaw_link'])
                body = pose_matrix(paired['box48_' + data['payload']])
                body_now = pose_matrix(latest['box48_' + data['payload']])
                agreement = error(fk, native)
                assert agreement[0] <= .001 and agreement[1] <= .2
                fk_metrics = error(fk @ reference['fk'], body)
                pair_metrics = error(native @ reference['native'], body)
                latest_metrics = error(native_now @ reference['native'], body_now)
                assert all(distance <= .002 and degrees <= 5 for distance, degrees in
                           (fk_metrics, pair_metrics, latest_metrics))
                assert abs(fk_metrics[0] - data['fk_slip_m']) < 1e-10
                assert abs(latest_metrics[0] - data['latest_native_slip_m']) < 1e-10
                assert 0 <= data['pose_sim'] - data['coherent_pose_sim'] <= .012 + 1e-9
                for key, value in [('fk_slip_m', fk_metrics[0]),
                    ('native_slip_m', max(pair_metrics[0], latest_metrics[0])),
                    ('angle_deg', max(fk_metrics[1], pair_metrics[1], latest_metrics[1])),
                    ('native_lag_s', data['native_lag_s']),
                    ('legacy_unaligned_fk_slip_m', data['legacy_unaligned_fk_slip_m'])]:
                    peaks[key] = max(peaks[key], value)
                checks += 1
            elif event['kind'] in ('coherent_grip_fault', 'coherent_wrist_agreement_fault', 'FAIL'):
                raise RuntimeError('Runtime guard failed; no independent PASS')
    assert anchors == 4 and checks > 100
    assert json.loads((EVIDENCE / 'cycle_audit.json').read_text())['status'] == 'PASS'
    with (EVIDENCE / 'coherent_audit.json').open('x') as stream:
        json.dump(dict(status='PASS', actual_joint_brackets_recomputed=True,
            current_payload_not_used_to_rebase=True, immediate_latest_native_recomputed=True,
            checks=checks, anchors=anchors, peaks=peaks,
            original_unaligned_time_contract_not_claimed=True), stream, indent=2)
    print('COHERENT_INDEPENDENT_PASS', checks, anchors, peaks, flush=True)


if __name__ == '__main__':
    main()

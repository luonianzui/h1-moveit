from collections import OrderedDict
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation


EVIDENCE = Path(__file__).resolve().parent
RUN = EVIDENCE
sys.path.insert(0, str(RUN))
from audit_coherent import verify_bracket
from execute_loading import pose_matrix
from speed_policy import PROFILE


def metric(expected, actual):
    return float(np.linalg.norm(expected[:3, 3] - actual[:3, 3])), math.degrees(
        Rotation.from_matrix(expected[:3, :3].T @ actual[:3, :3]).magnitude())


class Ownership:
    def __init__(self):
        self.rows = {}
        self.begins = 0
        self.ends = 0

    def begin(self, hand, payload, native, fk):
        if hand not in ('left', 'right') or payload not in ('left', 'right') or hand in self.rows:
            raise ValueError('Independent audit rejects holder rebase or unknown identity')
        if any(row['payload'] == payload for row in self.rows.values()):
            raise ValueError('A payload cannot be simultaneously certified for two hands')
        self.rows[hand] = dict(payload=payload, native=np.array(native), fk=np.array(fk))
        self.begins += 1

    def end(self, hand, payload):
        if hand not in self.rows or self.rows[hand]['payload'] != payload:
            raise ValueError('Independent audit rejects unmatched release')
        self.rows.pop(hand)
        self.ends += 1


def main():
    from moveit.core.robot_model import RobotModel
    from moveit.core.robot_state import RobotState

    assert json.loads((RUN / 'run/trace_metadata.json').read_text())['status'] == 'CLOSED'
    identity = json.loads((RUN / 'identity.json').read_text())['sha256']
    for name in ('audit_coherent.py', 'execute_loading.py', 'inputs/robot_dual.urdf'):
        assert hashlib.sha256((RUN / name).read_bytes()).hexdigest() == identity[name]
    state = RobotState(RobotModel(str(RUN / 'inputs/robot_dual.urdf'),
                                  str(EVIDENCE.parents[1] / 'src/h1_2_moveit_config/config/h1_2.srdf')))
    owners = Ownership()
    frames = OrderedDict()
    checks = {'left': 0, 'right': 0}
    peaks = dict(fk_slip_m=0., native_slip_m=0., angle_deg=0., agreement_m=0., agreement_deg=0.)
    faults = []
    simultaneous_checks = 0
    saw_full_pass = False
    with gzip.open(RUN / 'run/loading_trace.jsonl.gz', 'rt') as stream:
        for line in stream:
            event = json.loads(line)
            data = event['payload']
            kind = event['kind']
            if kind == 'actual_full_joint_frame':
                frames[data['source_sim']] = data['positions']
                while len(frames) > 1024:
                    frames.popitem(last=False)
            elif kind == 'coherent_grip_reference_frozen':
                positions = verify_bracket(data['actual_joint_pair'], data['pose_sim'], frames)
                state.joint_positions = positions
                state.update()
                fk = state.get_global_link_transform(data['hand'] + '_wrist_yaw_link')
                native = pose_matrix(data['paired_poses'][data['hand'] + '_wrist_yaw_link'])
                expected = np.linalg.inv(fk) @ native @ np.array(data['native_reference'])
                assert np.allclose(expected, data['fk_reference'], atol=1e-10, rtol=0)
                assert data['current_payload_not_used_to_rebase']
                owners.begin(data['hand'], data['payload'], data['native_reference'], data['fk_reference'])
                assert len(owners.rows) == 1
            elif kind == 'coherent_guard_scope_end':
                hand, payload = data['previous_key']
                owners.end(hand, payload)
            elif kind in ('synchronized_grip', 'retrieval_grip'):
                hand = data['hand']
                assert hand in owners.rows and owners.rows[hand]['payload'] == data['payload']
                reference = owners.rows[hand]
                positions = verify_bracket(data['actual_joint_bracket'], data['coherent_pose_sim'], frames)
                state.joint_positions = positions
                state.update()
                fk = state.get_global_link_transform(hand + '_wrist_yaw_link').copy()
                native = pose_matrix(data['paired_poses'][hand + '_wrist_yaw_link'])
                latest = pose_matrix(data['latest_poses'][hand + '_wrist_yaw_link'])
                paired_body = pose_matrix(data['paired_poses']['box48_' + data['payload']])
                latest_body = pose_matrix(data['latest_poses']['box48_' + data['payload']])
                agreement = metric(fk, native)
                assert agreement[0] <= .001 and agreement[1] <= .2
                observed = [metric(fk @ reference['fk'], paired_body),
                            metric(native @ reference['native'], paired_body),
                            metric(latest @ reference['native'], latest_body)]
                assert all(distance <= PROFILE['carry_slip_m'] and angle <= PROFILE['carry_angle_deg'] for distance, angle in observed)
                assert abs(observed[0][0] - data['fk_slip_m']) < 1e-10
                assert abs(observed[-1][0] - data['latest_native_slip_m']) < 1e-10
                assert 0 <= data['pose_sim'] - data['coherent_pose_sim'] <= PROFILE['coherent_native_lag_sim_s'] + 1e-9
                peaks['fk_slip_m'] = max(peaks['fk_slip_m'], observed[0][0])
                peaks['native_slip_m'] = max(peaks['native_slip_m'], observed[1][0], observed[2][0])
                peaks['angle_deg'] = max(peaks['angle_deg'], *(row[1] for row in observed))
                peaks['agreement_m'] = max(peaks['agreement_m'], agreement[0])
                peaks['agreement_deg'] = max(peaks['agreement_deg'], agreement[1])
                checks[hand] += 1
                simultaneous_checks += len(owners.rows) == 2
            elif kind == 'PASS' and data.get('scope') == 'two_box_load_close_spin_open_retrieve':
                saw_full_pass = True
            if kind.endswith('_fault') or kind == 'FAIL':
                faults.append(dict(kind=kind, sim=event['sim']))
    result = json.loads((RUN / 'run/result.json').read_text())
    complete = (result['status'] == 'PASS' and saw_full_pass and not faults and not owners.rows and
                owners.begins == owners.ends == 4 and min(checks.values()) > 100 and simultaneous_checks == 0 and
                not result.get('actual_holder_registry', {'unknown': True}))
    audit = dict(declared_relaxed_limits=PROFILE, original_strict_certificate_not_claimed=True, status='PASS' if complete else 'FAIL', complete_task_pass=complete, checks=checks,
                 anchors=owners.begins, observed_ends=owners.ends, still_held=list(owners.rows),
                 simultaneous_checks=simultaneous_checks, peaks=peaks, faults=faults,
                 original_actual_joint_brackets_and_both_immediate_native_guards_recomputed=True,
                 commanded_values_or_current_payload_reference_rebase_not_used=True)
    with (EVIDENCE / 'independent_speed_grip.json').open('x') as stream:
        json.dump(audit, stream, indent=2)
    print(json.dumps(audit, indent=2))
    assert complete


if __name__ == '__main__':
    main()


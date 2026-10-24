import hashlib
import json
import math

import numpy as np

from dual_motion import TASK_JOINTS
from sequential_geometry import CONTRACT, EVIDENCE, SinglePlanner


def main():
    source = EVIDENCE / 'sequential_chain_revision2.json'
    chain = json.loads(source.read_text())
    assert chain['status'] == 'PASS_OFFLINE_SEQUENTIAL_CHAIN_GEOMETRY_ONLY'
    parent = EVIDENCE.parent / 'centrifuge_48box_elbow_full_20261016_r291/run/1_load_pickup_request.json'
    request = json.loads(parent.read_text())
    request.update(sequential_contract=CONTRACT, hand='left', payload='left', original_runtime_holder_references={})
    planner = SinglePlanner(request)
    records, peaks = [], {}
    for row in chain['stages']:
        name, result = row['name'], row['result']
        if not (name.endswith('_load_transport') or name.endswith('_retrieval_transport')):
            continue
        hand = name.split('_')[0]
        other = 'right' if hand == 'left' else 'left'
        planner.hand = hand
        branch = 'retrieval' if '_retrieval_' in name else 'load'
        for stage in result['stages']:
            samples = stage['samples']
            metrics = []
            frozen_other = {key: samples[0][key] for key in samples[0] if key.startswith(other + '_')}
            for joints in samples:
                assert set(joints) == TASK_JOINTS and all(math.isfinite(value) for value in joints.values())
                assert all(joints[key] == value for key, value in frozen_other.items())
                vector = np.array([joints[key] for key in planner.names])
                assert np.all(vector >= planner.lower - 1e-12) and np.all(vector <= planner.upper + 1e-12)
                metrics.append(planner.elbow_metrics(joints)['from_down_deg'])
            key = branch + '_' + hand
            basket = stage['phase'] in ('load_lower', 'retrieve_axis_exit', 'retrieve_lift')
            if basket:
                peaks[key] = max(peaks.get(key, 0.), max(metrics))
            records.append(dict(name=name, phase=stage['phase'], hand=hand,
                                samples=len(samples), basket_geometry=basket,
                                forearm_max_from_down_deg=max(metrics), forearm_end_from_down_deg=metrics[-1],
                                inactive_arm_preserved_in_whole_scene=True))
    assert len(chain['stages']) == 17 and len(records) == 16
    assert set(peaks) == {'load_left', 'load_right', 'retrieval_left', 'retrieval_right'}
    result = dict(status='PASS_OFFLINE_FK_AND_ORDER_REVIEW_NOT_GAZEBO', records=records,
                  basket_forearm_max_from_down_deg=peaks, whole_chain_dense_checks_reported=chain['total_dense_checks'],
                  original_torso_bound_deg=44., original_joint_margin_rad=.02,
                  original_dense_step_rad=.002, original_wrist_position_m=.00002,
                  original_wrist_angle_deg=.01, no_physical_stability_or_runtime_claim=True,
                  whole_chain_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    with (EVIDENCE / 'sequential_geometry_review.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print('SEQUENTIAL_OFFLINE_POSTURE_REVIEW', peaks, chain['total_dense_checks'], flush=True)


if __name__ == '__main__':
    main()

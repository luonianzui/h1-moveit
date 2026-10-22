from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np

from dual_worker import PairedPlanner
from paired_setdown_braking import split_paired_setdown


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_startup_window_20261013_r247'


def main():
    manifest = dict((relative.lstrip('*'), checksum) for checksum, relative in
                    (line.split(maxsplit=1) for line in (PARENT / 'SHA256SUMS').read_text().splitlines()))
    request_path = PARENT / 'run/7_retrieval_transport_request.json'
    result_path = PARENT / 'run/7_retrieval_transport_result.json'
    for source in (request_path, result_path):
        assert hashlib.sha256(source.read_bytes()).hexdigest() == manifest[str(source.relative_to(PARENT))]
    request = json.loads(request_path.read_text())
    result = json.loads(result_path.read_text())
    assert result['status'] == 'PASS' and request['runtime_reference_not_rebased']
    stage = deepcopy(next(stage for stage in result['stages'] if stage['phase'] == 'setdown'))
    original = deepcopy(stage['samples'])
    stage['setdown_contract'] = dict(cartesian_samples=121, extra_initial_seed=True,
                                    vertical_span_by_hand_m=dict(left=.3, right=.3))
    coarse, fine, split = split_paired_setdown(stage)
    assert coarse[:-1] + fine == original
    planner = PairedPlanner(request, request['planning_references'])
    measured = {}
    for label, index, expected in (('first', 0, 1.14475), ('split', split, .85475), ('last', len(original) - 1, .84475)):
        joints = dict(request['positions'])
        joints.update(original[index])
        planner.planner.state.joint_positions = joints
        planner.planner.state.update()
        measured[label] = {}
        for hand in request['assignment']:
            body = planner.planner.state.get_global_link_transform(hand + '_wrist_yaw_link') @ planner.references[hand]
            height = float(body[2, 3])
            assert abs(height - expected) <= .00002, (label, hand, height)
            measured[label][hand] = height
    for hand in request['assignment']:
        assert abs(measured['split'][hand] - measured['last'][hand] - .01) <= .00004
    audit = dict(status='PASS', scope='original actual-plan FK split geometry only, not new execution',
                 points_preserved=len(original), split_index=split, fine_points=len(fine), heights_m=measured,
                 source_request_sha256=hashlib.sha256(request_path.read_bytes()).hexdigest(),
                 source_result_sha256=hashlib.sha256(result_path.read_bytes()).hexdigest(),
                 original_dense_fcl_checks=result['checked_samples'], original_path_and_references_unchanged=True,
                 no_robot_command_reset_detach_or_physical_attachment=True)
    with (EVIDENCE / 'actual_paired_setdown_split_audit.json').open('x') as output:
        json.dump(audit, output, indent=2)
    print(json.dumps(audit, indent=2))


if __name__ == '__main__':
    main()

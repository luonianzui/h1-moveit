import hashlib
import json
from pathlib import Path

from contact_phase_policy import bounded_motion
from dual_motion import common_arm_timing


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_table_clearance_20261013_r250'


def actual_approach_fixture():
    manifest = {relative.lstrip('*'): expected for expected, relative in
                (line.split(maxsplit=1) for line in (PARENT / 'SHA256SUMS').read_text().splitlines())}
    documents = []
    for name in ('run/5_retrieval_pickup_request.json', 'run/5_retrieval_pickup_result.json'):
        content = (PARENT / name).read_bytes()
        if hashlib.sha256(content).hexdigest() != manifest[name]:
            raise ValueError('Original actual approach fixture changed')
        documents.append(json.loads(content))
    request, result = documents
    if request['mode'] != 'retrieval_pickup' or result['status'] != 'PASS' or result['maximum_joint_step_rad'] != .002:
        raise ValueError('Actual two-arm dense-collision approach required')
    samples = next(stage['samples'] for stage in result['stages'] if stage['phase'] == 'approach')
    return request['positions'], samples


def main():
    actual, samples = actual_approach_fixture()
    rate, tracking = bounded_motion('paired_retrieval_approach', .8)
    old = common_arm_timing(actual, samples, .13)
    candidate = common_arm_timing(actual, samples, rate)
    assert candidate['points'] == old['points']
    assert candidate['duration_sim_s'] < old['duration_sim_s']
    assert tracking == .002 and candidate['original_floor_sim_s'] == .020
    audit = dict(status='PASS', scope='Original r250 actual path offline retiming only; not r251 physical execution or complete-speed PASS',
                 parent=PARENT.name, samples=len(samples), old_rate_rad_s=.13, candidate_rate_rad_s=rate,
                 original_tracking_limit_rad=tracking, old_path_duration_sim_s=old['duration_sim_s'],
                 candidate_path_duration_sim_s=candidate['duration_sim_s'],
                 unchanged_all_path_points=True, original_twenty_ms_floor_preserved=True,
                 original_actual_gripper_velocity_guard_m_s=.2, actual_current_run_plan_still_required=True)
    with (EVIDENCE / 'medium_retrieval_probe.json').open('x') as stream:
        json.dump(audit, stream, indent=2)
    print(json.dumps(audit))


if __name__ == '__main__':
    main()

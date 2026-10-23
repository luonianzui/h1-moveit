from copy import deepcopy
import hashlib
import json
from pathlib import Path

from retrieval_worker import RetrievalPlanner


EVIDENCE = Path(__file__).resolve().parent
BASELINE = EVIDENCE.parent / 'centrifuge_48box_demo_complete_20261010_r214'


def write_json(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2)


def main():
    first_transport = json.loads((BASELINE / 'run/left_right_transport_plan.json').read_text())
    retreat = next(stage for stage in first_transport['stages'] if stage['phase'] == 'retreat')
    pickup_request = json.loads((BASELINE / 'run/right_left_pickup_request.json').read_text())
    original_pickup_request = deepcopy(pickup_request)
    pickup_request['positions'].update(retreat['samples'][-1])
    write_json(EVIDENCE / 'skip_return_pickup_request.json', pickup_request)
    pickup = RetrievalPlanner(pickup_request).pickup()
    write_json(EVIDENCE / 'skip_return_pickup_plan.json', pickup)
    transport_request = json.loads((BASELINE / 'run/right_left_transport_request.json').read_text())
    transport_request['positions'].update(pickup_request['positions'])
    transport_request['positions'].update(pickup['stages'][-1]['samples'][-1])
    for number in (1, 2):
        name = 'left_dex1_finger_' + str(number) + '_joint'
        original = json.loads((BASELINE / 'run/right_left_transport_request.json').read_text())
        transport_request['positions'][name] = original['positions'][name]
    write_json(EVIDENCE / 'skip_return_transport_request.json', transport_request)
    transport = RetrievalPlanner(transport_request).transport()
    write_json(EVIDENCE / 'skip_return_transport_plan.json', transport)
    audit = {
        'status': 'PASS', 'scope': 'offline_actual_r214_geometry_branch_not_physical_execution',
        'first_hand': 'right', 'second_hand': 'left',
        'first_empty_return_omitted': True,
        'first_full_retreat_endpoint': retreat['samples'][-1],
        'second_plan_full_actual_other_arm_retained': True,
        'pickup_checked_samples': pickup['checked_samples'],
        'transport_checked_samples': transport['checked_samples'],
        'maximum_joint_sampling_rad': 0.002,
        'commanded_carry_tilt_limit_deg': 2,
        'original_pickup_request_sha256': hashlib.sha256(
            json.dumps(original_pickup_request, sort_keys=True).encode()).hexdigest(),
        'pickup_phases': [stage['phase'] for stage in pickup['stages']],
        'transport_phases': [stage['phase'] for stage in transport['stages']],
        'runtime_fresh_snapshot_and_independent_FCL_still_required': True,
        'no_robot_commands': True,
    }
    write_json(EVIDENCE / 'skip_return_offline_audit.json', audit)
    print(json.dumps(audit, indent=2))


if __name__ == '__main__':
    main()


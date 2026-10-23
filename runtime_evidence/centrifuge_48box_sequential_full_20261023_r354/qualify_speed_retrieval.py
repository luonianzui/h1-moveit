import copy
import hashlib
import json
from pathlib import Path

from sequential_worker import plan as original_plan
from speed_planning import plan
from speed_policy import configure_simulation_profile


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261020_r325'


def main():
    source = PARENT / 'run/16_retrieval_transport_left_request.json'
    data = source.read_bytes()
    request = json.loads(data)
    preserved = copy.deepcopy(request)
    configure_simulation_profile()
    try:
        original_plan(request)
    except RuntimeError as error:
        assert 'two-degree commanded payload tilt exceeded' in str(error)
    else:
        raise AssertionError('Original failure must remain reproducible')
    result = plan(request)
    assert request == preserved
    assert result['status'] == 'PASS_GEOMETRY_ONLY'
    assert result['maximum_joint_step_rad'] == .002
    assert result['commanded_tilt_limit_deg'] == 5.
    assert 2. <= result['maximum_planned_payload_tilt_deg'] < 5.
    assert result['original_runtime_holder_references'] == request['original_runtime_holder_references']
    assert result['world_level_after_axis_exit_with_original_actual_yaw']
    lift = next(stage for stage in result['stages'] if stage['phase'] == 'retrieve_lift')
    assert abs(lift['last_body'][2][2] - 1.) < 1e-12
    with (ROOT / 'speed_retrieval_qualification.json').open('x') as stream:
        json.dump(dict(status='PASS_ACTUAL_FAILED_SNAPSHOT_DENSE_GEOMETRY_NOT_PHYSICAL',
                       source=str(source), source_sha256=hashlib.sha256(data).hexdigest(),
                       original_two_degree_rejection_reproduced=True,
                       original_actual_snapshot_and_holder_anchors_unchanged=True, plan=result), stream, indent=2)
    print('SPEED_RETRIEVAL_GEOMETRY_ONLY', result['checked_samples'], result['maximum_planned_payload_tilt_deg'])


if __name__ == '__main__':
    main()

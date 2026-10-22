import copy
import hashlib
import json
from pathlib import Path

from speed_planning import plan
from speed_policy import configure_simulation_profile


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261020_r318'


def main():
    configure_simulation_profile()
    records = []
    for name in ('16_retrieval_transport_left_request.json', '20_retrieval_transport_right_request.json'):
        path = PARENT / 'run' / name
        content = path.read_bytes()
        request = json.loads(content)
        original = copy.deepcopy(request)
        result = plan(request)
        assert request == original
        assert result['status'] == 'PASS_GEOMETRY_ONLY'
        assert result['maximum_joint_step_rad'] == .002
        assert result['world_level_after_axis_exit_with_original_actual_yaw']
        assert result['original_runtime_holder_references'] == request['original_runtime_holder_references']
        records.append(dict(source=str(path), source_sha256=hashlib.sha256(content).hexdigest(), plan=result))
    with (ROOT / 'speed_retrieval_regression.json').open('x') as stream:
        json.dump(dict(status='PASS_TWO_R318_ACTUAL_SNAPSHOTS_NOT_NEW_PHYSICS', records=records), stream, indent=2)
    print('R318_RETRIEVAL_LEVEL_REGRESSION_PASS', [(row['plan']['single_hand'], row['plan']['checked_samples']) for row in records])


if __name__ == '__main__':
    main()

import copy
import hashlib
import json
from pathlib import Path
import sys

from empty_retreat_geometry import plan_empty_retreat


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_clock_budget_20261013_r245'
sys.path.insert(0, str(PARENT))
from dual_worker import PairedPlanner


def main():
    path = PARENT / 'run/4_empty_return_request.json'
    request = json.loads(path.read_text())
    original = copy.deepcopy(request)
    neutral = json.loads((PARENT / 'inputs/plan.json').read_text())['initial_positions']
    rows = []
    for lift, width in ((.12, .30), (.10, .30), (.08, .32), (.15, .32)):
        try:
            plan = plan_empty_retreat(PairedPlanner, request, neutral, lift, width)
            name = f'empty_retreat_{round(lift * 100):02d}_{round(width * 100):02d}.json'
            with (EVIDENCE / name).open('x') as stream:
                json.dump(plan, stream, indent=2)
            rows.append(dict(status='DENSE_GEOMETRY_PASS_NOT_RUNTIME', lift_m=lift, outboard_y_m=width,
                             checked=plan['checked_samples'], path=name))
            break
        except (RuntimeError, ValueError) as error:
            rows.append(dict(status='REJECTED', lift_m=lift, outboard_y_m=width, error=str(error)))
        print(rows[-1], flush=True)
    assert request == original
    audit = dict(status='PASS' if any(row['status'] == 'DENSE_GEOMETRY_PASS_NOT_RUNTIME' for row in rows) else 'FAIL',
                 records=rows, actual_request_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                 actual_measured_source_not_old_nominal_reversal=True,
                 request_pose_joint_and_holder_data_unchanged=True, no_robot_command_or_guard_relaxation=True,
                 fresh_actual_scene_plan_and_runtime_tracking_still_required=True)
    with (EVIDENCE / 'actual_empty_retreat_audit.json').open('x') as stream:
        json.dump(audit, stream, indent=2)
    print(json.dumps(audit), flush=True)
    assert audit['status'] == 'PASS'


if __name__ == '__main__':
    main()

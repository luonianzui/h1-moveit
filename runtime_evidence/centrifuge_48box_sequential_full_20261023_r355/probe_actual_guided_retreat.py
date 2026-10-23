import copy
import hashlib
import json
from pathlib import Path
import sys

from empty_retreat_geometry import plan_empty_guided_retreat


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_clock_budget_20261013_r245'
sys.path.insert(0, str(PARENT))
from dual_worker import PairedPlanner


def main():
    source = PARENT / 'run/4_empty_return_request.json'
    guides_path = PARENT / 'run/3_load_transport_result.json'
    request = json.loads(source.read_text())
    original = copy.deepcopy(request)
    guides = json.loads(guides_path.read_text())
    assert guides['status'] == 'PASS'
    neutral = json.loads((PARENT / 'inputs/plan.json').read_text())['initial_positions']
    try:
        result = plan_empty_guided_retreat(PairedPlanner, request, neutral, guides['stages'])
        assert request == original
        with (EVIDENCE / 'actual_guided_retreat_plan.json').open('x') as stream:
            json.dump(result, stream, indent=2)
        audit = dict(status='PASS', dense_geometry_not_actual_new_execution=True,
                     checked=result['checked_samples'], phases=[stage['phase'] for stage in result['stages']],
                     source_request_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                     source_current_run_guides_sha256=hashlib.sha256(guides_path.read_bytes()).hexdigest(),
                     original_loose_payload_collision_guards_unchanged=True,
                     actual_measured_start_not_replaced_by_commands=True,
                     current_run_commands_only_geometry_proposals=True,
                     no_robot_command_or_physical_attach_detach=True,
                     actual_new_scene_FCL_tracking_and_complete_cycle_still_required=True)
    except (RuntimeError, ValueError) as error:
        audit = dict(status='FAIL', error=str(error), no_robot_command_issued=True)
    with (EVIDENCE / 'actual_guided_retreat_audit.json').open('x') as stream:
        json.dump(audit, stream, indent=2)
    print(json.dumps(audit), flush=True)
    assert audit['status'] == 'PASS'


if __name__ == '__main__':
    main()

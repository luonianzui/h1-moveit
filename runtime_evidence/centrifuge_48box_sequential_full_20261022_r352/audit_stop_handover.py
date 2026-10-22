import hashlib
import json
from pathlib import Path

from stop_handover_worker import audit


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_empty_return_gain_20261014_r255'


def main():
    manifest = dict((relative.lstrip('*'), checksum) for checksum, relative in
                    (line.split(maxsplit=1) for line in (PARENT / 'SHA256SUMS').read_text().splitlines()))
    candidates = sorted((PARENT / 'run').glob('*_empty_return_request.json'))
    assert len(candidates) == 1
    source = candidates[0]
    assert hashlib.sha256(source.read_bytes()).hexdigest() == manifest[str(source.relative_to(PARENT))]
    actual = json.loads(source.read_text())
    assert not actual['original_runtime_holder_references']
    positions = dict(actual['positions'])
    initial = json.loads((EVIDENCE / 'inputs/plan.json').read_text())['initial_positions']
    plan = json.loads((EVIDENCE / 'inputs/button_plan.json').read_text())
    positions['torso_joint'] = initial['torso_joint']
    positions.update(dict(zip(plan['arms']['left']['joints'], plan['arms']['left']['approach'][0])))
    positions.update(dict(zip(plan['arms']['right']['joints'], plan['arms']['right']['pre'])))
    request = dict(mode='stop_handover', positions=positions, source_sim=actual['source_sim'],
                   original_runtime_holder_references={}, nominal_task_joints_replaced_for_plan_only_fixture=True)
    with (EVIDENCE / 'stop_handover_fixture_request.json').open('x') as output:
        json.dump(request, output, indent=2)
    result = audit(request)
    result.update(parent_actual_snapshot_sha256=manifest[str(source.relative_to(PARENT))],
                  plan_only_fixture_not_current_actual_handover=True, new_actual_worker_still_required=True)
    with (EVIDENCE / 'stop_handover_fixture_audit.json').open('x') as output:
        json.dump(result, output, indent=2)
    print('STOP_HANDOVER_NOMINAL_FIXTURE_PASS_NOT_ACTUAL_CERTIFICATE', flush=True)


if __name__ == '__main__':
    main()

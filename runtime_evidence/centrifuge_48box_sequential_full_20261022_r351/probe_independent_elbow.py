import copy
import json
from pathlib import Path

from audit_elbow import audit_plan
from paired_setdown_braking import split_paired_setdown


ROOT = Path(__file__).resolve().parent
records = []
for prefix in ('1_load_pickup', '7_retrieval_pickup'):
    source = json.loads((ROOT / (prefix + '_elbow_chain.json')).read_text())
    request = copy.deepcopy(source['carry_request'])
    request['original_runtime_holder_references'] = copy.deepcopy(request['planning_references'])
    request['runtime_reference_not_rebased'] = True
    request['prediction_only_not_current_actual_snapshot'] = True
    result = source['carry']
    commands = {}
    phase_prefix = 'paired_retrieval_' if request['retrieval'] else 'paired_load_'
    for stage in result['stages']:
        if stage['phase'] == 'setdown':
            coarse, fine, split = split_paired_setdown(stage)
            commands[phase_prefix + 'setdown'] = dict(samples=coarse, simultaneous_physical_controllers=True)
            commands[phase_prefix + 'setdown_fine'] = dict(samples=fine, simultaneous_physical_controllers=True)
        else:
            commands[phase_prefix + stage['phase']] = dict(samples=stage['samples'], simultaneous_physical_controllers=True)
    audit = audit_plan(request, result, commands)
    audit['synthetic_command_fixture_not_actually_published'] = True
    records.append(audit)
    print('INDEPENDENT_OFFLINE_CARRY', audit, flush=True)
with (ROOT / 'independent_elbow_offline.json').open('x') as stream:
    json.dump(dict(status='PASS_OFFLINE_ONLY_NOT_PHYSICS', records=records,
                   command_fields_are_synthetic_fixture_not_actual_execution=True), stream, indent=2)

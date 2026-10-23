import hashlib
import json
from pathlib import Path

from table_bucket_clearance import bucket_collision_bounds, table_bucket_clearance


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_retrieval_approach_20261013_r249'
ROOT = EVIDENCE.parents[1]


def main():
    source = EVIDENCE / 'actual_table_clearance_native_inputs.json'
    context = json.loads(source.read_text())
    fault = context['fault']
    assert fault['phase'] == 'paired_retrieval_setdown'
    assert not fault['payload']['clearance']['valid']
    trace = json.loads((PARENT / 'trace_audit.json').read_text())
    assert trace['status'] == 'PASS' and trace['decoded_sha256'] == context['trace_utf8_sha256']
    bounds = bucket_collision_bounds(ROOT / 'src/h1_2_description/models/centrifuge/model.sdf')
    results = {}
    for payload in ('left', 'right'):
        measured = context['contacts'][payload]
        assert 0 <= fault['sim'] - measured['stamp'] <= .012
        pairs = measured['pairs']
        results[payload] = table_bucket_clearance(fault['payload']['poses'], payload, pairs, bounds)
    assert all(row['valid'] and row['clearance_m'] >= .003 for row in results.values())
    audit = dict(status='PASS', scope='actual failed table geometry, not new execution or old FAIL reclassification',
                 old_vertical_only_clearance=fault['payload']['clearance'], actual_results=results,
                 fault_sim=fault['sim'], original_three_mm_margin_preserved=True,
                 both_measured_contact_stamps={payload: row['stamp'] for payload, row in context['contacts'].items()},
                 both_original_contact_caches_verified=True,
                 collision_mesh_bounds=bounds, source_context_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                 no_robot_command_reset_detach_or_physical_attachment=True)
    with (EVIDENCE / 'actual_table_clearance_probe.json').open('x') as output:
        json.dump(audit, output, indent=2)
    print(json.dumps(audit, indent=2))


if __name__ == '__main__':
    main()

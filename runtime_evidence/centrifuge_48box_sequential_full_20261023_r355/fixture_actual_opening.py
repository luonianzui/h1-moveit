import hashlib
import json
from pathlib import Path


EVIDENCE = Path(__file__).resolve().parent


def main():
    original = EVIDENCE / 'fixture_nominal_stop.py'
    source = original.read_text()
    source = source.replace("    with (EVIDENCE / 'nominal_stop_fixture_provenance.json').open('x') as stream:",
                            "    source = source.replace(\"request['positions'] = json.loads((EVIDENCE / 'inputs/plan.json').read_text())['initial_positions']\", \"request['positions'] = dict(json.loads((EVIDENCE / 'inputs/plan.json').read_text())['initial_positions'])\\n    for hand in ('left', 'right'):\\n        for number in (1, 2):\\n            request['positions'][hand + f'_dex1_finger_{number}_joint'] = -.0199\")\n    with (EVIDENCE / 'actual_opening_fixture_provenance.json').open('x') as stream:")
    assert 'actual_opening_fixture_provenance.json' in source
    source = source.replace('paired_fixtures_nominal_phase', 'paired_fixtures_actual_opening')
    source = source.replace('paired_fixture_nominal_phase_audit.json', 'paired_fixture_actual_opening_audit.json')
    with (EVIDENCE / 'actual_opening_generator_provenance.json').open('x') as stream:
        json.dump(dict(status='PROSPECTIVE_ACTUAL_RUNTIME_REOPEN_TARGET_NOT_MEASURED_GRASP',
                       source_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
                       generated_sha256=hashlib.sha256(source.encode()).hexdigest(),
                       reopen_target_m=-.0199, old_failed_fixture_retained=True,
                       no_physical_model_pose_or_command_changed=True), stream, indent=2)
    exec(compile(source, str(original), 'exec'), dict(__name__='__main__', __file__=str(original)))


if __name__ == '__main__':
    main()

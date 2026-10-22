import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

from dual_motion import common_arm_timing
from paired_ramp import ramped_common_timing
from sequential_cycle import SequentialCycle
from single_setdown import single_setdown_profile
from speed_policy import PROFILE, common_timing, motion_limits


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261020_r318'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    assert digest(PARENT / 'SHA256SUMS') == '4bdfd4c608f56f0ed0c7b1c4396e727f0986d6b8df0bb7d10c1d3a3c8116823f'
    manifest = {line.split(maxsplit=1)[1]: line.split()[0] for line in (PARENT / 'SHA256SUMS').read_text().splitlines()}
    records = []
    for path in sorted((PARENT / 'run').glob('*_result.json')):
        document = json.loads(path.read_text())
        if document.get('status') != 'PASS_GEOMETRY_ONLY':
            continue
        assert digest(path) == manifest[str(path.relative_to(PARENT))]
        request_path = path.with_name(path.name.replace('_result.json', '_request.json'))
        assert digest(request_path) == manifest[str(request_path.relative_to(PARENT))]
        request = json.loads(request_path.read_text())
        actual = request['positions']
        loaded = bool(document['original_runtime_holder_references'])
        legacy = SimpleNamespace(holders=SimpleNamespace(rows=document['original_runtime_holder_references']))
        for stage in document['stages']:
            phase = ('seq_retrieval_' if 'retrieval' in document['mode'] else 'seq_load_') + stage['phase']
            sections = [(phase, stage['samples'], None)]
            if stage['phase'] == 'retrieve_setdown':
                split = max(1, len(stage['samples']) - 8)
                sections = [('paired_retrieval_setdown', stage['samples'][:split + 1], single_setdown_profile(document['single_hand'])),
                            ('paired_retrieval_setdown_fine', stage['samples'][split:], None)]
            for phase, samples, setdown in sections:
                old_rate, unused = SequentialCycle.motion_limits(legacy, phase, .25 if loaded else .35)
                if setdown:
                    original = ramped_common_timing(actual, samples, setdown[0], setdown[1])
                else:
                    original = common_arm_timing(actual, samples, old_rate)
                rate, unused = motion_limits(phase, loaded)
                candidate = common_timing(actual, samples, rate, ramp_s=.3 if setdown else .15)
                assert candidate['points'] == original['points'] and candidate['groups'] == original['groups']
                records.append(dict(source=str(path), source_sha256=digest(path), phase=phase,
                                    points=len(samples), old_duration_sim_s=original['duration_sim_s'],
                                    candidate_duration_sim_s=candidate['duration_sim_s'], loaded=loaded))
                actual = samples[-1]
    assert len(records) >= 40 and any(row['loaded'] for row in records)
    result = dict(status='PASS_ACTUAL_R318_PATH_RETIMING_ONLY_NOT_NEW_PHYSICS',
                  profile=PROFILE, records=records,
                  old_trajectory_sim_s=sum(row['old_duration_sim_s'] for row in records),
                  candidate_trajectory_sim_s=sum(row['candidate_duration_sim_s'] for row in records),
                  excludes_planning_buttons_feedback_grippers_spin_and_holds=True,
                  points_groups_and_original_path_geometry_unchanged=True)
    with (ROOT / 'speed_qualification.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(result['status'], len(records), result['old_trajectory_sim_s'], result['candidate_trajectory_sim_s'])


if __name__ == '__main__':
    main()

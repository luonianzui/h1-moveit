import json
from pathlib import Path

from trace_io import open_trace
from empty_speed import EMPTY_PHASES, motion_rate, minimum_interval


EVIDENCE = Path(__file__).resolve().parent
BASELINE = EVIDENCE.parents[1] / 'runtime_evidence/centrifuge_48box_full_cycle_20261010_r206'


def summary(root, allow_incomplete=False):
    first = final = None
    phases = []
    with open_trace(root) as stream:
        for line in stream:
            row = json.loads(line)
            if row['sim'] is not None and first is None:
                first = row
            if row['kind'] in ('LOADING_PASS', 'MACHINE_CYCLE_PASS', 'table_return_pass', 'PASS'):
                phases.append(dict(kind=row['kind'], phase=row['phase'], sim=row['sim'], wall=row['wall']))
            if row['kind'] == 'PASS':
                final = row
    assert first is not None and (final is not None or allow_incomplete)
    loading = next(row for row in phases if row['kind'] == 'LOADING_PASS')
    return dict(complete=final is not None,
                duration_sim_s=final['sim'] - first['sim'] if final else None,
                duration_wall_s=final['wall'] - first['wall'] if final else None,
                loading_duration_sim_s=loading['sim'] - first['sim'],
                loading_duration_wall_s=loading['wall'] - first['wall'],
                start_sim=first['sim'], final_sim=final['sim'] if final else None, phases=phases)


def command_duration(data, rate, floor):
    previous = data['initial']
    elapsed = .1
    times = []
    for point in [previous] + data['samples']:
        elapsed += max(floor, max(abs(point[name] - previous[name]) /
            (.0008 if 'dex1_finger' in name else rate) for name in data['names']))
        times.append(elapsed)
        previous = point
    return times


def main():
    assert json.loads((EVIDENCE / 'cycle_audit.json').read_text())['status'] == 'PASS'
    assert json.loads((BASELINE / 'cycle_audit.json').read_text())['gates']['both_loaded']
    retimes, motions, buttons, pilots = {}, [], [], []
    declaration = None
    with open_trace(EVIDENCE) as stream:
        for line in stream:
            row = json.loads(line)
            kind, data, phase = row['kind'], row['payload'], row['phase']
            if kind == 'speed_profile_declared':
                assert declaration is None
                declaration = data
                assert set(data['empty_phases']) == EMPTY_PHASES
                assert data['scale'] == 1.0 and data['maximum_rate_rad_s'] == .36
                assert data['original_arm_rates_and_dense_timing'] and not data['empty_phases']
                assert data['minimum_empty_interval_sim_s'] == .020 and data['button_travel_rate_rad_s'] == .44
                assert not data['loaded_paths_holds_physics_and_gates_unchanged']
                assert data['only_gate_change_retrieval_transient_rpm'] == .1
                assert data['only_loaded_rate_change_pilot_rad_s'] == .03
            elif kind == 'empty_motion_retime':
                assert phase == data['phase'] and phase in EMPTY_PHASES
                assert data['holding'] is None and data['held_payload'] is None and data['unchanged_path']
                assert abs(data['effective_rate_rad_s'] - motion_rate(phase, data['original_rate_rad_s'], None, None)) < 1e-12
                assert data['minimum_interval_s'] == minimum_interval(phase, None, None)
                retimes[phase] = data
            elif kind == 'retrieval_pilot_retime':
                assert phase in ('left_retrieval_pilot_lift', 'right_retrieval_pilot_lift')
                assert data['phase'] == phase and data['original_rate_rad_s'] == .12
                assert data['effective_rate_rad_s'] == .03 and data['original_quiet_rpm'] == .02
                assert data['unchanged_path']
                pilots.append(dict(data))
            elif kind == 'command' and phase in retimes:
                profile = retimes[phase]
                actual = command_duration(data, profile['effective_rate_rad_s'], .020)
                original = command_duration(data, profile['original_rate_rad_s'], .020)
                assert len(actual) == len(data['times'])
                assert max(abs(expected - measured) for expected, measured in zip(actual, data['times'])) < 1e-10
                assert original[-1] >= actual[-1] - 1e-10
                motions.append(dict(phase=phase, original_duration_sim_s=original[-1],
                    actual_duration_sim_s=actual[-1], saving_sim_s=original[-1] - actual[-1]))
            elif kind == 'empty_button_retime':
                assert data['holding'] is None and data['held_payload'] is None
                assert data['unchanged_path'] and data['fixed_contact_intervals_unchanged']
                assert data['original_rate_rad_s'] == .22 and data['effective_rate_rad_s'] == .44
            elif kind == 'button_arm_command' and phase.endswith(('_approach', '_withdraw')):
                points = data['points']
                original = actual = .05
                for previous, current in zip(points, points[1:]):
                    delta = max(abs(first - second) for first, second in zip(
                        previous['positions'], current['positions']))
                    original += max(.02, delta / .22)
                    actual += max(.02, delta / .44)
                if abs(actual - data['duration']) < 1e-10:
                    buttons.append(dict(phase=phase, original_duration_sim_s=original,
                        actual_duration_sim_s=actual, saving_sim_s=original - actual))
    assert declaration is not None and not motions and buttons and len(pilots) == 2
    baseline, candidate = summary(BASELINE, allow_incomplete=True), summary(EVIDENCE)
    saving = sum(row['saving_sim_s'] for row in motions + buttons)
    result = dict(status='PASS' if candidate['complete'] and saving > 0 and
                  candidate['loading_duration_sim_s'] < baseline['loading_duration_sim_s'] else 'FAIL',
        scope='physical_complete_cycle_and_empty_retime_comparison_not_video_speed',
        baseline=baseline, candidate=candidate, empty_motion_commands=motions, empty_button_commands=buttons,
        unchanged_path_pilot_retime=pilots,
        same_path_counterfactual_saved_sim_s=saving,
        measured_loading_saved_sim_s=baseline['loading_duration_sim_s'] - candidate['loading_duration_sim_s'],
        measured_loading_saved_wall_s=baseline['loading_duration_wall_s'] - candidate['loading_duration_wall_s'],
        measured_complete_saved_sim_s=baseline['duration_sim_s'] - candidate['duration_sim_s'] if baseline['complete'] else None,
        measured_complete_saved_wall_s=baseline['duration_wall_s'] - candidate['duration_wall_s'] if baseline['complete'] else None,
        full_baseline_not_passed_no_complete_speedup_claim=not baseline['complete'])
    with (EVIDENCE / 'speed_audit.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))
    assert result['status'] == 'PASS'


if __name__ == '__main__':
    main()

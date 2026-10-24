import json
from pathlib import Path

from trace_io import open_trace


EVIDENCE = Path(__file__).resolve().parent
BASELINE = EVIDENCE.parent / 'centrifuge_48box_fast_complete_20261012_r219'


def main():
    current = json.loads((EVIDENCE / 'cycle_audit.json').read_text())
    previous = json.loads((BASELINE / 'cycle_audit.json').read_text())
    baseline_file=BASELINE/'complete_speed_audit.json'
    if not baseline_file.exists():
        baseline_file=BASELINE/'speed_audit.json'
    baseline=json.loads(baseline_file.read_text())['candidate']
    assert current['status'] == previous['status'] == 'PASS' and baseline['complete']
    tokens = tuple('"kind": "' + kind + '"' for kind in (
        'empty_return_policy_declared', 'verified_empty_return_skipped', 'command',
        'table_return_pass', 'LOADING_PASS', 'MACHINE_CYCLE_PASS', 'PASS'))
    first = final = None
    phases, skips, declarations = [], [], []
    returns, retreats = {}, {}
    return_commands = 0
    with open_trace(EVIDENCE) as stream:
        for line in stream:
            if first is None:
                row = json.loads(line)
                if row['sim'] is not None:
                    first = row
            if not any(token in line[:140] for token in tokens):
                continue
            row = json.loads(line)
            kind, phase, data = row['kind'], row['phase'], row['payload']
            if kind == 'empty_return_policy_declared':
                declarations.append(data)
            elif kind == 'table_return_pass':
                returns[data['payload']] = row
            elif kind == 'command' and phase.endswith('_retrieval_retreat'):
                retreats[phase.split('_')[0]] = row
            elif kind == 'command' and phase.endswith('_retrieval_return'):
                return_commands += 1
            elif kind == 'verified_empty_return_skipped':
                assert data['holding'] is None and data['held_payload'] is None
                assert data['phase'] == 'return' and data['verified_table'] and data['retreat_complete']
                table = returns[data['payload']]
                retreat = retreats[data['hand']]
                assert table['payload']['duration_sim_s'] >= 3 and table['sim'] < row['sim']
                assert retreat['sim'] >= table['sim']
                assert row['sim'] >= retreat['payload']['start'] + retreat['payload']['times'][-1] + .15
                skips.append(row)
            if kind in ('LOADING_PASS', 'MACHINE_CYCLE_PASS', 'table_return_pass', 'PASS'):
                phases.append(dict(kind=kind, phase=phase, sim=row['sim'], wall=row['wall']))
            if kind == 'PASS':
                final = row
    assert first is not None and final is not None and len(skips) == len(declarations) == 1
    assert declarations[0].get('empty_return_rates_holds_and_physics_gates_unchanged',
                               declarations[0]['original_rates_holds_and_physics_gates_unchanged']) is True
    assert declarations[0]['next_hand_fresh_actual_scene_IK_FCL_required'] is True
    assert return_commands == 0 and set(returns) == {'left', 'right'}
    loading = next(row for row in phases if row['kind'] == 'LOADING_PASS')
    machine = next(row for row in phases if row['kind'] == 'MACHINE_CYCLE_PASS')
    last_table = max(row['sim'] for row in phases if row['kind'] == 'table_return_pass')
    candidate = dict(
        complete=True, duration_sim_s=final['sim'] - first['sim'],
        duration_wall_s=final['wall'] - first['wall'],
        loading_duration_sim_s=loading['sim'] - first['sim'],
        machine_duration_sim_s=machine['sim'] - loading['sim'],
        retrieval_duration_sim_s=last_table - machine['sim'],
        terminal_hold_sim_s=final['sim'] - last_table, phases=phases)
    saved = baseline['duration_sim_s'] - candidate['duration_sim_s']
    result = dict(status='PASS' if saved > 0 else 'FAIL',
        scope='complete_physical_cycle_against_complete_PASS_baseline_not_video_speed',
        baseline_identity=BASELINE.name,
        baseline=baseline, baseline_complete_physical_PASS_required=True,
        candidate=candidate, measured_complete_saved_sim_s=saved,
        measured_complete_saved_wall_s=baseline['duration_wall_s'] - candidate['duration_wall_s'],
        measured_sim_reduction_percent=100 * saved / baseline['duration_sim_s'],
        verified_empty_return_skips=skips, unexpected_return_commands=return_commands,
        under_three_minutes_sim=candidate['duration_sim_s'] < 180)
    with (EVIDENCE / 'complete_speed_audit.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))
    assert result['status'] == 'PASS'


if __name__ == '__main__':
    main()


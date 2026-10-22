import json

import pytest

import audit_complete_speed


def fixture(tmp_path):
    candidate, baseline = tmp_path / 'candidate', tmp_path / 'baseline'
    for root in (candidate, baseline):
        root.mkdir()
        (root / 'run').mkdir()
        (root / 'cycle_audit.json').write_text(json.dumps(dict(status='PASS')))
    (baseline / 'speed_audit.json').write_text(json.dumps(dict(status='FAIL', candidate=dict(
        complete=True, duration_sim_s=509.164, duration_wall_s=5047.543210))))
    def row(kind, phase, sim, payload):
        return dict(kind=kind, phase=phase, sim=sim, wall=1000 + sim * 8, payload=payload)
    rows = [
        row('state', 'prepare', 0, {}),
        row('empty_return_policy_declared', 'prepare', 0, dict(
            original_rates_holds_and_physics_gates_unchanged=True,
            next_hand_fresh_actual_scene_IK_FCL_required=True)),
        row('LOADING_PASS', 'loading', 223, {}),
        row('MACHINE_CYCLE_PASS', 'machine', 321, {}),
        row('table_return_pass', 'right_release_on_table', 390, dict(payload='left', duration_sim_s=3)),
        row('command', 'right_retrieval_retreat', 390.1, dict(start=390.25, times=[.1, 6])),
        row('verified_empty_return_skipped', 'right_retrieval_return', 397, dict(
            payload='left', hand='right', phase='return', holding=None, held_payload=None,
            verified_table=True, retreat_complete=True)),
        row('table_return_pass', 'left_release_on_table', 482, dict(payload='right', duration_sim_s=3)),
        row('PASS', 'terminal_table_hold', 492, {}),
    ]
    return candidate, baseline, rows


def save(root, rows):
    (root / 'run/loading_trace.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))


def test_complete_physical_baseline_required_despite_old_speed_audit_FAIL(monkeypatch, tmp_path):
    candidate, baseline, rows = fixture(tmp_path)
    save(candidate, rows)
    monkeypatch.setattr(audit_complete_speed, 'EVIDENCE', candidate)
    monkeypatch.setattr(audit_complete_speed, 'BASELINE', baseline)
    audit_complete_speed.main()
    result = json.loads((candidate / 'complete_speed_audit.json').read_text())
    assert result['status'] == 'PASS'
    assert result['measured_complete_saved_sim_s'] == pytest.approx(17.164)
    assert result['baseline_complete_physical_PASS_required']
    assert not result['under_three_minutes_sim']


@pytest.mark.parametrize('defect', ['held', 'no_table', 'no_retreat', 'return_command', 'failed_baseline'])
def test_unverified_skip_or_failed_full_baseline_never_claims_speed(monkeypatch, tmp_path, defect):
    candidate, baseline, rows = fixture(tmp_path)
    if defect == 'held':
        rows[6]['payload']['holding'] = 'right'
    elif defect == 'no_table':
        rows.pop(4)
    elif defect == 'no_retreat':
        rows.pop(5)
    elif defect == 'return_command':
        rows.insert(7, dict(kind='command', phase='right_retrieval_return', sim=397,
                            wall=4400, payload={}))
    else:
        (baseline / 'cycle_audit.json').write_text(json.dumps(dict(status='FAIL')))
    save(candidate, rows)
    monkeypatch.setattr(audit_complete_speed, 'EVIDENCE', candidate)
    monkeypatch.setattr(audit_complete_speed, 'BASELINE', baseline)
    with pytest.raises((AssertionError, KeyError)):
        audit_complete_speed.main()


import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from floor_time_source import CURRENT, restore_floor_time
from preload_floor_frame import FloorContactHistory, single_missing_floor_frame

ROOT = Path(__file__).resolve().parent
FIXTURE = ROOT.parent / 'centrifuge_48box_sequential_review_20261022_r8/r350_preload_fixture.json'


def fixture():
    data = FIXTURE.read_bytes()
    assert hashlib.sha256(data).hexdigest() == '5d1e206ce238a3344c7c66fbb8d11bfa0b91e3dad4a5ffc96359795fa4dd5afa'
    record = json.loads(data)
    fault = record['fault']
    history = FloorContactHistory()
    for row in record['contacts_before']:
        history.observe(row['payload'])
    cycle = SimpleNamespace(current_grasp_hand='right', grasp_candidate='right', phase='right_retrieval_grasp',
        retrieval_closing=True, holders=SimpleNamespace(rows={}),
        pair_goal_handles={'right': SimpleNamespace(accepted=True)},
        preload_grasp_claim=record['claim']['payload'],
        accepted_gripper_targets={'right': record['goal']['payload']['target']},
        accepted_gripper_efforts={'right': record['goal']['payload']['feedforward_N']},
        sim=fault['sim'], joint_time=record['joints']['payload']['source_sim'],
        positions=record['joints']['payload']['positions'], poses=fault['payload']['poses'],
        contacts={'right': fault['payload']['contacts']}, closing_floor_history=history)
    return cycle, fault['payload']['metrics']


def test_actual_r350_rounding_boundary_not_a_new_grip_or_task_certificate():
    cycle, metrics = fixture()
    previous = {}
    source = (ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r350/preload_floor_frame.py').read_text()
    exec(compile(source, '<sealed-parent-floor-rule>', 'exec'), previous)
    before = copy.deepcopy(cycle.contacts)
    stamp = cycle.closing_floor_history.rows['right']['stamp']
    assert cycle.sim - stamp > .001 and round(cycle.sim * 1e9) - round(stamp * 1e9) == 1000000
    assert not previous['single_missing_floor_frame'](cycle, 'right', metrics)['valid']
    result = single_missing_floor_frame(cycle, 'right', metrics)
    assert result['valid'] and result['not_a_grip_certificate']
    assert result['final_endpoint_and_two_opposed_strong_capture_still_required']
    assert not cycle.holders.rows and cycle.contacts == before
    assert restore_floor_time((ROOT / 'preload_floor_frame.py').read_text()) == source


@pytest.mark.parametrize('age_ns', [-1, 0, 999999, 1000000, 1000001])
def test_exact_one_nanosecond_boundary_still_fails_outside_original_limit(age_ns):
    cycle, metrics = fixture()
    stamp = cycle.closing_floor_history.rows['right']['stamp']
    cycle.sim = (round(stamp * 1e9) + age_ns) / 1e9
    assert single_missing_floor_frame(cycle, 'right', metrics)['valid'] == (0 <= age_ns <= 1000000)


def test_inverse_rejects_an_expanded_deadline():
    source = (ROOT / 'preload_floor_frame.py').read_text()
    with pytest.raises(ValueError, match='exact single'):
        restore_floor_time(source.replace(CURRENT, CURRENT.replace('1_000_000', '1_000_001')))

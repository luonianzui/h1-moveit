import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from preload_floor_frame import FloorContactHistory, single_missing_floor_frame
from preload_observation import PreloadObservationMixin, pending_zero_force_frame, wait_for_observation
from sequential_support import actual_preload_support


ROOT = Path(__file__).resolve().parent
FIXTURE = ROOT.parent / 'centrifuge_48box_sequential_review_20261020_r19/r330_preload_fixture.json'


def actual_fixture():
    data = FIXTURE.read_bytes()
    assert hashlib.sha256(data).hexdigest() == '565c48b124c9ddd13bad50963fd672ce977fb771093d59c315d0572517593c6c'
    record = json.loads(data)
    fault = record['fault']
    history = FloorContactHistory()
    for event in record['contacts_before']:
        history.observe(event['payload'])
    cycle = SimpleNamespace(remaining_guard_enabled=True, current_grasp_hand='right', grasp_candidate='right',
        phase='right_retrieval_grasp', holders=SimpleNamespace(rows={}, snapshot=lambda: {}),
        order=SimpleNamespace(locations={'left': 'table', 'right': 'basket'}), retrieval_closing=True,
        pair_goal_handles={'right': SimpleNamespace(accepted=True)}, preload_grasp_claim=record['claim']['payload'],
        accepted_gripper_targets={'right': record['goal']['payload']['target']},
        accepted_gripper_efforts={'right': record['goal']['payload']['feedforward_N']},
        sim=fault['sim'], joint_time=record['joints']['payload']['source_sim'],
        positions=record['joints']['payload']['positions'], poses=fault['payload']['poses'],
        contacts={'right': fault['payload']['contacts']}, closing_floor_history=history)
    return cycle, fault['payload']['metrics'], record


def test_actual_zero_force_is_wait_eligible_but_original_support_remains_invalid():
    cycle, metrics, _ = actual_fixture()
    before = copy.deepcopy(cycle.contacts)
    assert not actual_preload_support(cycle, 'right', retrieval=True)['valid']
    assert not single_missing_floor_frame(cycle, 'right', metrics)['valid']
    waiting = pending_zero_force_frame(cycle)
    assert waiting and waiting['not_a_support_or_grip_certificate']
    assert waiting['observed_fingers'] == [1]
    assert waiting['contact_stamp'] - waiting['last_positive_basket_stamp'] == pytest.approx(.0005)
    assert cycle.contacts == before and not cycle.holders.rows


@pytest.mark.parametrize('change', [
    'disabled', 'holder', 'not_closing', 'phase', 'payload', 'location', 'unaccepted',
    'wrong_claim', 'target', 'effort', 'old_positive', 'future_positive', 'old_joint',
    'old_contact', 'late_profile', 'before_start', 'nan_joint', 'nan_clock', 'no_pairs',
    'force_positive', 'force_nan', 'force_negative', 'foreign', 'depth',
    'positions', 'wrong_band', 'nonfinite_position', 'above_floor', 'sidewall',
])
def test_unqualified_cases_do_not_get_observation_wait(change):
    cycle, _, _ = actual_fixture()
    history = cycle.closing_floor_history.rows['right']
    pair = cycle.contacts['right'][0]
    if change == 'disabled': cycle.remaining_guard_enabled = False
    elif change == 'holder': cycle.holders.rows['left'] = {'payload': 'left'}
    elif change == 'not_closing': cycle.retrieval_closing = False
    elif change == 'phase': cycle.phase = 'seq_retrieval_single_pilot'
    elif change == 'payload': cycle.grasp_candidate = 'left'
    elif change == 'location': cycle.order.locations['right'] = 'held'
    elif change == 'unaccepted': cycle.pair_goal_handles['right'].accepted = False
    elif change == 'wrong_claim': cycle.preload_grasp_claim['payload'] = 'left'
    elif change == 'target': cycle.accepted_gripper_targets['right'] = 0.
    elif change == 'effort': cycle.accepted_gripper_efforts['right'] = 18.
    elif change == 'old_positive': history['positive'] -= .0005
    elif change == 'future_positive': history['positive'] = history['stamp'] + .0005
    elif change == 'old_joint': cycle.joint_time = cycle.sim - .013
    elif change == 'old_contact': cycle.sim += .002
    elif change == 'late_profile': cycle.preload_grasp_claim['accepted_common_start_sim'] = cycle.sim - 2.1
    elif change == 'before_start': cycle.preload_grasp_claim['accepted_common_start_sim'] = cycle.sim + .001
    elif change == 'nan_joint': cycle.positions['right_dex1_finger_1_joint'] = float('nan')
    elif change == 'nan_clock': cycle.sim = float('nan')
    elif change == 'no_pairs': cycle.contacts['right'] = []
    elif change == 'force_positive': pair['force'] = .001
    elif change == 'force_nan': pair['force'] = float('nan')
    elif change == 'force_negative': pair['force'] = -.001
    elif change == 'foreign': pair['second'] = 'h1_2::left_dex1_finger_1_link::collision'
    elif change == 'depth': pair['depth'] = .002
    elif change == 'positions': pair['positions'] = []
    elif change == 'wrong_band': pair['positions'][0][0] += .1
    elif change == 'nonfinite_position': pair['positions'][0][0] = float('nan')
    elif change == 'above_floor': cycle.poses['box48_right']['position']['z'] += .002
    elif change == 'sidewall': cycle.poses['box48_right']['position']['x'] += .05
    assert pending_zero_force_frame(cycle) is None


def test_bounded_synthetic_next_frame_recovery_is_not_a_new_physics_certificate():
    cycle, _, record = actual_fixture()
    initial = pending_zero_force_frame(cycle)
    guard_calls = []

    def pump():
        contact = copy.deepcopy(record['contacts_before'][-2]['payload'])
        contact['stamp'] = initial['contact_stamp'] + .0005
        cycle.sim = contact['stamp']
        cycle.contacts['right'] = contact['pairs']
        cycle.closing_floor_history.observe(contact)

    result = wait_for_observation(cycle, initial, pump, lambda: guard_calls.append(True), clock=lambda: 1.)
    assert result['callbacks'] == 1 and not result['still_missing']
    assert guard_calls == [True] and cycle.holders.rows == {}
    assert not actual_preload_support(cycle, 'right', retrieval=True)['valid']


def test_actual_post_fault_nine_ms_gap_is_rejected_not_claimed_as_recovery():
    cycle, _, record = actual_fixture()
    initial = pending_zero_force_frame(cycle)

    def pump():
        contact = record['contacts_after'][0]['payload']
        cycle.sim = contact['stamp']
        cycle.contacts['right'] = contact['pairs']
        cycle.closing_floor_history.observe(contact)

    with pytest.raises(RuntimeError, match='bounded simulation window'):
        wait_for_observation(cycle, initial, pump, lambda: None, clock=lambda: 1.)


def test_callback_budget_leaves_missing_observation_for_original_guard():
    cycle, _, _ = actual_fixture()
    result = wait_for_observation(cycle, pending_zero_force_frame(cycle), lambda: None, lambda: None, clock=lambda: 1.)
    assert result['callbacks'] == 64 and result['still_missing']


def test_wall_budget_leaves_missing_observation_for_original_guard():
    cycle, _, _ = actual_fixture()
    clock_values = iter([1., 1.021, 1.022])
    result = wait_for_observation(cycle, pending_zero_force_frame(cycle), lambda: None, lambda: None,
                                  clock=lambda: next(clock_values))
    assert result['callbacks'] == 0 and result['still_missing']


def test_original_core_guard_failure_propagates():
    cycle, _, _ = actual_fixture()

    def guard():
        raise RuntimeError('original force/depth guard')

    with pytest.raises(RuntimeError, match='original force'):
        wait_for_observation(cycle, pending_zero_force_frame(cycle), lambda: None, guard, clock=lambda: 1.)


def test_runtime_still_calls_original_full_safety_after_wait(monkeypatch):
    import preload_observation
    calls = []

    class Original:
        def safety(self):
            calls.append('original_full_safety')
            raise RuntimeError('original support still invalid')

    class Candidate(PreloadObservationMixin, Original):
        def record(self, kind, payload):
            calls.append(kind)

    monkeypatch.setattr(preload_observation, 'pending_zero_force_frame', lambda cycle: {'payload': 'right'})
    monkeypatch.setattr(preload_observation, 'wait_for_observation', lambda *args: {})
    with pytest.raises(RuntimeError, match='original support still invalid'):
        Candidate().safety()
    assert calls == ['preload_observation_wait_started', 'original_full_safety']


def test_original_support_capture_geometry_sources_unchanged():
    parent = ROOT.parent / 'centrifuge_48box_sequential_full_20261020_r330'
    for name in ('sequential_cycle.py', 'preload_floor_frame.py', 'sequential_support.py',
                 'sequential_force.py', 'execute_dual.py', 'execute_coherent.py',
                 'sequential_geometry.py', 'speed_retrieval_geometry.py', 'speed_policy.py'):
        from pipeline_source import restore_pipeline_source
        assert restore_pipeline_source(name, (ROOT / name).read_text()) == (parent / name).read_text().replace(str(parent), str(ROOT)), name
    from sequential_speed_cycle import SpeedPriorityCycle
    assert SpeedPriorityCycle.safety is PreloadObservationMixin.safety

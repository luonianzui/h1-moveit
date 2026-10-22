import copy

import numpy as np
import pytest

from dual_motion import HoldRegistry, TASK_JOINTS, common_arm_timing, validate_common_start


def certificate():
    return dict(source='gazebo_physics_contact', midband_verified=True, normal_loads_N={'1': 16., '2': 16.})


def test_two_real_holders_have_separate_immutable_native_and_fk_references():
    registry = HoldRegistry()
    reference = np.eye(4)
    registry.begin('left', 'left', reference, reference, 3., certificate())
    reference[0, 3] = .02
    registry.begin('right', 'right', reference, reference, 3., certificate())
    assert registry.rows['left']['native_reference'][0, 3] == 0
    assert registry.rows['right']['native_reference'][0, 3] == .02
    with pytest.raises(ValueError):
        registry.rows['left']['native_reference'][0, 3] = 99
    registry.release('left')
    assert set(registry.snapshot()) == {'right'}


@pytest.mark.parametrize('hand,payload', [('unknown', 'left'), ('left', 'unknown')])
def test_unknown_holder_or_payload_is_rejected(hand, payload):
    with pytest.raises(ValueError):
        HoldRegistry().begin(hand, payload, np.eye(4), np.eye(4), 1., certificate())


def test_grip_reference_cannot_be_rebased_while_holding_or_claimed_by_other_hand():
    registry = HoldRegistry()
    registry.begin('left', 'right', np.eye(4), np.eye(4), 1., certificate())
    with pytest.raises(ValueError):
        registry.begin('left', 'right', np.eye(4), np.eye(4), 2., certificate())
    with pytest.raises(ValueError):
        registry.begin('right', 'right', np.eye(4), np.eye(4), 2., certificate())


@pytest.mark.parametrize('field,value', [('source', 'command'), ('midband_verified', False),
                                       ('normal_loads_N', {'1': 16.}),
                                       ('normal_loads_N', {'1': 2.99, '2': 16.}),
                                       ('normal_loads_N', {'1': 16., '2': 20.51}),
                                       ('normal_loads_N', {'1': 16., '2': float('nan')})])
def test_both_actual_contact_and_original_preload_gates_are_required(field, value):
    proof = certificate()
    proof[field] = value
    with pytest.raises(ValueError):
        HoldRegistry().begin('left', 'left', np.eye(4), np.eye(4), 1., proof)


@pytest.mark.parametrize('value', [np.eye(3), np.eye(4) * np.nan, np.diag([1., 1., -1., 1.])])
def test_nonfinite_nonrigid_or_reflected_held_reference_is_rejected(value):
    with pytest.raises(ValueError):
        HoldRegistry().begin('left', 'left', value, np.eye(4), 1., certificate())


def test_releasing_one_holder_does_not_preclaim_other_hand_empty():
    registry = HoldRegistry()
    with pytest.raises(ValueError):
        registry.release('left')
    for side in ('left', 'right'):
        registry.begin(side, side, np.eye(4), np.eye(4), 1., certificate())
    registry.release('right')
    assert set(registry.rows) == {'left'}


def test_torso_and_two_arms_get_identical_timestamps_and_shared_max_joint_budget():
    actual = dict.fromkeys(sorted(TASK_JOINTS), 0.)
    target = copy.deepcopy(actual)
    target['left_elbow_joint'] = .2
    target['right_elbow_joint'] = -.3
    timing = common_arm_timing(actual, [actual, target], .25)
    assert timing['times'] == pytest.approx([.1, .12, 1.32])
    assert timing['duration_sim_s'] == pytest.approx(1.32)
    assert set(timing['groups']) == {'torso', 'left_arm', 'right_arm'}


@pytest.mark.parametrize('bad', ['missing_right', 'extra_leg', 'nan', 'missing_second'])
def test_partial_leg_or_nonfinite_command_is_rejected(bad):
    actual = dict.fromkeys(TASK_JOINTS, 0.)
    samples = [dict(actual), dict(actual)]
    if bad == 'missing_right':
        samples[0].pop('right_elbow_joint')
    elif bad == 'extra_leg':
        samples[0]['left_knee_joint'] = 0.
    elif bad == 'nan':
        samples[1]['left_elbow_joint'] = float('nan')
    else:
        samples[1].pop('left_elbow_joint')
    with pytest.raises(ValueError):
        common_arm_timing(actual, samples, .25)


@pytest.mark.parametrize('current,start', [(1., 1.1), (float('nan'), 2.), (1., float('inf')), (-1., 2.)])
def test_missed_or_invalid_common_start_is_refused(current, start):
    with pytest.raises(ValueError):
        validate_common_start(current, start)


def test_common_start_preserves_subsecond_sim_stamp():
    assert validate_common_start(10., 10.3) == (10, 300000000)

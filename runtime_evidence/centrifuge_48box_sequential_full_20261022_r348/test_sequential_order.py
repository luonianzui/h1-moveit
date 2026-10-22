import math

import pytest

from sequential_order import SequentialOrder, finite_duration


def holder(hand, payload=None):
    return {hand: dict(payload=payload or hand)}


def load_both():
    order = SequentialOrder()
    for hand in ('left', 'right'):
        order.begin(hand, hand, holder(hand))
        order.finish_loading(hand, hand, {}, 3.)
    return order


def stopped():
    return dict(rotor_rpm=0., rotor_rad_s=0., lid_rad=math.radians(120), rotor_enabled=False,
                can_open=True, starts=1, stops=1)


def test_complete_sequential_order():
    order = load_both()
    order.finish_machine({}, 30., stopped(), dict(left='right', right='left'))
    for hand, payload in (('left', 'right'), ('right', 'left')):
        order.begin(hand, payload, holder(hand, payload))
        order.finish_return(hand, payload, {}, 3.)
    order.finish_terminal({}, 10.)
    assert order.snapshot()['phase'] == 'complete'
    assert order.loaded == ['left', 'right']
    assert order.returned == ['left', 'right']


@pytest.mark.parametrize('hand,payload', [('right', 'right'), ('left', 'right'), ('unknown', 'left')])
def test_first_pickup_order_cannot_change(hand, payload):
    with pytest.raises(ValueError):
        SequentialOrder().begin(hand, payload, holder(hand, payload))


def test_no_second_hand_while_first_holds():
    order = SequentialOrder()
    order.begin('left', 'left', holder('left'))
    with pytest.raises(ValueError):
        order.begin('right', 'right', holder('right'))


@pytest.mark.parametrize('rows', [{}, {'right': dict(payload='left')},
                                 {'left': dict(payload='left'), 'right': dict(payload='right')}])
def test_exact_single_actual_holder_required(rows):
    with pytest.raises(ValueError):
        SequentialOrder().begin('left', 'left', rows)


def test_drop_requires_release_and_original_three_seconds():
    order = SequentialOrder()
    order.begin('left', 'left', holder('left'))
    for rows, seconds in [(holder('left'), 3.), ({}, 2.999), ({}, float('nan'))]:
        with pytest.raises(ValueError):
            order.finish_loading('left', 'left', rows, seconds)
    assert order.holder == dict(hand='left', payload='left')


def test_machine_cannot_run_after_only_one_box():
    order = SequentialOrder()
    order.begin('left', 'left', holder('left'))
    order.finish_loading('left', 'left', {}, 3.)
    with pytest.raises(ValueError):
        order.finish_machine({}, 30., stopped(), dict(left='left', right='right'))


@pytest.mark.parametrize('change', [dict(rotor_enabled=True), dict(rotor_rpm=.021), dict(rotor_rpm=float('nan')),
                                  dict(lid_rad=0.), dict(can_open=False), dict(starts=2), dict(stops=0),
                                  dict(rotor_rad_s=.02), dict(rotor_rad_s=float('nan'))])
def test_original_machine_gates_preserved(change):
    order = load_both()
    with pytest.raises(ValueError):
        order.finish_machine({}, 30., dict(stopped(), **change), dict(left='left', right='right'))


@pytest.mark.parametrize('duration', [29.999, float('nan'), float('inf'), -1.])
def test_original_thirty_second_spin_preserved(duration):
    with pytest.raises(ValueError):
        load_both().finish_machine({}, duration, stopped(), dict(left='left', right='right'))


def test_single_box_or_duplicate_retrieval_assignment_rejected():
    for assignment in [dict(left='left'), dict(left='left', right='left')]:
        with pytest.raises(ValueError):
            load_both().finish_machine({}, 30., stopped(), assignment)


def test_machine_field_names_match_original_live_interface():
    import json
    from pathlib import Path
    root = Path(__file__).resolve().parent
    source = root.parent / 'centrifuge_48box_elbow_full_20261016_r291/run/result.json'
    observed = json.loads(source.read_text())['final_machine']
    order = load_both()
    order.finish_machine({}, 30., observed, dict(left='left', right='right'))
    assert order.phase == 'retrieval'


def test_terminal_requires_two_returns():
    order = load_both()
    order.finish_machine({}, 30., stopped(), dict(left='left', right='right'))
    order.begin('left', 'left', holder('left'))
    order.finish_return('left', 'left', {}, 3.)
    with pytest.raises(ValueError):
        order.finish_terminal({}, 10.)


def test_snapshots_are_not_mutable_runtime_aliases():
    order = SequentialOrder()
    snapshot = order.snapshot()
    snapshot['locations']['left'] = 'basket'
    assert order.locations['left'] == 'table'


@pytest.mark.parametrize('minimum', [3., 10., 30.])
def test_duration_must_be_finite(minimum):
    with pytest.raises(ValueError):
        finite_duration(float('inf'), minimum)

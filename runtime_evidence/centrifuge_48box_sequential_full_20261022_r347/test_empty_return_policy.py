import inspect

import pytest

from empty_return_policy import skip_verified_empty_return
from execute_cycle import FullCycle


def eligible(**changes):
    arguments = dict(is_last_payload=False, phase='return', holding=None,
                     held_payload=None, table_verified=True, retreat_complete=True)
    arguments.update(changes)
    return skip_verified_empty_return(**arguments)


def test_only_first_empty_return_after_verified_table_and_full_retreat_is_skipped():
    assert eligible()


@pytest.mark.parametrize('phase', ['approach', 'descend', 'pilot_lift', 'lift',
                                   'transfer', 'setdown', 'retreat', 'unknown'])
def test_no_material_or_required_retreat_motion_is_skipped(phase):
    assert not eligible(phase=phase)


@pytest.mark.parametrize('changes', [
    dict(is_last_payload=True), dict(holding='left'), dict(holding='right'),
    dict(held_payload='left'), dict(held_payload='right'),
    dict(table_verified=False), dict(retreat_complete=False),
    dict(table_verified=1), dict(retreat_complete=1),
])
def test_held_or_unverified_table_or_retreat_cannot_skip(changes):
    assert not eligible(**changes)


def test_table_verified_only_after_full_stability_event():
    source = inspect.getsource(FullCycle.verify_table)
    assert source.index('while self.sim - start < 3:') < source.index("self.record('table_return_pass'")
    assert source.index("self.record('table_return_pass'") < source.index('self.verified_table_payloads.add(payload)')


def test_execution_keeps_retreat_and_original_last_parking_policy():
    source = inspect.getsource(FullCycle.retrieve)
    assert 'skip_final_parking(payload == order[-1], phase)' in source
    assert 'skip_verified_empty_return(' in source
    assert source.index('self.move(stage[') < source.index("if phase == 'retreat':")
    assert 'self.plan_retrieval(hand, payload' in source
    assert 'self.terminal_hold(10)' in source


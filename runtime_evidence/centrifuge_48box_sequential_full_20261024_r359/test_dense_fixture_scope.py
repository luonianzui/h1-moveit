import copy
from pathlib import Path

import pytest

from dense_fixture_scope import declare_prospective_empty_holder_scope


EVIDENCE = Path(__file__).resolve().parent


def fixture():
    return dict(mode='retrieval_pickup', retrieval=True,
                nominal_phase_fixture_is_prospective_not_actual_new_stop=True,
                positions={'joint': .123}, poses={'box': {'position': {'x': .5}}})


@pytest.mark.parametrize('declared', [False, True])
def test_only_labeled_prospective_empty_scope_is_added_without_physical_state_changes(declared):
    current = fixture()
    if declared:
        current['original_runtime_holder_references'] = {}
    original = copy.deepcopy(current)
    result = declare_prospective_empty_holder_scope(current)
    assert current == original and result is not current
    assert result['positions'] == original['positions'] and result['poses'] == original['poses']
    assert result['original_runtime_holder_references'] == {}
    assert result['prospective_empty_holder_registry_not_measured']
    assert result['not_current_fresh_execution_permission']


@pytest.mark.parametrize('invalid', [{'mode': 'load_pickup'}, {'retrieval': False},
                                   {'nominal_phase_fixture_is_prospective_not_actual_new_stop': False},
                                   {'original_runtime_holder_references': {'left': 'box'}},
                                   {'original_runtime_holder_references': None}])
def test_actual_or_held_or_unknown_scope_is_never_normalized_to_empty(invalid):
    with pytest.raises(ValueError):
        declare_prospective_empty_holder_scope(dict(fixture(), **invalid))


def test_runtime_executor_does_not_import_or_use_the_prospective_fixture_helper():
    source = (EVIDENCE / 'execute_dual.py').read_text()
    assert 'declare_prospective_empty_holder_scope' not in source
    assert 'original_runtime_holder_references=self.holders.snapshot()' in source

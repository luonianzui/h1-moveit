import json
from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r356'

@pytest.mark.parametrize('index', [0, 1])
def test_bit_exact_repeated_contact_worlds(index):
    record = json.loads((ROOT / 'dart_cache_dependencies.json').read_text())['lookup_qualification'][index]
    assert record['status'] == 'PASS_EXACT_LOOKUP_NOT_FULL_TASK'
    assert record['world_steps'] == 5400 and record['vector_checks'] == 43200
    assert record['contact_steps'] >= 5000
    assert record['lookup_hits'] > 10000 and record['lookup_misses'] > 1000
    assert record['lookup_retained'] == record['retained_after_solve'] == 0
    assert record['component_speedup'] > 1.

def test_scope_and_exception():
    record = json.loads((ROOT / 'dart_cache_dependencies.json').read_text())['lookup_scope_qualification']
    assert record['checks'] == 1296 and record['mode_rejections'] == 31
    assert record['exception_scope_cleared'] and record['dirty_refreshes'] >= 60
    assert record['lookup_retained'] == record['retained'] == 0

def test_lookup_scoped_collision_fallback():
    source = (ROOT / 'inputs/CachedSolver.cc').read_text()
    assert source.count('lookupSlots.fill({});') == 2
    assert source.count('fastBodyQuery(') == 4
    assert 'if (!lookupEnabled)\n    return queryBody(body);' in source
    assert 'if (slot.body != body)' in source
    assert 'slot.query = &queryBody(body);' in source
    assert 'thread_local std::array<InertiaSlot, 256> lookupSlots;' in source

@pytest.mark.parametrize('start,finish', [
    ('const dart::dynamics::ConstSkeletonPtr& inertiaOwner', 'struct QueryScope'),
    ('const math::Inertia& BodyNode::getArticulatedInertia()', 'bool BodyNode::isReactive()'),
    ('  const std::size_t constraintCount', '    const auto completed ='),
])
def test_arithmetic_and_dirty_checks_byte_identical(start, finish):
    source = (ROOT / 'inputs/CachedSolver.cc').read_text()
    parent = (PARENT / 'inputs/CachedSolver.cc').read_text()
    assert source[source.index(start):source.index(finish)] == parent[parent.index(start):parent.index(finish)]

def test_contact_response_only_changes_lookup():
    source = (ROOT / 'inputs/CachedSolver.cc').read_text()
    parent = (PARENT / 'inputs/CachedSolver.cc').read_text()
    start, finish = 'void ContactConstraint::getVelocityChange', '\nnamespace {\nusing Clock'
    actual = source[source.index(start):source.index(finish)].replace('fastBodyQuery(', 'queryBody(')
    assert actual == parent[parent.index(start):parent.index(finish)]

@pytest.mark.parametrize('name', ['sequential_cycle.py', 'sequential_speed_cycle.py', 'preload_observation.py',
    'sequential_support.py', 'table_alignment_cycle.py', 'pipeline_source.py', 'unilateral_source.py'])
def test_no_action_guard_or_hold_change(name):
    expected = (PARENT / name).read_text().replace(PARENT.name, ROOT.name).replace(
        'install_lab_scene_48box_sequential_full_20261023_r356', 'install_lab_scene_48box_sequential_full_20261023_r358')
    assert (ROOT / name).read_text() == expected

def test_r356_core_and_review_are_pinned():
    source = (ROOT / 'verify_r354_protection.py').read_text()
    assert '91127fe77596ad386d79a7c1763904d016e9a944b4a4ded7eef4b125f357de5d' in source
    assert '702e0af1bc9fa34e1b997b6ac6ea1f3ec0027186a399498593fcff356d67c17c' in source

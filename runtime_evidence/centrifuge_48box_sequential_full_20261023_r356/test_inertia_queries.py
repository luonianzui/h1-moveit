import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent

def test_inertia_numeric_qualification():
    record = json.loads((ROOT / 'dart_cache_dependencies.json').read_text())['inertia_qualification']
    assert record['world_steps'] == 5400
    assert record['vector_checks'] == 43200
    assert record['inertia_queries'] > 10000
    assert record['retained_after_solve'] == 0
    assert record['component_speedup'] > 1.

def test_both_original_dirty_checks_preserved():
    content = (ROOT / 'inputs/CachedSolver.cc').read_text()
    assert content.count('owner->mTreeCache[mTreeIndex].mDirty.mArticulatedInertia') == 2
    assert content.count('owner->updateArticulatedInertia(mTreeIndex);') == 2
    assert content.count('if (!queriesActive || !inertiaEnabled)') == 2
    assert 'return mArtInertia;' in content
    assert 'return mArtInertiaImplicit;' in content
    assert 'nativeQuery<Function>' in content
    assert content.count('inertiaSlots.fill({});') == 2
    assert content.count('const auto& owner = inertiaOwner(this);') == 2

def test_dirty_invalidation_inside_solve():
    record = json.loads((ROOT / 'dart_cache_dependencies.json').read_text())['dirty_qualification']
    assert record['checks'] == 1266 and record['mode_rejections'] == 30
    assert record['dirty_refreshes'] >= 60
    assert record['retained'] == 0

def test_new_parent_protected():
    assert "read('r354_protection_before.json')['status'] == 'PASS'" in (ROOT / 'seal_sequential.py').read_text()
    assert 'b26e2f0e94230ddd26465fee3e6ac9da68ba4d71145647c7b1830d3741cedf5d' in (ROOT / 'verify_r354_protection.py').read_text()

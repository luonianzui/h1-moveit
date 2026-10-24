import ast
import bisect
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent

def interpolation():
    tree = ast.parse((ROOT / 'audit_initial_button_motion.py').read_text())
    function = next(item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == 'commanded')
    namespace = dict(bisect=bisect)
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<interpolation>', 'exec'), namespace)
    return namespace['commanded']

@pytest.mark.parametrize('relative,expected', [(-.1, None), (0., [0.]), (.5, [.5]), (1., [1.]), (1.1, None)])
def test_bounds_and_linear(relative, expected):
    points = [dict(seconds=0., positions=[0.], velocities=[]), dict(seconds=1., positions=[1.], velocities=[])]
    assert interpolation()(points, relative) == expected

def test_hermite():
    points = [dict(seconds=0., positions=[0.], velocities=[0.]), dict(seconds=1., positions=[1.], velocities=[0.])]
    assert interpolation()(points, .25) == [.15625]

def test_gzip_only_source_correction():
    parent = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r353'
    original = (parent / 'audit_initial_button_motion.py').read_text()
    expected = original.replace('import bisect', 'import bisect\nimport gzip').replace(
        "with (ROOT / 'run/loading_trace.jsonl').open() as stream:",
        "with gzip.open(ROOT / 'run/loading_trace.jsonl.gz', 'rt') as stream:")
    assert (ROOT / 'audit_initial_button_motion.py').read_text() == expected

def test_exact_query_qualification():
    qualification = json.loads((ROOT / 'dart_cache_dependencies.json').read_text())['queries_qualification']
    assert qualification['world_steps'] == 5400 and qualification['vector_checks'] == 43200
    assert qualification['retained_after_solve'] == 0 and qualification['hits'] > qualification['misses']
    assert qualification['responses'] > 10000

def test_scope_does_not_cache_velocity_or_impulse_flag():
    source = (ROOT / 'inputs/CachedSolver.cc').read_text()
    assert source.count('bodyQueries.clear();') == 2
    assert 'bodyA.owner->isImpulseApplied() && bodyA.reactive' in source
    assert 'bodyB.owner->isImpulseApplied() && bodyB.reactive' in source
    assert 'mBodyNodeA->getBodyVelocityChange()' in source
    assert 'mBodyNodeB->getBodyVelocityChange()' in source
    assert 'return queriesActive ? fastBodyQuery(this).reactive : nativeReactive(this);' in source
    assert 'QueryScope queryScope;' in source


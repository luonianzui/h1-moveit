import ast
import bisect
import json
from pathlib import Path

import pytest


REVIEW = Path(__file__).resolve().parent
ROOT = REVIEW.parent / 'centrifuge_48box_sequential_full_20261022_r353'


def interpolation():
    tree = ast.parse((REVIEW / 'audit_initial_button_motion_revision2.py').read_text())
    function = next(item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == 'commanded')
    namespace = dict(bisect=bisect)
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<audited_interpolation>', 'exec'), namespace)
    return namespace['commanded']


def test_exactly_gzip_reader_and_review_root_change():
    original = (ROOT / 'audit_initial_button_motion.py').read_text()
    expected = original.replace('import bisect', 'import bisect\nimport gzip')
    expected = expected.replace('ROOT = Path(__file__).resolve().parent',
                                "ROOT = Path(__file__).resolve().parent.parent / 'centrifuge_48box_sequential_full_20261022_r353'")
    expected = expected.replace("with (ROOT / 'run/loading_trace.jsonl').open() as stream:",
                                "with gzip.open(ROOT / 'run/loading_trace.jsonl.gz', 'rt') as stream:")
    assert (REVIEW / 'audit_initial_button_motion_revision2.py').read_text() == expected


@pytest.mark.parametrize('relative,expected', [(-.1, None), (0., [0.]), (.5, [.5]), (1., [1.]), (1.1, None)])
def test_linear_and_bounds(relative, expected):
    points = [dict(seconds=0., positions=[0.], velocities=[]), dict(seconds=1., positions=[1.], velocities=[])]
    assert interpolation()(points, relative) == expected


def test_cubic_hermite_zero_endpoint_velocity():
    points = [dict(seconds=0., positions=[0.], velocities=[0.]), dict(seconds=1., positions=[1.], velocities=[0.])]
    assert interpolation()(points, .25) == [.15625]


def test_actual_initial_withdraw_reference_is_observed():
    report = json.loads((ROOT / 'initial_button_motion_audit.json').read_text())
    phase = report['phases']['machine_preparation_open_withdraw']
    assert report['complete_task'] and phase['samples'] == 1626
    assert phase['max_feedback_reference_error_rad'] < .15
    assert phase['max_reference_command_error_rad'] < 1e-12
    assert 'FileNotFoundError' in (ROOT / 'post_audit_initial_button_motion.log').read_text()

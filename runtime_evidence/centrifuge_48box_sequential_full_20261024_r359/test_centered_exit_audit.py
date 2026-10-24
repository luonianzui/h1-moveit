import copy
import json
from pathlib import Path

import pytest

from audit_speed_paths import verify_return_height


ROOT = Path(__file__).resolve().parent


@pytest.fixture
def fixture():
    folder = ROOT.parent / 'centrifuge_48box_exit_clearance_20261024_r1'
    row = json.loads((folder / 'centered_exit_qualification.json').read_text())['records'][0]
    return json.loads(Path(row['source']).read_text()), row['plan']


def test_actual_corrected_path_geometry_contract(fixture):
    request, planned = fixture
    verify_return_height(request, planned)


@pytest.mark.parametrize('field,value', [('no_basket_collision_exemption', False),
    ('original_holder_reference_unchanged', False), ('orientation_unchanged', False),
    ('full_scene_dense_checked', False), ('maximum_correction_m', .003),
    ('translation_world_m', [0., 0., 0.]), ('original_axis_rejection', 'Unrelated error')])
def test_broader_or_fabricated_corrections_rejected(fixture, field, value):
    request, planned = fixture
    changed = copy.deepcopy(planned)
    changed['stages'][0]['clearance_correction'][field] = value
    with pytest.raises(ValueError, match='Undeclared or unbounded'):
        verify_return_height(request, changed)


def test_undeclared_transverse_motion_rejected(fixture):
    request, planned = fixture
    changed = copy.deepcopy(planned)
    changed['stages'][0]['last_body'][1][3] += .001
    with pytest.raises(ValueError, match='preserve actual transverse'):
        verify_return_height(request, changed)

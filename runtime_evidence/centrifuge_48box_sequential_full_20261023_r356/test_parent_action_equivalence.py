from acceleration_source import restore_acceleration, restore_acceleration_bytes
import hashlib
from test_return_height_inverse import undo_return_height
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261017_r296'


@pytest.mark.parametrize('name', [
    'sequential_force.py', 'sequential_geometry.py', 'sequential_planning.py',
    'sequential_worker.py', 'sequential_order.py', 'execute_sequential.py', 'execute_dual.py'])
def test_new_identity_keeps_unfinished_vertical_first_action(name):
    identity = json.loads((PARENT / 'identity.json').read_text())['sha256']
    original = (PARENT / name).read_bytes()
    assert hashlib.sha256(original).hexdigest() == identity[name]
    current = restore_acceleration(name, (ROOT / name).read_text()).replace(str(ROOT), str(PARENT)).replace(ROOT.name, PARENT.name)
    current = current.replace('install_lab_scene_48box_sequential_full_20261023_r356', 'install_lab_scene_48box_sequential_full_20261017_r296')
    current = current.replace('h1_seq_full_r337', 'h1_seq_full_r296').replace(
        'centrifuge_48box_sequential_full_r349', 'centrifuge_48box_sequential_full_r296')
    if name == 'sequential_planning.py':
        current = undo_return_height(current)
        current = current.replace('from load_bound_reserve import InteriorLoadPlanner\n', '')
        current = current.replace('planner = InteriorLoadPlanner(request, reference)', 'planner = SinglePlanner(request, reference)')
        current = current.replace('from pilot_centering import centered_pilot_target\n', '')
        current = current.replace("    if retrieval:\n        target = centered_pilot_target(body, request['poses'], request['payload'])\n", '')
    if name == 'sequential_force.py':
        current = current.replace('RETRIEVAL_FORCE_N = 16.5', 'RETRIEVAL_FORCE_N = 17.5')
    if name == 'sequential_worker.py':
        current = current.replace('from release_alignment import plan_release_alignment\n', '')
        current = current.replace(", 'load_release_align'}", "}")
        current = current.replace("        if mode == 'load_release_align':\n            result = plan_release_alignment(request, reference)\n        elif mode.endswith('_pilot'):", "        if mode.endswith('_pilot'):")
    assert current.encode() == original

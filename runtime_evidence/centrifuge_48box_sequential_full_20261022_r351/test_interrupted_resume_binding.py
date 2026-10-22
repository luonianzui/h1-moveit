from acceleration_source import restore_acceleration, restore_acceleration_bytes
import hashlib
from test_return_height_inverse import undo_return_height
import json
from pathlib import Path

import pytest
from test_frame_source_inverse import undo_frame_integration


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261019_r304'


@pytest.mark.parametrize('name', [
    'sequential_force.py', 'sequential_geometry.py', 'sequential_planning.py',
    'sequential_worker.py', 'sequential_order.py', 'sequential_support.py',
    'sequential_cycle.py', 'execute_sequential.py', 'camera_pose_guard.py'])
def test_interrupted_parent_action_and_recorder_are_preserved(name):
    identity_path = PARENT / 'identity.json'
    assert hashlib.sha256(identity_path.read_bytes()).hexdigest() == 'da852057ed35e8f76e67711a31f54e1fad01d18b92f96a3cf31566a166ebd4fe'
    original = (PARENT / name).read_bytes()
    assert hashlib.sha256(original).hexdigest() == json.loads(identity_path.read_text())['sha256'][name]
    current = restore_acceleration(name, (ROOT / name).read_text()).replace(str(ROOT), str(PARENT)).replace(ROOT.name, PARENT.name)
    current = current.replace('install_lab_scene_48box_sequential_full_20261022_r351', 'install_lab_scene_48box_sequential_full_20261019_r304')
    current = current.replace('h1_seq_full_r337', 'h1_seq_full_r304').replace(
        'centrifuge_48box_sequential_full_r349', 'centrifuge_48box_sequential_full_r304')
    if name == 'sequential_planning.py':
        current = undo_return_height(current)
        current = current.replace('from load_bound_reserve import InteriorLoadPlanner\n', '')
        current = current.replace('planner = InteriorLoadPlanner(request, reference)', 'planner = SinglePlanner(request, reference)')
        current = current.replace('from pilot_centering import centered_pilot_target\n', '')
        current = current.replace("    if retrieval:\n        target = centered_pilot_target(body, request['poses'], request['payload'])\n", '')
    if name == 'sequential_worker.py':
        current = current.replace('from release_alignment import plan_release_alignment\n', '')
        current = current.replace(", 'load_release_align'}", "}")
        current = current.replace("        if mode == 'load_release_align':\n            result = plan_release_alignment(request, reference)\n        elif mode.endswith('_pilot'):", "        if mode.endswith('_pilot'):")
    if name == 'sequential_cycle.py':
        current = undo_frame_integration(current)
        current = current.replace("        if phase == 'seq_retrieval_retrieve_axis_exit':\n            return min(requested, .03), .002\n", '')
        current = current.replace('from release_alignment import align_before_release\n', '')
        current = current.replace('            align_before_release(self, hand, hand)\n', '')
    assert current.encode() == original

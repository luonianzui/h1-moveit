from acceleration_source import restore_acceleration, restore_acceleration_bytes
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from load_bound_reserve import InteriorLoadPlanner, LOAD_IK_RESERVE_RAD
from sequential_geometry import SinglePlanner
from test_frame_source_inverse import undo_frame_integration


def fixture():
    planner = InteriorLoadPlanner.__new__(InteriorLoadPlanner)
    planner.lower = np.array([-1., -2., -3.])
    planner.upper = np.array([1., 2., 3.])
    planner.indices = [0, 2]
    return planner


@pytest.mark.parametrize('refine', [True, False])
def test_only_active_ik_bounds_tightened_and_original_check_bounds_restored(monkeypatch, refine):
    planner = fixture()
    original_lower, original_upper = planner.lower, planner.upper
    def observed(self, body, seed, **keywords):
        assert np.array_equal(self.lower, [-.999, -2., -2.999])
        assert np.array_equal(self.upper, [.999, 2., 2.999])
        assert keywords == dict(refine=refine)
        return seed
    monkeypatch.setattr(SinglePlanner, 'solve', observed)
    seed = dict(joint=.1)
    assert planner.solve(np.eye(4), seed, refine=refine) is seed
    assert planner.lower is original_lower and planner.upper is original_upper
    assert LOAD_IK_RESERVE_RAD == .001


def test_ik_failure_still_restores_original_bounds(monkeypatch):
    planner = fixture()
    original_lower, original_upper = planner.lower, planner.upper
    def fail(*args, **kwargs):
        raise RuntimeError('IK rejected')
    monkeypatch.setattr(SinglePlanner, 'solve', fail)
    with pytest.raises(RuntimeError, match='IK rejected'):
        planner.solve(np.eye(4), {})
    assert planner.lower is original_lower and planner.upper is original_upper


def test_original_joint_guard_and_other_actions_unchanged():
    root = Path(__file__).resolve().parent
    parent = root.parent / 'centrifuge_48box_sequential_full_20261019_r308'
    identity = json.loads((parent / 'identity.json').read_text())['sha256']
    for name in ('sequential_geometry.py', 'release_alignment.py', 'sequential_cycle.py',
                 'sequential_force.py', 'pilot_centering.py', 'sequential_worker.py'):
        source = restore_acceleration(name, (root / name).read_text())
        if name == 'sequential_cycle.py':
            source = undo_frame_integration(source)
        assert hashlib.sha256(source.encode()).hexdigest() == identity[name]
    current = (root / 'sequential_planning.py').read_text()
    from test_return_height_inverse import undo_return_height
    current = undo_return_height(current)
    assert current.count('planner = InteriorLoadPlanner(request, reference)') == 1
    restored = current.replace('from load_bound_reserve import InteriorLoadPlanner\n', '')
    restored = restored.replace('planner = InteriorLoadPlanner(request, reference)', 'planner = SinglePlanner(request, reference)')
    assert hashlib.sha256(restored.encode()).hexdigest() == identity['sequential_planning.py']

import copy
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from bounded_release_alignment import align_before_release, correction_target
from release_alignment import release_target
from test_release_alignment import fixture, poses


@pytest.mark.parametrize('angle', [.01, .08, .269, 1., 1.001, 1.2328127, 1.9, 2.])
def test_each_correction_stays_within_original_single_step_envelope(angle):
    source = poses(angle)
    original = copy.deepcopy(source)
    metrics, target, step = correction_target(source, 'left')
    assert source == original
    assert 0 <= step <= 1. + 1e-10
    assert np.allclose(target[:3, 3], release_target(source, 'left')[:3, 3])
    if metrics['angle_deg'] > 1.:
        assert step == pytest.approx(.9)
    else:
        assert target == pytest.approx(release_target(source, 'left'))


@pytest.mark.parametrize('angle,shift', [(2.001, 0.), (.1, .001001), (float('nan'), 0.)])
def test_outside_total_envelope_rejected(angle, shift):
    with pytest.raises(ValueError):
        correction_target(poses(angle, shift), 'left')


def test_actual_size_error_is_corrected_twice_without_release_or_reanchoring():
    cycle = fixture(1.2328127)
    before = cycle.holders.snapshot()
    def execute(plan, prefix):
        cycle.commands.append((plan, prefix))
        metrics, target, step = correction_target(cycle.poses, 'left')
        cycle.poses['box48_left']['orientation'] = dict(zip('xyzw', Rotation.from_matrix(target[:3, :3]).as_quat()))
    cycle.execute_stages = execute
    align_before_release(cycle, 'left', 'left')
    assert len(cycle.commands) == 2
    assert cycle.waits == [.3, .3]
    assert cycle.holders.snapshot() == before
    assert cycle.records[-1][1]['metrics']['within_alignment_goal']


def test_no_progress_stops_before_release():
    cycle = fixture(1.23)
    cycle.execute_stages = lambda plan, prefix: cycle.commands.append(plan)
    with pytest.raises(RuntimeError, match='two measured'):
        align_before_release(cycle, 'left', 'left')
    assert len(cycle.commands) == 2


def test_anchor_replacement_stops_before_second_correction():
    cycle = fixture(1.23)
    cycle.execute_stages = lambda plan, prefix: setattr(cycle.holders, 'snapshot', lambda: {})
    with pytest.raises(RuntimeError, match='anchors'):
        align_before_release(cycle, 'left', 'left')


def test_no_motion_for_already_aligned_pose():
    cycle = fixture(.04)
    align_before_release(cycle, 'left', 'left')
    assert not cycle.commands and not cycle.waits


def test_only_import_selection_changes_original_runtime_and_planner():
    root = Path(__file__).resolve().parent
    parent = root.parent / 'centrifuge_48box_sequential_full_20261021_r335'
    for name in ('sequential_cycle.py', 'sequential_worker.py'):
        current = (root / name).read_text()
        assert current.count('from bounded_release_alignment import ') == 1
        restored = current.replace('from bounded_release_alignment import ', 'from release_alignment import ')
        assert restored == (parent / name).read_text().replace(parent.name, root.name)
    assert (root / 'release_alignment.py').read_bytes() == (parent / 'release_alignment.py').read_bytes()

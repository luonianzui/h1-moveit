import copy
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from release_alignment import align_before_release, release_error, release_target


def poses(angle=.269, shift=0.):
    return dict(centrifuge=dict(position=dict(x=.52, z=.815), orientation=dict(w=1.)),
                bucket_left=dict(position=dict(x=.003, y=.0857, z=.2375), orientation=dict(w=1.)),
                box48_left=dict(position=dict(x=.523 + shift, y=.0857, z=1.07525),
                                orientation=dict(zip('xyzw', (Rotation.from_euler('z', .309270119) *
                                    Rotation.from_euler('y', angle, degrees=True)).as_quat()))))


def fixture(angle=.269):
    holders = {'left': dict(payload='left', native_reference='unchanged_native', fk_reference='unchanged_fk')}
    cycle = SimpleNamespace(poses=poses(angle), holders=SimpleNamespace(snapshot=lambda: copy.deepcopy(holders)),
                            safety=lambda: None, commands=[], records=[], waits=[])
    cycle.record = lambda kind, data: cycle.records.append((kind, data))
    cycle.dwell = lambda duration: cycle.waits.append(duration)
    cycle.single_plan = lambda mode, hand, payload: dict(mode=mode, hand=hand, payload=payload)
    def execute(plan, prefix):
        cycle.commands.append((plan, prefix))
        cycle.poses = poses(0.)
    cycle.execute_stages = execute
    return cycle


def test_original_measured_small_release_error_requests_real_correction():
    cycle = fixture()
    original = cycle.holders.snapshot()
    align_before_release(cycle, 'left', 'left')
    assert cycle.commands == [(dict(mode='load_release_align', hand='left', payload='left'), 'seq_load')]
    assert cycle.waits == [.3]
    assert cycle.holders.snapshot() == original
    assert cycle.records[-1][1]['metrics']['within_alignment_goal']


def test_aligned_pose_does_not_request_unnecessary_motion():
    cycle = fixture(.04)
    align_before_release(cycle, 'left', 'left')
    assert not cycle.commands and not cycle.waits


@pytest.mark.parametrize('angle,shift', [(1.001, 0.), (.1, .001001)])
def test_large_errors_are_rejected_before_planning(angle, shift):
    cycle = fixture(angle)
    cycle.poses = poses(angle, shift)
    with pytest.raises(RuntimeError, match='envelope'):
        align_before_release(cycle, 'left', 'left')
    assert not cycle.commands


def test_correction_attempts_are_bounded_and_never_release():
    cycle = fixture()
    cycle.execute_stages = lambda plan, prefix: cycle.commands.append((plan, prefix))
    align_before_release(cycle, 'left', 'left')
    assert len(cycle.commands) == 2 and cycle.waits == [.3, .3]
    assert not cycle.records[-1][1]['metrics']['within_alignment_goal']


def test_holder_changes_during_correction_are_rejected():
    cycle = fixture()
    def replace(plan, prefix):
        cycle.holders = SimpleNamespace(snapshot=lambda: {})
    cycle.execute_stages = replace
    with pytest.raises(RuntimeError, match='anchors'):
        align_before_release(cycle, 'left', 'left')


@pytest.mark.parametrize('holders', [{}, {'right': dict(payload='left')}, {'left': dict(payload='right')}])
def test_exact_current_hand_and_payload_are_required(holders):
    cycle = fixture()
    cycle.holders = SimpleNamespace(snapshot=lambda: holders)
    with pytest.raises(ValueError, match='matching'):
        align_before_release(cycle, 'left', 'left')


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_measured_translation_is_rejected(value):
    source = poses()
    source['box48_left']['position']['x'] = value
    with pytest.raises(ValueError):
        release_error(source, 'left')


def test_target_matches_original_fixed_release_orientation_and_height():
    target = release_target(poses(), 'left')
    assert np.allclose(target[:3, 3], [.523, .0857, 1.07525], atol=1e-12, rtol=0.)
    assert np.allclose(target[:3, :3], Rotation.from_euler('z', .309270119).as_matrix(), atol=1e-12, rtol=0.)

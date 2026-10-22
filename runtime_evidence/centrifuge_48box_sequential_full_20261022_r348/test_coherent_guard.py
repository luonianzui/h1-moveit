from collections import deque
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from coherent_time import JointTimeline
from execute_coherent import CoherentCycle


def pose(position=0.):
    return dict(position=dict(x=position, y=0., z=0.), orientation=dict(x=0., y=0., z=0., w=1.))


def fixture():
    instance = object.__new__(CoherentCycle)
    instance.joint_timeline = JointTimeline(['joint'])
    instance.joint_timeline.add(.99, {'joint':0.})
    instance.joint_timeline.add(1., {'joint':.005})
    packet = {'right_wrist_yaw_link':pose(), 'box48_right':pose(.0004)}
    instance.native_frames = deque([(1., time.monotonic(), packet)])
    instance.native_lock = threading.Lock()
    instance.native_grasp_transform = np.eye(4)
    instance.coherent_reference_key = None
    instance.coherent_reference = None
    instance.reference_native_bytes = None
    instance.state = SimpleNamespace(joint_positions={}, update=lambda:None,
        get_global_link_transform=lambda link:np.eye(4))
    instance.poses = {'right_wrist_yaw_link':pose(), 'box48_right':pose(.0004)}
    instance.pose_time, instance.joint_time = 1.005, 1.
    instance.grasp_transform = np.eye(4)
    unaligned = np.eye(4)
    unaligned[0, 3] = -.003
    instance.wrist = lambda side:unaligned
    instance.max_slip = instance.max_native_slip = instance.maximum_legacy_unaligned_slip = 0.
    instance.last_coherent_record = -1.
    instance.events = []
    instance.record = lambda kind, data:instance.events.append((kind, data))
    return instance


def test_frozen_reference_not_rebased_from_payload_and_old_unaligned_is_diagnostic():
    instance = fixture()
    instance.check_coherent_grip('right', 'right', False)
    assert instance.max_slip == pytest.approx(.0004)
    assert instance.max_native_slip == pytest.approx(.0004)
    assert instance.maximum_legacy_unaligned_slip == pytest.approx(.0034)
    assert np.allclose(instance.coherent_reference, np.eye(4))
    assert instance.events[0][0] == 'coherent_grip_reference_frozen'
    assert instance.events[0][1]['current_payload_not_used_to_rebase']


def test_latest_native_true_slip_fails_even_when_older_coherent_packet_is_safe():
    instance = fixture()
    instance.poses['box48_right'] = pose(.0021)
    with pytest.raises(RuntimeError, match='2mm/5deg'):
        instance.check_coherent_grip('right', 'right', False)
    assert instance.max_native_slip == pytest.approx(.0021)


def test_cannot_change_reference_during_same_hold_to_hide_real_slip():
    instance = fixture()
    instance.check_coherent_grip('right', 'right', False)
    instance.native_grasp_transform[0, 3] = .001
    with pytest.raises(RuntimeError, match='reference changed'):
        instance.check_coherent_grip('right', 'right', False)


def test_old_packet_or_unknown_bracket_never_falls_back_to_native_only():
    instance = fixture()
    instance.pose_time = 1.03
    with pytest.raises(RuntimeError, match='No recent'):
        instance.check_coherent_grip('right', 'right', False)


def test_model_and_native_geometry_disagreement_still_fails_closed():
    instance = fixture()
    mismatch = np.eye(4)
    mismatch[0, 3] = .002
    instance.state.get_global_link_transform = lambda link:mismatch
    with pytest.raises(RuntimeError, match='wrist agreement'):
        instance.check_coherent_grip('right', 'right', False)

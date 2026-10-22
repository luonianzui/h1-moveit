import numpy as np
import pytest

from audit_dual_grip import Ownership, metric


def test_separate_simultaneous_holders_cannot_overwrite_each_others_reference():
    audit = Ownership()
    for hand in ('left', 'right'):
        audit.begin(hand, hand, np.eye(4), np.eye(4))
    assert audit.begins == 2 and len(audit.rows) == 2
    audit.end('left', 'left')
    assert set(audit.rows) == {'right'}
    audit.end('right', 'right')
    assert audit.ends == 2 and not audit.rows


def test_rebase_duplicate_payload_and_unmatched_release_are_rejected():
    audit = Ownership()
    audit.begin('left', 'right', np.eye(4), np.eye(4))
    with pytest.raises(ValueError):
        audit.begin('left', 'right', np.eye(4), np.eye(4))
    with pytest.raises(ValueError):
        audit.begin('right', 'right', np.eye(4), np.eye(4))
    with pytest.raises(ValueError):
        audit.end('left', 'left')


def test_contact_sensor_identity_is_not_confused_with_which_hand_holds_payload():
    audit = Ownership()
    audit.begin('left', 'right', np.eye(4), np.eye(4))
    audit.begin('right', 'left', np.eye(4), np.eye(4))
    audit.end('right', 'left')
    assert audit.rows['left']['payload'] == 'right'


def test_independent_metric_uses_real_geometric_distance_and_angle():
    first, last = np.eye(4), np.eye(4)
    last[0, 3] = .0021
    distance, angle = metric(first, last)
    assert distance == pytest.approx(.0021) and angle == 0

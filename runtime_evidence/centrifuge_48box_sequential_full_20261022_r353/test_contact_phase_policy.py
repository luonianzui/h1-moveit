import pytest

from contact_phase_policy import allow_planning_grasp_touch, bounded_motion, table_birth_violation


@pytest.mark.parametrize('mode', ['load_pickup', 'retrieval_pickup', 'empty_return'])
def test_no_loose_payload_finger_exemption_in_any_empty_hand_path(mode):
    assert allow_planning_grasp_touch(mode) is False


@pytest.mark.parametrize('mode', ['pilot', 'load_transport', 'load_alignment', 'retrieval_transport'])
def test_grasp_touch_links_are_only_for_held_or_explicit_rigid_planning_scope(mode):
    assert allow_planning_grasp_touch(mode) is True


def test_unknown_phase_cannot_silently_exempt_collisions():
    with pytest.raises(ValueError):
        allow_planning_grasp_touch('unknown')


@pytest.mark.parametrize('phase', ['paired_load_descend', 'paired_retrieval_descend'])
def test_near_contact_descent_is_slower_and_tracking_is_stricter_than_old_global_gate(phase):
    assert bounded_motion(phase, .8) == (.13, .002)
    assert bounded_motion(phase, .05) == (.05, .002)


def test_free_empty_and_loaded_transport_rates_are_not_globally_changed():
    assert bounded_motion('paired_load_approach', .8) == (.8, .015)
    assert bounded_motion('paired_load_forward', .25) == (.25, .015)


@pytest.mark.parametrize('payload, y_position', [('left', .38), ('right', -.38)])
def test_unchanged_actual_birth_pose_is_accepted(payload, y_position):
    assert table_birth_violation(payload, [.44, y_position, .84475], 0.) is None


def test_r241_actual_displaced_right_box_is_rejected_before_grasp():
    assert table_birth_violation('right', [.4377849857, -.4311277925, .8447499864], 0.) == 'table_payload_displaced_before_grasp'


def test_original_grasp_translation_and_angle_limits_are_not_relaxed():
    import math

    assert table_birth_violation('left', [.440301, .38, .84475], 0.) == 'table_payload_displaced_before_grasp'
    assert table_birth_violation('left', [.44, .38, .84475], math.radians(.201)) == 'table_payload_rotated_before_grasp'


def test_invalid_native_pose_fails_closed():
    assert table_birth_violation('left', [float('nan'), .38, .84475], 0.) == 'invalid_native_table_pose'
    with pytest.raises(ValueError):
        table_birth_violation('unknown', [.44, .38, .84475], 0.)

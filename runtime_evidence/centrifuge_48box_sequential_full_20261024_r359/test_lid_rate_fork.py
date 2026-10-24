import hashlib
import json
from pathlib import Path

import pytest

from lid_rate_fork import CANDIDATE_VELOCITY_XML, ORIGINAL_VELOCITY_XML, verify_private_lid_rate_fork
from startup_ramp_model import restore_startup_model


EVIDENCE = Path(__file__).resolve().parent


def original_and_candidate():
    previous = json.loads((EVIDENCE / 'offline_profile_revision1_source_snapshot.json').read_text())
    record = previous['records']['inputs/centrifuge_brake.sdf']
    original = record['utf8'].encode()
    assert hashlib.sha256(original).hexdigest() == record['sha256']
    original = original.decode().replace('centrifuge_48box_complete_rate_profile_20261014_r262',
                                         EVIDENCE.name).encode()
    return original, restore_startup_model((EVIDENCE / 'inputs/centrifuge_brake.sdf').read_bytes())


def test_only_the_one_new_private_lid_velocity_field_changed():
    original, candidate = original_and_candidate()
    proof = verify_private_lid_rate_fork(original, candidate)
    assert proof['status'] == 'PASS'
    assert proof['candidate_lid_velocity_rad_s'] == .8
    assert proof['unchanged_upper_angle_deg'] == 120.
    assert proof['unchanged_joint_effort_Nm'] == 30.
    assert proof['rotor_motor_contact_interlock_and_steady_30_seconds_unchanged']
    assert proof['actual_lid_and_full_cycle_not_preclaimed']


@pytest.mark.parametrize('original_bytes,new_bytes', [
    (b'<effort>30</effort>', b'<effort>31</effort>'),
    (b'<upper>2.0943951023931953</upper>', b'<upper>2.2</upper>'),
    (b'<damping>0.15</damping>', b'<damping>0.16</damping>'),
    (b'<cmd_max>30</cmd_max>', b'<cmd_max>31</cmd_max>'),
    (CANDIDATE_VELOCITY_XML, b'<velocity>1.0</velocity>')])
def test_no_other_physical_limit_geometry_or_controller_change_is_accepted(original_bytes, new_bytes):
    original, candidate = original_and_candidate()
    assert original_bytes in candidate
    with pytest.raises(ValueError):
        verify_private_lid_rate_fork(original, candidate.replace(original_bytes, new_bytes, 1))


def test_duplicate_original_velocity_field_cannot_be_used_for_loose_global_replacement():
    original, candidate = original_and_candidate()
    with pytest.raises(ValueError):
        verify_private_lid_rate_fork(original + ORIGINAL_VELOCITY_XML, candidate)


def test_only_one_terminal_newline_not_arbitrary_serialization_changes_is_allowed():
    original, candidate = original_and_candidate()
    assert verify_private_lid_rate_fork(original, candidate)['terminal_newline_added_by_patch_tool']
    with pytest.raises(ValueError):
        verify_private_lid_rate_fork(original, candidate + b'\n')

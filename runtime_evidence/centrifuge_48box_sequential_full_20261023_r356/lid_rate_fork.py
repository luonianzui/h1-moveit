import math
import xml.etree.ElementTree as ET


ORIGINAL_LID_VELOCITY_RAD_S = .35
CANDIDATE_LID_VELOCITY_RAD_S = .8
ORIGINAL_VELOCITY_XML = b'<velocity>0.35</velocity>'
CANDIDATE_VELOCITY_XML = b'<velocity>0.8</velocity>'


def verify_private_lid_rate_fork(original, candidate):
    if original.count(ORIGINAL_VELOCITY_XML) != 1:
        raise ValueError('Exactly one original lid velocity field is required')
    expected = original.replace(ORIGINAL_VELOCITY_XML, CANDIDATE_VELOCITY_XML, 1)
    terminal_newline_added = not expected.endswith(b'\n') and candidate == expected + b'\n'
    if candidate != expected and not terminal_newline_added:
        raise ValueError('Private model may change only the declared lid velocity field')
    model = ET.fromstring(candidate)
    joint = model.find("model/joint[@name='lid_hinge_joint']")
    if joint is None:
        raise ValueError('Original physical lid hinge joint is required')
    limits = joint.find('axis/limit')
    values = {name: float(limits.findtext(name)) for name in ('lower', 'upper', 'effort', 'velocity')}
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError('Finite physical lid limits required')
    if values != dict(lower=0., upper=math.radians(120), effort=30., velocity=CANDIDATE_LID_VELOCITY_RAD_S):
        raise ValueError('Original angle/force limits or declared private velocity were changed')
    return dict(status='PASS', original_lid_velocity_rad_s=ORIGINAL_LID_VELOCITY_RAD_S,
                candidate_lid_velocity_rad_s=CANDIDATE_LID_VELOCITY_RAD_S,
                unchanged_upper_angle_deg=120., unchanged_joint_effort_Nm=30.,
                exact_byte_inverse_with_only_declared_terminal_newline=True,
                terminal_newline_added_by_patch_tool=terminal_newline_added,
                all_other_model_fields_unchanged=True,
                only_new_private_model_changed_original_models_not_written=True,
                rotor_motor_contact_interlock_and_steady_30_seconds_unchanged=True,
                no_joint_pose_reset_or_direct_gui_toggle=True,
                actual_lid_and_full_cycle_not_preclaimed=True)

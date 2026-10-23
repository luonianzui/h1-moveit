import hashlib
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PLUGIN_SHA256 = 'cb030abe1b549ac266132da3ba4cb5f78e460d015598d7be8bde81500d9dc8f1'
PARENT_MARKER = ('<plugin filename="' + str(ROOT / 'inputs/libCentrifugeInterlockBrakeR189.so')
                 + '" name="CentrifugeInterlockBrakeR189">').encode()
CURRENT_MARKER = ('<plugin filename="' + str(ROOT / 'inputs/libCentrifugeInterlockStartupRampProbeR1.so')
                  + '" name="CentrifugeInterlockStartupRampProbeR1">').encode()


def restore_startup_model(candidate):
    if candidate.count(CURRENT_MARKER) != 1 or candidate.count(PARENT_MARKER) != 0:
        raise ValueError('Exactly one declared private startup plugin binding required')
    return candidate.replace(CURRENT_MARKER, PARENT_MARKER, 1)


def verify_model_fork(expected, candidate):
    if restore_startup_model(candidate) != expected:
        raise ValueError('Only the declared private startup plugin binding may change')
    artifact = ROOT / 'inputs/libCentrifugeInterlockStartupRampProbeR1.so'
    if hashlib.sha256(artifact.read_bytes()).hexdigest() != PLUGIN_SHA256:
        raise ValueError('Tested startup plugin bytes do not match')
    return dict(status='PASS', plugin_sha256=PLUGIN_SHA256, startup_target_acceleration_rad_s2=.8,
                original_startup_target_acceleration_rad_s2=1., original_stop_target_acceleration_rad_s2=1.,
                run_speed_rad_s=3.141592653589793, original_geometry_inertias_friction_and_joint_limits=True,
                original_safety_lease_contact_and_torque_brake_guards=True,
                actual_fresh_status_field_readback_and_full_execution_required=True,
                not_current_physical_or_180_certificate=True)

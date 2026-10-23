import hashlib
import json
from pathlib import Path
from gain_contract import POSITION_GAIN, verify_gain_fork
from gripper_damping import DAMPING
from lid_rate_fork import verify_private_lid_rate_fork
from startup_ramp_model import verify_model_fork, PLUGIN_SHA256
from acceleration_inputs import verify_acceleration_input


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_retrieval_rate_restore_20261014_r266'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    manifest = dict((relative.lstrip('*'), checksum) for checksum, relative in
                    (line.split(maxsplit=1) for line in (PARENT / 'SHA256SUMS').read_text().splitlines()))
    records = []
    lid_fork = None
    for source in sorted((PARENT / 'inputs').iterdir()):
        if not source.is_file():
            continue
        relative = str(source.relative_to(PARENT))
        assert digest(source) == manifest[relative]
        expected = source.read_bytes()
        if source.suffix in ('.py', '.sh', '.md', '.json', '.yaml', '.sdf', '.urdf', '.config'):
            expected = expected.decode().replace(str(PARENT), str(EVIDENCE))
            expected = expected.replace('install_lab_scene_48box_retrieval_rate_restore_20261014_r266',
                                        'install_lab_scene_48box_sequential_full_20261023_r356')
            expected = expected.replace('centrifuge_48box_retrieval_rate_restore_r266',
                                        'centrifuge_48box_sequential_full_r349').encode()
        destination = EVIDENCE / relative
        if source.name == 'centrifuge_brake.sdf':
            startup_fork = verify_model_fork(expected, destination.read_bytes())
            with (EVIDENCE / 'startup_ramp_model_audit.json').open('x') as stream:
                json.dump(startup_fork, stream, indent=2)
            inherited = PARENT / 'private_lid_rate_fork_audit.json'
            assert digest(inherited) == manifest[inherited.name]
            lid_fork = json.loads(inherited.read_text())
            assert lid_fork['status'] == 'PASS' and lid_fork['candidate_lid_velocity_rad_s'] == .8
            lid_fork.update(parent_private_lid_rate_rad_s=.8, no_new_private_lid_velocity_change=True,
                            inherited_partial_lid_physics_not_current_full_certificate=True)
            lid_fork.update(parent_sha256=manifest[relative], current_sha256=digest(destination),
                            original_sha256_after_rebinding=hashlib.sha256(expected).hexdigest())
            lid_fork.update(rotor_motor_contact_interlock_and_steady_30_seconds_unchanged=False,
                            inherited_lid_proof_excludes_new_startup_ramp=True,
                            original_angle_force_and_lid_velocity_preserved=True)
        elif source.name == 'centrifuge_run_gui.config':
            assert destination.read_bytes().replace(b'1.65 -1.55 1.95 0.0 0.44652461926754994 2.305390786134123', b'-0.85 -1.45 1.9 0 0.36 0.87', 1) == expected, relative
        elif source.name == 'controllers_servo.yaml':
            assert destination.read_bytes() == expected, relative
        elif source.name in ('robot_dual.urdf', 'gazebo_dual.launch.py'):
            fork = verify_acceleration_input(expected, destination.read_bytes(), source.name,
                                            EVIDENCE.parents[1] / 'src/h1_2_description')
            with (EVIDENCE / (source.name + '.acceleration_audit.json')).open('x') as stream:
                json.dump(dict(status='PASS_DECLARED_INPUT_DELTA_ONLY', **fork), stream, indent=2)
        elif source.name == 'lab_dual.launch.py':
            from localization_source import restore_localizer_launch
            assert restore_localizer_launch(destination.read_text()).encode() == expected
        elif source.name == 'robot_servo.urdf':
            assert destination.read_bytes() == expected, relative
        else:
            assert destination.read_bytes() == expected, relative
        records.append(dict(relative=relative, parent_sha256=manifest[relative], current_sha256=digest(destination),
                            only_declared_private_lid_rate_change=False))
    for name in ('control_fork_audit.json', 'servo_fork_audit.json',
                 'dual_layout_audit.json'):
        source = PARENT / name
        assert digest(source) == manifest[name]
        document = json.loads(source.read_text())
        assert document['status'] == 'PASS'
        document.update(parent_audit_sha256=digest(source), input_rebind_independently_verified=True,
                        inherited_configuration_or_old_calibration_not_new_physical_pass=True,
                        current_identity=str(EVIDENCE))
        document['current_gripper_pid_d'] = DAMPING
        document['parent_gripper_dynamics_not_current_physical_certificate'] = True
        document['inherited_same_pid_not_general_physical_certificate'] = True
        document['new_d6_current_physical_qualification_not_preclaimed'] = True
        document['controllers_servo_yaml_sha256'] = digest(EVIDENCE / 'inputs/controllers_servo.yaml')
        if name == 'servo_fork_audit.json':
            document['robot_servo_sha256'] = digest(EVIDENCE / 'inputs/robot_servo.urdf')
            document['position_proportional_gain'] = POSITION_GAIN
            document['gain_specific_calibration_required'] = True
            document['loaded_motion_not_preclaimed'] = True
        if name == 'dual_layout_audit.json':
            document['robot_dual_sha256'] = digest(EVIDENCE / 'inputs/robot_dual.urdf')
            document['physics_controller_and_all_model_geometry_inertias_limits_friction_unchanged'] = False
            document['all_geometry_inertias_friction_robot_limits_lid_angle_and_force_unchanged'] = False
            document['only_two_fixed_pelvis_ground_trimmed_foot_colliders_approximated'] = True
            document['manipulation_geometry_inertias_friction_limits_lid_angle_and_force_unchanged'] = True
            document['only_declared_new_private_lid_velocity_limit_changed'] = False
            document['current_private_lid_rate_unchanged_from_sealed_parent'] = True
        with (EVIDENCE / name).open('x') as stream:
            json.dump(document, stream, indent=2)
    assert lid_fork is not None
    artifact = EVIDENCE / 'inputs/libCentrifugeInterlockStartupRampProbeR1.so'
    assert digest(artifact) == PLUGIN_SHA256
    records.append(dict(relative=str(artifact.relative_to(EVIDENCE)), current_sha256=PLUGIN_SHA256,
                        tested_isolated_private_startup_plugin=True))
    with (EVIDENCE / 'inherited_inputs_audit.json').open('x') as stream:
        json.dump(dict(status='PASS', records=records, all_input_changes_only_exclusive_identity_bindings=False,
                       only_new_physics_change_is_declared_private_startup_ramp_plugin=False,
                       startup_plugin_inherited_byte_identical_from_r274=True,
                       only_new_runtime_change_is_qualified_retrieval_grip_feedforward=False,
                       retrieval_feedforward_inherited_twenty_N_not_yet_physically_reached=True,
                       only_new_runtime_change_is_empty_retrieval_approach_rate=True,
                       inherited_private_lid_rate_preserved_without_current_physical_change=True,
                       inherited_unchanged_four_gripper_pid_derivative_fields=DAMPING,
                       inherited_gain_specific_empty_calibration_not_full_physical_pass=POSITION_GAIN,
                       empty_contact_policy_corrected_in_current_source_not_parent=True,
                       no_old_claim_run_or_actual_physical_result_inherited=True), stream, indent=2)
    with (EVIDENCE / 'private_lid_rate_fork_audit.json').open('x') as stream:
        json.dump(lid_fork, stream, indent=2)
    print('INHERITED_INPUTS_VERIFIED', len(records), flush=True)


if __name__ == '__main__':
    main()

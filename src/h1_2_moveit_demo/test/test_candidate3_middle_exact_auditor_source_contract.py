"""Static fail-closed contract for the offline 828-E0 exact auditor."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / 'src/candidate3_middle_exact_auditor.cpp').read_text(
    encoding='utf-8')
CMAKE = (PACKAGE / 'CMakeLists.txt').read_text(encoding='utf-8')


def test_auditor_has_no_ros_or_controller_output_surface():
    """Controller topic names are input identities, never output handles."""
    forbidden = (
        'create_publisher', 'create_client', 'create_subscription',
        'ActionClient', 'async_send_goal', 'FollowJointTrajectory',
        '.execute(', 'move_group_interface',
    )
    for token in forbidden:
        assert token not in SOURCE
    assert 'controller_commands_issued\\\": 0' in SOURCE
    assert 'offline_only\\\": true' in SOURCE
    assert 'ARGUMENT_DIAGNOSTIC_FLAG_FORBIDDEN' in SOURCE


def test_auditor_has_exact_eleven_phase_middle_and_suffix_owns_release():
    """The middle keeps q90 held and has no right release/retreat phase."""
    phases = (
        'door_pre_support', 'door_open_supported',
        'door60_seam_to_cube', 'left_cube_ready', 'cube_contact',
        'cube_carry_place', 'cube_release', 'cube_retreat',
        'left_cube_park', 'door60_seam_to_hinge',
        'door_close_supported',
    )
    assert 'const std::array<PhaseSpec, 11> kPhases' in SOURCE
    for phase in phases:
        assert SOURCE.count(f'{{"{phase}", AssetKind::') == 1
    assert 'right_release' not in SOURCE
    assert 'right_retreat' not in SOURCE
    assert 'return true;\n}\n\nbool right_handle_distance_window' in SOURCE
    assert 'constexpr double kHandleAngle = kPi / 2.0' in SOURCE
    assert 'verify_seam_state(state, park_roll)' in SOURCE


def test_contact_authorizations_are_exact_and_guard_semantics_are_separate():
    """Only right pads hold the handle and only six left links touch cubes."""
    for link in (
        'right_dex1_finger_1_pad_link',
        'right_dex1_finger_2_pad_link',
        'left_dex1_finger_1_slider_link',
        'left_dex1_finger_1_link',
        'left_dex1_finger_1_pad_link',
        'left_dex1_finger_2_slider_link',
        'left_dex1_finger_2_link',
        'left_dex1_finger_2_pad_link',
    ):
        assert link in SOURCE
    assert 'matrix.setEntry(kHandleId, pad, true)' in SOURCE
    assert 'matrix.setEntry("incubator_door_exact"' not in SOURCE
    assert 'left_handle_acm.setEntry(kHandleId, link, false)' in SOURCE
    assert 'green_cube_dynamic_exact_physical_only' in SOURCE
    assert 'green_cube_10mm_guard_conservative_nonphysical' in SOURCE
    assert 'nearest_signed_distance_observation_not_collision_event' in SOURCE
    assert 'cube_grasp_carry_observed_exact_pairs' in SOURCE
    assert 'cube_grasp_carry_observed_guard_pairs' in SOURCE
    assert 'cube_release_retreat_observed_exact_pairs' in SOURCE
    assert 'cube_release_retreat_observed_guard_pairs' in SOURCE
    assert r'\"right_door_allowed_contact_pairs\": []' in SOURCE


def test_projection_and_dynamic_exact_geometry_are_fail_closed():
    """Perception masks are projected, while exact objects stay dynamic."""
    assert 'remove_stage_objects' not in SOURCE
    assert 'getWorldNonConst()->removeObject' not in SOURCE
    assert 'RAW_SCENE_ALREADY_PHYSICAL' in SOURCE
    assert 'ALLOWLIST_MASKS_PROJECTED' in SOURCE
    assert 'INCOMPLETE_MASK_CONFLICT_PAIR' in SOURCE
    assert 'UNKNOWN_VISUAL_MASK_ID' in SOURCE
    assert 'HANDLE_EXACT_REMOVED' in SOURCE
    assert 'CUBE_RELEASE_PHYSICAL_STL_CLEARANCE' in SOURCE
    assert 'CUBE_RELEASE_VISUAL_DISTANCE' in SOURCE
    assert 'move_dynamic_geometry(' in SOURCE


def test_environment_and_self_clearances_are_actually_measured():
    """World and self minima use signed dense distance queries."""
    assert 'DistanceRequestTypes::ALL' in SOURCE
    assert 'allow_all_self_distance_pairs(distance_acm, collision_links)' in SOURCE
    assert 'physical_environment_distance(' in SOURCE
    assert 'distanceSelf(' in SOURCE
    assert 'sample_self_clearance' in SOURCE
    assert 'environment_table_10mm_margin' in SOURCE
    assert 'green_cube_10mm_guard' in SOURCE
    assert r'\"self_pairs_excluded\": true' in SOURCE


def test_all_inputs_are_sealed_and_parsed_from_the_verified_bytes():
    """Every semantic parser consumes the bytes checked before and after read."""
    assert 'before.st_nlink != 1' in SOURCE
    assert 'SEALED_FILE_METADATA' in SOURCE
    assert 'SEALED_FILE_TOCTOU' in SOURCE
    assert 'SEALED_FILE_SHA256' in SOURCE
    assert 'YAML::LoadFile' not in SOURCE
    assert 'YAML::Load(canonical_wrapper)' in SOURCE
    assert 'YAML::Load(inventory_text)' in SOURCE
    assert 'load_robot_model(\n      robot_model_snapshot.urdf_xml' in SOURCE
    assert 'reverify_robot_model_snapshot(robot_model_snapshot)' in SOURCE
    assert 'ROBOT_MODEL_ARGUMENT_PATH_BINDING' in SOURCE
    assert 'CUBE_REFERENCE_CANONICAL_PAYLOAD' in SOURCE
    assert 'CUBE_BUMP_UNAUTHORIZED_JOINT_DELTA' in SOURCE


def test_retreat_bump_target_and_dense_metrics_are_hard_gates():
    """All values copied by the assembler are measured and finite-gated."""
    for token in (
        'cube_bump_maximum_tcp_translation',
        'cube_bump_maximum_tcp_rotation',
        'final_target_translation_error',
        'final_target_rotation_error',
        'cube_release_retreat_minimum_time_step_ns',
        'cube_release_retreat_minimum_path_parameter_step',
        'minimum_tcp_separation_projection_step_m',
        'post_gate_minimum_guard_physical_lower_bound_m',
        'gate_never_regressed',
    ):
        assert token in SOURCE
    assert 'CUBE_RELEASE_RETREAT_EXACT_PENETRATION' in SOURCE
    assert 'CUBE_RELEASE_RETREAT_GATE_REGRESSION' in SOURCE
    assert 'AUDIT_RELEASE_RETREAT_METRICS' in SOURCE
    assert 'AUDIT_CONTACT_WINDOWS' in SOURCE
    assert 'TORSO_45_DEGREE_LIMIT' in SOURCE


def test_raw_pass_is_explicitly_complete_but_never_a_certificate():
    """A failure or draft document cannot masquerade as final evidence."""
    assert 'h1_candidate3_middle_e0_raw_combined_exact/828-e0-v1' in SOURCE
    assert 'h1_candidate3_middle_exact_audit_failure/v1' in SOURCE
    assert 'raw_audit_complete\\\": true' in SOURCE
    assert 'geometry_valid\\\": true' in SOURCE
    assert 'valid\\\": false' in SOURCE
    assert 'RAW_EXACT_PASS_NOT_SEALED_RESULT' in SOURCE
    assert 'candidate3_middle_exact_auditor' in CMAKE
    target_block = CMAKE.split(
        'add_executable(\n  candidate3_middle_exact_auditor', 1)[1]
    assert 'target_link_libraries(candidate3_middle_exact_auditor fcl ' in target_block

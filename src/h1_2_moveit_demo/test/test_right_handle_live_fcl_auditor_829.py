"""Fail-closed source contract for the 829 live exact-FCL replay auditor."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / 'src/right_handle_live_fcl_auditor_829.cpp').read_text(
    encoding='utf-8')


def test_replay_is_file_only_and_has_no_control_or_ros_graph_surface():
    for token in (
            'create_publisher', 'create_client', 'create_subscription',
            'ActionClient', 'async_send_goal', 'FollowJointTrajectory',
            'move_group_interface', 'rclcpp::init', 'rclcpp::Node'):
        assert token not in SOURCE
    assert '#define RIGHT_HANDLE_PATH_EXACT_AUDITOR_NO_MAIN' in SOURCE
    assert 'geometric_only=true force_or_wrench_claimed=false' in SOURCE


def test_tsv_contract_separates_door_hinge_from_handle_latch():
    header = (
        'id\\tcontact_required\\tdoor_angle_deg\\thandle_angle_deg\\t'
        'torso_rad\\t"\n  "leg_rad\\t')
    assert header in SOURCE
    assert 'fields.size() != 10U' in SOURCE
    assert 'fields[1] != "0" && fields[1] != "1"' in SOURCE
    assert 'live_csv_vector<7>' in SOURCE
    assert 'live_csv_vector<12>' in SOURCE
    assert 'live_csv_vector<2>' in SOURCE
    assert 'TSV_DOOR_RANGE' in SOURCE
    assert 'TSV_HANDLE_RANGE' in SOURCE
    assert 'TSV_ID' in SOURCE


def test_only_door_range_has_a_dedicated_physics_tolerance():
    assert 'constexpr double kLiveDoorPhysicsTolerance = 5e-5;' in SOURCE
    assert 'row.door_angle < -kLiveDoorPhysicsTolerance' in SOURCE
    assert 'row.door_angle > kLiveDoorLimit + kLiveDoorPhysicsTolerance' in SOURCE
    assert 'row.handle_angle < -kLiveNumericTolerance' in SOURCE
    assert 'row.handle_angle > kLiveHandleLimit + kLiveNumericTolerance' in SOURCE
    assert 'constexpr double kLiveJointBoundsTolerance = 2e-6;' in SOURCE
    assert 'state.satisfiesBounds(kLiveJointBoundsTolerance)' in SOURCE


def test_path_and_input_contracts_are_strict_and_toctou_checked():
    assert 'h1_moveit/right_handle_variable_torso_path/v4' in SOURCE
    assert 'std::abs(asset.torso_abs_limit - kLiveTorsoLimit)' in SOURCE
    assert '44.5' in SOURCE
    assert 'ARGUMENT_UNKNOWN' in SOURCE
    assert 'ARGUMENT_DUPLICATE' in SOURCE
    assert 'INPUT_NOT_REGULAR_SINGLE_LINK' in SOURCE
    assert 'INPUT_TOCTOU' in SOURCE
    assert 'PATH_TOCTOU' in SOURCE
    assert 'verify_handle_mesh(projection.physical, handle_bytes)' in SOURCE


def test_dynamic_original_geometry_uses_both_mechanism_angles():
    assert 'live_desired_door_pose(' in SOURCE
    assert 'live_desired_handle_pose(' in SOURCE
    assert '0.0, 0.0, door_angle' in SOURCE
    assert '0.0, handle_angle, 0.0' in SOURCE
    assert 'LIVE_HANDLE_Q0_SCENE_POSE' in SOURCE
    assert 'set_single_shape_pose(scene_, "incubator_door_exact"' in SOURCE
    assert 'set_single_shape_pose(scene_, kHandleId' in SOURCE


def test_all_31_live_robot_joints_are_replayed_including_12_leg_joints():
    assert 'const std::array<std::string, 12> kLiveLegVariables' in SOURCE
    for name in (
            'left_hip_yaw_joint', 'left_hip_pitch_joint',
            'left_hip_roll_joint', 'left_knee_joint',
            'left_ankle_pitch_joint', 'left_ankle_roll_joint',
            'right_hip_yaw_joint', 'right_hip_pitch_joint',
            'right_hip_roll_joint', 'right_knee_joint',
            'right_ankle_pitch_joint', 'right_ankle_roll_joint'):
        assert f'"{name}"' in SOURCE
    assert 'state.setVariablePosition(kLiveLegVariables[index]' in SOURCE


def test_collision_clearance_and_contact_exclusivity_are_hard_gates():
    assert 'state.satisfiesBounds(kLiveJointBoundsTolerance)' in SOURCE
    assert 'TORSO_44_5_DEGREE_LIMIT' in SOURCE
    assert 'scene_.checkSelfCollision' in SOURCE
    assert 'getCollisionEnvUnpadded()->checkRobotCollision' in SOURCE
    assert 'kLiveLeftHandleClearance = 0.050' in SOURCE
    assert 'kNonpadHandleClearance' in SOURCE
    assert 'kLiveRightPadGripProximity = 0.0003' in SOURCE
    assert 'RIGHT_PAD_GRIP_PROXIMITY_REQUIRED' in SOURCE
    assert 'RIGHT_PAD_GRIP_PROXIMITY_FORBIDDEN' in SOURCE
    assert 'RIGHT_FINGER_BODY_HANDLE_PENETRATION' in SOURCE
    assert 'right_dex1_finger_1_link' in SOURCE
    assert 'right_dex1_finger_2_link' in SOURCE
    assert 'metric.pad1_valid = live_valid_patch' in SOURCE
    assert 'metric.pad2_valid = live_valid_patch' in SOURCE
    assert 'std::min(metric.pad1_distance, metric.pad2_distance)' in SOURCE
    assert 'metric.right_pad_grip_engaged' in SOURCE


def test_dynamic_cube_domain_is_explicitly_delegated_not_replayed_at_q0():
    assert 'exclude_live_dynamic_cube_domain(physical_scene)' in SOURCE
    assert '"green_cube_dynamic_exact", "green_cube_10mm_guard"' in SOURCE
    assert 'LIVE_CUBE_DOMAIN_OBJECT:' in SOURCE
    assert 'LIVE_CUBE_DOMAIN_REMOVE:' in SOURCE
    assert 'candidate3 middle exact certificate' in SOURCE


def test_machine_parseable_rows_and_summary_fail_closed():
    assert 'RIGHT_HANDLE_LIVE_FCL_ROW_829' in SOURCE
    assert 'RIGHT_HANDLE_LIVE_FCL_SUMMARY_829' in SOURCE
    for field in (
            'evaluated=', 'passed=', 'contact_required_passed=',
            'noncontact_passed=', 'max_torso_rad=',
            'min_nonpad_handle_clearance_m=',
            'min_left_handle_clearance_m=', 'min_patch_span_m=',
            'min_patch_station_m=', 'min_patch_radius_m='):
        assert field in SOURCE
    for field in (
            'right_pad_grip_engaged=', 'pad1_handle_distance_m=',
            'pad2_handle_distance_m=',
            'adjacent_finger_handle_clearance_m=',
            'max_contact_minimum_right_pad_distance_m=',
            'min_adjacent_finger_handle_clearance_m='):
        assert field in SOURCE
    assert 'CONTACT_AND_NONCONTACT_ROWS_REQUIRED' in SOURCE
    assert 'return valid ? 0 : 3;' in SOURCE
    assert 'return 3;' in SOURCE


def test_native_library_stderr_is_silenced_but_failures_remain_structured():
    assert 'void silence_native_library_stderr()' in SOURCE
    assert 'std::freopen("/dev/null", "w", stderr)' in SOURCE
    assert 'silence_native_library_stderr();' in SOURCE
    assert 'print_live_row_error(row, error.what())' in SOURCE
    assert 'print_live_summary(' in SOURCE
    assert 'return valid ? 0 : 3;' in SOURCE

from pathlib import Path


SOURCE = (Path(__file__).parents[1] / 'src' /
          'candidate3_clearance_auditor.cpp').read_text(encoding='utf-8')


def test_auditor_is_file_only_and_has_no_control_surface():
    assert 'create_publisher' not in SOURCE
    assert 'create_client' not in SOURCE
    assert 'ActionClient' not in SOURCE
    assert 'FollowJointTrajectory' not in SOURCE
    assert 'execute(' not in SOURCE
    assert 'apply_planning_scene' not in SOURCE
    assert 'rclcpp::Node' not in SOURCE


def test_auditor_uses_world_only_unpadded_distance_query():
    assert 'getCollisionEnvUnpadded()->distanceRobot' in SOURCE
    assert 'getCollisionEnvUnpadded()->checkRobotCollision' in SOURCE
    assert 'scene.checkSelfCollision' in SOURCE
    assert 'kRequiredClearance = 0.010' in SOURCE
    assert 'DistanceRequestTypes::SINGLE' in SOURCE


def test_expanded_guards_are_not_double_counted():
    assert 'environment_table_10mm_margin' in SOURCE
    assert 'green_cube_10mm_guard' in SOURCE
    assert 'distance_acm.setEntry' in SOURCE


def test_post_navigation_right_open_has_explicit_contact_removal_stages():
    assert 'post_nav_right_handle_open/v1' in SOURCE
    assert 'post_nav_right_handle_open_roundtrip_audit/v1' in SOURCE
    assert '{0U, {"incubator_handle_exact"}}' in SOURCE
    assert '{2U, {"incubator_door_exact"}}' in SOURCE
    assert '? 6U :' in SOURCE
    assert '(post_nav_right_open ? 3U : 7U)' in SOURCE

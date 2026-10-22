import math
from pathlib import Path

from h1_2_navigation.incubator_auto_navigator import (
    locomotion_hold_ready,
)
from h1_2_navigation.incubator_auto_navigator import (
    locomotion_support_ready,
)
from h1_2_navigation.incubator_auto_navigator import (
    point_to_segment_distance,
)
from h1_2_navigation.incubator_auto_navigator import (
    segment_intersects_rectangle,
)
from h1_2_navigation.incubator_auto_navigator import (
    sustained_hold_sample_ready,
)
from h1_2_navigation.incubator_auto_navigator import (
    validate_incubator_staging_contract,
)
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BASE_LAUNCH = PACKAGE_ROOT / 'launch' / 'v12_navigation_gazebo.launch.py'
COMBINED_LAUNCH = (
    PACKAGE_ROOT / 'launch' / 'incubator_navigation_gazebo.launch.py')
SETUP = PACKAGE_ROOT / 'setup.py'
AUTO_NAVIGATOR = (
    PACKAGE_ROOT / 'h1_2_navigation' / 'incubator_auto_navigator.py')


def test_fixed_startup_and_incubator_staging_goal_are_bounded():
    distance, expanded = validate_incubator_staging_contract(
        (-0.60, 0.0, 0.0),
        (-0.45, 0.0, 0.0),
        (0.20, 1.00, -0.60, 0.60),
    )

    assert distance == pytest.approx(0.15)
    assert expanded == pytest.approx((-0.30, 1.50, -1.05, 1.05))


def test_original_origin_to_forward_goal_enters_workcell_keepout():
    with pytest.raises(ValueError, match='workcell keepout'):
        validate_incubator_staging_contract(
            (0.0, 0.0, 0.0),
            (0.15, 0.0, 0.0),
            (0.20, 1.00, -0.60, 0.60),
        )


def test_staging_contract_rejects_long_or_rotated_motion():
    with pytest.raises(ValueError, match='bounded motion envelope'):
        validate_incubator_staging_contract(
            (-1.0, 0.0, 0.0),
            (-0.45, 0.0, 0.0),
            (0.20, 1.00, -0.60, 0.60),
        )
    with pytest.raises(ValueError, match='zero yaw'):
        validate_incubator_staging_contract(
            (-0.60, 0.0, math.radians(5.0)),
            (-0.45, 0.0, 0.0),
            (0.20, 1.00, -0.60, 0.60),
        )


def test_segment_rectangle_intersection_handles_stationary_axes():
    bounds = (-0.30, 1.50, -1.05, 1.05)

    assert not segment_intersects_rectangle(
        (-0.60, 0.0), (-0.45, 0.0), bounds)
    assert segment_intersects_rectangle(
        (-0.60, 0.0), (0.0, 0.0), bounds)
    assert not segment_intersects_rectangle(
        (-0.45, 1.20), (-0.45, 1.20), bounds)
    assert segment_intersects_rectangle(
        (0.0, 0.0), (0.0, 0.0), bounds)


def test_physical_corridor_distance_catches_overshoot_and_sideways_drift():
    start = (-0.60, 0.0)
    goal = (-0.45, 0.0)

    assert point_to_segment_distance((-0.52, 0.0), start, goal) == 0.0
    assert point_to_segment_distance((-0.30, 0.0), start, goal) == pytest.approx(
        0.15)
    assert point_to_segment_distance((-0.52, 0.20), start, goal) == pytest.approx(
        0.20)


def test_hold_gate_accepts_legacy_capture_and_unitree_station_keeping():
    assert locomotion_hold_ready({
        'state': 'CAPTURED_HOLD',
        'capture_latched': True,
    }, planar_speed=0.02, position_tolerance=0.08, yaw_tolerance=0.20)
    assert locomotion_hold_ready({
        'state': 'STATION_KEEPING_GAIT',
        'zero_command_behavior': 'recurrent_station_keeping',
        'command': [0.0, 0.0, 0.0],
        'station_error': 0.04,
        'station_yaw_error': -0.10,
    }, planar_speed=0.20, position_tolerance=0.08, yaw_tolerance=0.20)


def test_hold_gate_rejects_drift_or_nonzero_requested_command():
    payload = {
        'state': 'STATION_KEEPING_GAIT',
        'zero_command_behavior': 'recurrent_station_keeping',
        'command': [0.01, 0.0, 0.0],
        'station_error': 0.09,
        'station_yaw_error': 0.0,
    }

    assert not locomotion_hold_ready(
        payload, planar_speed=0.0,
        position_tolerance=0.08, yaw_tolerance=0.20)


def test_support_gate_distinguishes_dynamic_and_static_hold():
    assert locomotion_support_ready(
        {'state': 'STATION_KEEPING_GAIT'}, [0.0, 620.0])
    assert not locomotion_support_ready(
        {'state': 'STATION_KEEPING_GAIT'}, [0.0, 250.0])
    assert locomotion_support_ready(
        {'state': 'CAPTURED_HOLD'}, [300.0, 320.0])
    assert not locomotion_support_ready(
        {'state': 'CAPTURED_HOLD'}, [0.0, 620.0])


def test_sustained_hold_gate_requires_complete_continuous_envelope():
    payload = {
        'state': 'STATION_KEEPING_GAIT',
        'zero_command_behavior': 'recurrent_station_keeping',
        'command': [0.0, 0.0, 0.0],
        'station_error': 0.04,
        'station_yaw_error': -0.10,
    }
    arguments = dict(
        payload=payload,
        localization_valid=True,
        localized_goal_error=0.07,
        physical_goal_error=0.06,
        localized_yaw_error=0.10,
        planar_speed=0.20,
        position_tolerance=0.08,
        yaw_tolerance=0.20,
        up_z=0.99,
        height=1.0,
        contact_forces=[0.0, 350.0],
    )

    assert sustained_hold_sample_ready(**arguments)
    assert not sustained_hold_sample_ready(
        **dict(arguments, physical_goal_error=0.081))
    assert not sustained_hold_sample_ready(
        **dict(arguments, localization_valid=False))
    assert not sustained_hold_sample_ready(
        **dict(arguments, up_z=0.979))
    assert not sustained_hold_sample_ready(
        **dict(arguments, contact_forces=[0.0, 299.0]))


def test_base_launch_binds_one_startup_pose_to_spawn_and_localization():
    launch = BASE_LAUNCH.read_text(encoding='utf-8')

    assert "LaunchConfiguration('startup_x')" in launch
    assert "LaunchConfiguration('startup_y')" in launch
    assert "LaunchConfiguration('startup_yaw')" in launch
    assert "'-x', startup_x" in launch
    assert "'-y', startup_y" in launch
    assert "'-Y', startup_yaw" in launch
    assert "'initial_x': ParameterValue(startup_x" in launch
    assert "'initial_y': ParameterValue(startup_y" in launch
    assert "'initial_yaw': ParameterValue(startup_yaw" in launch


def test_combined_launch_is_additive_and_auto_navigation_is_fail_closed():
    launch = COMBINED_LAUNCH.read_text(encoding='utf-8')
    setup = SETUP.read_text(encoding='utf-8')

    assert "startup_x = '-0.60'" in launch
    assert "goal_x = '-0.45'" in launch
    assert "'static_obstacle_enabled': 'true'" in launch
    assert "'auto_navigate', default_value='false'" in launch
    assert "'movement_authorized', default_value='false'" in launch
    assert "'models', 'table', 'model.sdf'" in launch
    assert "'models', 'green_cube', 'model.sdf'" in launch
    assert "'models', 'incubator', 'model.sdf'" in launch
    assert 'candidate3_roundtrip_831_authority' not in launch
    assert 'incubator_auto_navigator = ' in setup


def test_auto_navigation_waits_for_active_nav2_lifecycle_nodes():
    source = AUTO_NAVIGATOR.read_text(encoding='utf-8')

    for node_name in (
            'amcl', 'planner_server', 'controller_server',
            'behavior_server', 'bt_navigator'):
        assert f"'{node_name}'," in source
    assert 'state == State.PRIMARY_STATE_ACTIVE' in source
    assert 'self.navigation_client.server_is_ready()' in source
    assert 'QoSProfile(depth=1)' in source
    assert 'ReliabilityPolicy.BEST_EFFORT' in source
    assert "'VALIDATING_LONG_HOLD'" in source
    assert "'INCUBATOR_STAGING_GOAL_LONG_HOLD'" in source
    assert 'self.last_status_published_at' in source
    assert "self.state == 'PASS'" in source
    assert '>= 0.2' in source
    assert "'POST_PASS_HOLD_ENVELOPE_EXCEEDED'" in source

from pathlib import Path

import pytest

from scene_contract import classify_regression, verify_declared_nodes, verify_fixed_pelvis
from scene_contract import verify_live_processes, verify_scope, verify_ctest_result, SCOPE


ROBOT = '<robot><link name="pelvis"/><link name="left_wrist_yaw_link"/><link name="right_wrist_yaw_link"/><joint name="world_joint" type="fixed"><parent link="world"/><child link="pelvis"/></joint></robot>'


def test_existing_fixed_root_passes():
    verify_fixed_pelvis(ROBOT)


@pytest.mark.parametrize('replacement', ('floating', 'revolute', 'prismatic'))
def test_free_root_rejected(replacement):
    with pytest.raises(ValueError):
        verify_fixed_pelvis(ROBOT.replace('type="fixed"', 'type="' + replacement + '"'))


def test_localizer_without_walking_passes():
    assert verify_declared_nodes("Node(package='h1_2_navigation', executable='mid360_square_localizer')")


@pytest.mark.parametrize('executable', ('v12_runtime', 'unitree_h12_runtime', 'locomotion_bridge', 'velocity_gate', 'nav2_controller', 'other_controller'))
def test_unreviewed_navigation_node_rejected(executable):
    with pytest.raises(ValueError):
        verify_declared_nodes("Node(package='h1_2_navigation', executable='" + executable + "')")


def test_dynamic_node_rejected():
    with pytest.raises(ValueError):
        verify_declared_nodes("Node(package='h1_2_navigation', executable=LaunchConfiguration('controller'))")


@pytest.mark.parametrize('scope,walking,reconstructed', ((SCOPE, True, False), (SCOPE, False, True), ('navigation', False, False)))
def test_scope_cannot_enable_walking_or_fake_checkpoint(scope, walking, reconstructed):
    with pytest.raises(ValueError):
        verify_scope(scope, walking, reconstructed)


def test_startup_scope_passes():
    verify_scope(SCOPE, False, False)


def test_selected_launch_chain_is_policy_free():
    root = Path(__file__).resolve().parent
    verify_fixed_pelvis((root / 'inputs/robot_effort.urdf').read_text())
    for name in ('lab_effort.launch.py', 'gazebo_effort.launch.py'):
        verify_declared_nodes((root / 'inputs' / name).read_text())


def regression_xml(failed_name=None, reason='FileNotFoundError: IsaacLab-3.0-beta2/model_17250.pt missing'):
    cases = ['<testcase name="passed_' + str(number) + '"/>' for number in range(112)]
    cases.append('<testcase name="' + (failed_name or 'passed_last') + '">' +
                 ('<failure>' + reason + '</failure>' if failed_name else '') + '</testcase>')
    return '<testsuites><testsuite>' + ''.join(cases) + '</testsuite></testsuites>'


def test_missing_training_source_remains_visible():
    result = classify_regression(regression_xml('test_v12_checkpoint_identity_matches_contract'))
    assert result['scene_dependencies_passed'] and not result['original_regression_passed']
    assert result['passed'] == 112 and len(result['failed']) == 1


def test_other_failure_is_not_exempted():
    with pytest.raises(ValueError):
        classify_regression(regression_xml('test_model_geometry'))


def test_known_test_unrelated_failure_is_not_exempted():
    with pytest.raises(ValueError):
        classify_regression(regression_xml('test_v12_checkpoint_identity_matches_contract', 'wrong policy hash'))


def test_live_policy_is_rejected():
    with pytest.raises(ValueError):
        verify_live_processes(['/usr/bin/python3 unitree_h12_runtime'])


def test_live_nonpolicy_nodes_pass():
    verify_live_processes(['/usr/bin/gz sim', '/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge'])


def test_successful_ctest_zero_failures_pass():
    assert verify_ctest_result('Test project\n100% tests passed, 0 tests failed out of 6\n') == 6


@pytest.mark.parametrize('contents', ('83% tests passed, 1 tests failed out of 6',
                                    'No tests were found!!!',
                                    '100% tests passed, 0 tests failed out of 0',
                                    '***Skipped\n100% tests passed, 0 tests failed out of 6'))
def test_failed_empty_or_skipped_ctest_rejected(contents):
    with pytest.raises(ValueError):
        verify_ctest_result(contents)

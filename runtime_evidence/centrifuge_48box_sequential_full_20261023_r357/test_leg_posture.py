import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from execute_cycle import FullCycle
from leg_posture import LEG_JOINTS, leg_posture_targets, posture_controller_active


EVIDENCE = Path(__file__).resolve().parent


def test_only_twelve_existing_leg_position_interfaces_are_claimed():
    assert len(LEG_JOINTS) == 12 and len(set(LEG_JOINTS)) == 12
    robot = ET.parse(EVIDENCE / 'inputs/robot_effort.urdf').getroot()
    for name in LEG_JOINTS:
        joint = robot.find("./ros2_control/joint[@name='" + name + "']")
        assert joint.find('command_interface').attrib == {'name': 'position'}
    configuration = yaml.safe_load((EVIDENCE / 'inputs/controllers.yaml').read_text())
    params = configuration['leg_posture_controller']['ros__parameters']
    assert params['joints'] == list(LEG_JOINTS)
    assert params['command_interfaces'] == ['position']
    assert configuration['controller_manager']['ros__parameters']['leg_posture_controller']['type'] == 'joint_trajectory_controller/JointTrajectoryController'
    configuration['controller_manager']['ros__parameters'].pop('leg_posture_controller')
    configuration.pop('leg_posture_controller')
    baseline = yaml.safe_load((EVIDENCE.parents[1] / 'runtime_evidence/centrifuge_48box_terminal_unloading_20261010_r198/inputs/controllers.yaml').read_text())
    assert configuration == baseline


def test_targets_are_exact_existing_plan_not_zero_guess():
    initial = dict(zip(LEG_JOINTS, [index / 100 for index in range(12)]))
    targets, measured = leg_posture_targets(initial, initial)
    assert targets == measured == list(initial.values())


@pytest.mark.parametrize('invalid', [math.nan, math.inf, .010, -.011])
def test_nonfinite_or_unsettled_initial_posture_rejected(invalid):
    initial = dict.fromkeys(LEG_JOINTS, 0.)
    actual = dict(initial, left_knee_joint=invalid)
    with pytest.raises(ValueError):
        leg_posture_targets(initial, actual)


@pytest.mark.parametrize('status', ['inactive', 'unconfigured', 'activating', 'unknown'])
def test_nonactive_or_duplicate_posture_controller_rejected(status):
    assert not posture_controller_active('leg_posture_controller controller/type ' + status)
    assert not posture_controller_active('leg_posture_controller controller/type active\n' * 2)
    assert posture_controller_active('leg_posture_controller controller/type active')


def test_original_leg_gate_and_smooth_action_not_pose_reset():
    import inspect
    method = inspect.getsource(FullCycle.initialize_leg_posture)
    assert 'leg_posture_targets' in method and "last.time_from_start.sec = 1" in method
    assert 'original_smooth_duration_sim_s=5' in method
    assert "scope='fixed_pelvis_stationary_manipulation_not_free_gait'" in method
    assert 'response.error_code == 0' in method and "self.dwell(1)" in method
    assert 'reset' not in method and 'detach' not in method
    original = (EVIDENCE / 'execute_loading.py').read_text()
    assert "abs(self.positions[name] - initial) > .025" in original


import math
from pathlib import Path

import pytest

from cycle_guard import machine_ready_for_cycle


def machine(**changes):
    result = dict(ready=True, open_lid_demo=False, rotor_enabled=False,
                  rotor_rpm=0.0, rotor_rad_s=0.0, lid_rad=math.radians(120))
    result.update(changes)
    return result


@pytest.mark.parametrize('phase', ['preflight', 'right_lift', 'left_transfer'])
def test_loading_and_retrieval_keep_original_stopped_open_gate(phase):
    assert machine_ready_for_cycle(machine(), phase, 0)
    assert not machine_ready_for_cycle(machine(lid_rad=0), phase, 0)
    assert not machine_ready_for_cycle(machine(rotor_enabled=True), phase, 0)
    assert not machine_ready_for_cycle(machine(rotor_rpm=.021), phase, 0)


@pytest.mark.parametrize('phase', ['machine_start_press', 'machine_running_steady', 'machine_stop_press', 'machine_stop_wait_quiet'])
def test_only_closed_machine_phases_allow_rotor_motion(phase):
    assert machine_ready_for_cycle(machine(lid_rad=0, rotor_enabled=True, rotor_rpm=30, rotor_rad_s=math.pi), phase, 0)
    assert not machine_ready_for_cycle(machine(rotor_enabled=True, rotor_rpm=30, rotor_rad_s=math.pi), phase, 0)


@pytest.mark.parametrize('phase', ['machine_preparation_open', 'machine_close_press', 'machine_final_open_press'])
def test_lid_phases_refuse_moving_rotor(phase):
    assert not machine_ready_for_cycle(machine(lid_rad=0, rotor_rpm=1, rotor_rad_s=.1), phase, 0)
    assert machine_ready_for_cycle(machine(lid_rad=1), phase, 0)


@pytest.mark.parametrize('key', ['lid_rad', 'rotor_rpm', 'rotor_rad_s'])
@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_nonfinite_machine_fails_closed(key, value):
    assert not machine_ready_for_cycle(machine(**{key: value}), 'machine_running_steady', 0)


def test_demo_and_overspeed_refused():
    assert not machine_ready_for_cycle(machine(open_lid_demo=True), 'machine_start', 0)
    assert not machine_ready_for_cycle(machine(lid_rad=0, rotor_rpm=30.31, rotor_rad_s=math.pi), 'machine_running', 0)


def test_closed_lid_small_numeric_undershoot():
    assert machine_ready_for_cycle(machine(lid_rad=-1e-6), 'machine_close', 0)


def test_loading_executor_only_changes_identity_and_timeout_hook():
    current = Path(__file__).with_name('execute_loading.py').read_text()
    from speed_source import restore_speed_source
    current = restore_speed_source('execute_loading.py', current)
    current=current.replace('    def gripper_observation_phase(self, phase):\n        return phase\n\n','')
    current=current.replace('self.phase = self.gripper_observation_phase(phase)','self.phase = phase')
    previous = Path(__file__).parents[1] / 'centrifuge_48box_global_speed_20261009_r159/execute_loading.py'
    assert current.count('from grip_limits import grip_exceeds_limits\n') == 1
    current = current.replace('from grip_limits import grip_exceeds_limits\n', '')
    assert current.count('if grip_exceeds_limits(float(slip), math.degrees(angle)):') == 1
    current = current.replace('if grip_exceeds_limits(float(slip), math.degrees(angle)):', 'if slip > .002 or angle >= math.radians(2.0):')
    assert current.count('if grip_exceeds_limits(native_slip, math.degrees(native_angle)):') == 1
    current = current.replace('if grip_exceeds_limits(native_slip, math.degrees(native_angle)):', 'if native_slip > .002 or native_angle >= math.radians(2.0):')
    assert current.count('from trace_io import LosslessTrace\n') == 1
    current = current.replace('from trace_io import LosslessTrace\n', '')
    assert current.count("self.trace = LosslessTrace(EVIDENCE / 'run')") == 1
    current = current.replace("self.trace = LosslessTrace(EVIDENCE / 'run')",
                              "self.trace = (EVIDENCE / 'run/loading_trace.jsonl').open('x')")
    assert current.count('centrifuge_48box_sequential_full_r347') == 1
    current = current.replace('centrifuge_48box_sequential_full_r347', 'centrifuge_48box_global_speed_r159')
    current = current.replace('    def gripper_goal_time_tolerance(self, side, phase):\n        return 3\n\n', '')
    current = current.replace('goal.goal_time_tolerance.sec = self.gripper_goal_time_tolerance(side, phase)',
                              'goal.goal_time_tolerance.sec = 3')
    current = current.replace('    def localization_is_current(self, now):\n        return now - self.latest.get(\'localization\', 0) <= 8 and self.values[\'localization\'][\'valid\']\n\n', '')
    current = current.replace('if not self.localization_is_current(now):', "if now - self.latest.get('localization', 0) > 8 or not self.values['localization']['valid']:")
    begin = current.index('    def gripper_action_timeout_s(')
    end = current.index('    def gripper(self,', begin)
    hook = current[current.index('        initial = JointTrajectoryPoint(', begin):end]
    hook = hook.replace('        return [initial, point]', '        goal.trajectory.points = [initial, point]').rstrip()
    current = current[:begin] + current[end:]
    current = current.replace('        goal.trajectory.points = self.gripper_trajectory_points(side, value, phase, effort)', hook)
    current = current.replace('deadline = time.monotonic() + self.gripper_action_timeout_s(side, phase)',
                              'deadline = time.monotonic() + 90')
    assert current.count('    def minimum_waypoint_interval(self, phase):\n        return .020\n\n') == 1
    current = current.replace('    def minimum_waypoint_interval(self, phase):\n        return .020\n\n', '')
    assert current.count('max(self.minimum_waypoint_interval(phase),') == 1
    current = current.replace('max(self.minimum_waypoint_interval(phase),', 'max(.020,')
    assert current.count('times=times, initial=first') == 1
    current = current.replace('times=times, initial=first', 'times=times')
    assert current == previous.read_text()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_only_closing_gets_extra_convergence_time(side):
    from execute_cycle import FullCycle
    cycle = FullCycle.__new__(FullCycle)
    assert cycle.gripper_goal_time_tolerance(side, side + '_grasp') == 6
    assert cycle.gripper_goal_time_tolerance(side, side + '_release_on_table') == 3


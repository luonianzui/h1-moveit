from copy import deepcopy
import inspect
import json
from pathlib import Path

import pytest

from cycle_guard import machine_ready_for_cycle
from execute_cycle import FullCycle
from execute_loading import LoadingCycle
from gripper_phase_context import observation_phase


@pytest.mark.parametrize('side',['left','right'])
def test_only_explicit_retrieval_closing_gets_correct_observer_phase(side):
    assert observation_phase(side+'_grasp',True)==side+'_retrieval_grasp'
    assert observation_phase(side+'_grasp',False)==side+'_grasp'
    cycle=FullCycle.__new__(FullCycle)
    assert cycle.gripper_observation_phase(side+'_grasp')==side+'_grasp'
    cycle.retrieval_closing=True
    assert cycle.gripper_observation_phase(side+'_grasp')==side+'_retrieval_grasp'


@pytest.mark.parametrize('flag',[None,0,'true',1,False])
def test_ambiguous_context_never_loosens_loading_gate(flag):
    assert observation_phase('right_grasp',flag)=='right_grasp'


@pytest.mark.parametrize('phase',['right_release_on_table','left_close_empty_check',
                                 'right_open_empty','machine_stop','unknown'])
def test_all_other_phases_keep_original_identity(phase):
    assert observation_phase(phase,True)==phase


def fixture():
    root=Path(__file__).resolve().parent
    return json.loads((root/'retrieval_grasp_regression_fixture.json').read_text())['fault']['payload']['machine']


def test_actual_r223_fault_routes_to_existing_authorized_gate_only():
    machine=fixture()
    assert .02<abs(machine['rotor_rpm'])<.1
    assert not machine_ready_for_cycle(machine,'right_grasp',0)
    assert machine_ready_for_cycle(machine,observation_phase('right_grasp',True),0)
    assert not machine_ready_for_cycle(machine,observation_phase('right_grasp',False),0)


@pytest.mark.parametrize('changes',[{'rotor_rpm':.10001},{'rotor_enabled':True},
    {'brake_active':False},{'motor_torque_nm':2.001},{'lid_rad':0.},
    {'open_lid_demo':True},{'rotor_rad_s':float('nan')}])
def test_no_existing_retrieval_limit_or_interlock_relaxed(changes):
    machine=deepcopy(fixture());machine.update(changes)
    assert not machine_ready_for_cycle(machine,observation_phase('right_grasp',True),0)


def test_goal_parameters_stay_original_while_only_observer_phase_is_translated():
    source=inspect.getsource(LoadingCycle.gripper)
    assert 'self.phase = self.gripper_observation_phase(phase)' in source
    assert 'self.gripper_trajectory_points(side, value, phase, effort)' in source
    assert "effort = 16.0 if phase == side + '_grasp' else 0.0" in source
    assert source.index('gripper_observation_phase')<source.index('self.safety()')

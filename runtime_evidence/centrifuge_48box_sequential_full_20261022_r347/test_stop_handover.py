from types import SimpleNamespace

import pytest

from execute_dual import DualCycle
from handover_policy import FREE_PHASES, require_snapshot_drift, require_stop_pre, transport_rate
from stop_handover_cycle import StopHandoverCycle


@pytest.mark.parametrize('phase', sorted(FREE_PHASES))
def test_four_free_phases_require_both_actual_holders(phase):
    assert transport_rate(phase, .25, {'left': {}, 'right': {}}) == .4
    with pytest.raises(ValueError):
        transport_rate(phase, .25, {'left': {}})


@pytest.mark.parametrize('phase,rate', [('paired_load_descend', .13), ('paired_retrieval_descend', .13),
                                     ('paired_retrieval_pilot_lift', .03), ('paired_retrieval_setdown', .2),
                                     ('paired_retrieval_setdown_fine', .03),
                                     ('paired_load_forward_inside_basket_raise', .08)])
def test_all_contact_pilot_and_setdown_rates_remain_original(phase, rate):
    assert transport_rate(phase, rate, {}) == rate


def test_snapshot_and_original_pre_gates_cannot_be_relaxed():
    arm = dict(joints=['right_elbow_joint'], pre=[.5])
    require_stop_pre({'right_elbow_joint': .5001}, arm)
    with pytest.raises(ValueError):
        require_stop_pre({'right_elbow_joint': .506}, arm)
    assert require_snapshot_drift({'right_elbow_joint': .5001}, {'right_elbow_joint': .5}) < .0003
    with pytest.raises(ValueError):
        require_snapshot_drift({'right_elbow_joint': .5004}, {'right_elbow_joint': .5})


def mock_cycle():
    cycle = StopHandoverCycle.__new__(StopHandoverCycle)
    cycle.holders = SimpleNamespace(rows={})
    cycle.positions = {'right_elbow_joint': .5}
    cycle.buttons = dict(arms=dict(right=dict(joints=['right_elbow_joint'], pre=[.5])))
    cycle.values = dict(contact_status=dict(buttons=dict(rotor=dict(armed=True))))
    cycle.record = lambda kind, payload: None
    return cycle


def test_stop_keeps_original_actual_contact_withdrawal_but_skips_home(monkeypatch):
    observed = []
    def physical_press(self, side, phase, return_home=True, scheduled=None):
        observed.append((side, phase, return_home, scheduled))
        return {'button': 'rotor', 'source': 'gazebo_physics_contact'}, scheduled
    monkeypatch.setattr(DualCycle, 'press', physical_press)
    cycle = mock_cycle()
    event, stamp = cycle.press('right', 'machine_stop', scheduled=55.)
    assert observed == [('right', 'machine_stop', False, 55.)]
    assert event['source'] == 'gazebo_physics_contact' and stamp == 55.


def test_left_final_open_requires_actual_audit_before_original_press(monkeypatch):
    observed = []
    def physical_press(self, side, phase, return_home=True, scheduled=None):
        observed.append('press')
        return {}, 0.
    monkeypatch.setattr(DualCycle, 'press', physical_press)
    cycle = mock_cycle()
    cycle.check_actual_stop_handover = lambda: observed.append('actual_dense_audit')
    cycle.press('left', 'machine_final_open')
    assert observed == ['actual_dense_audit', 'press']


def test_stop_interception_refuses_missing_schedule_or_wrong_hand():
    cycle = mock_cycle()
    with pytest.raises(ValueError):
        cycle.press('right', 'machine_stop')
    with pytest.raises(ValueError):
        cycle.press('left', 'machine_stop', scheduled=55.)


def test_actor_inherits_the_complete_load_machine_retrieve_program():
    assert issubclass(StopHandoverCycle, DualCycle)
    assert StopHandoverCycle.run_complete is DualCycle.run_complete
    assert StopHandoverCycle.retrieve is DualCycle.retrieve
    assert StopHandoverCycle.machine_cycle is DualCycle.machine_cycle


@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_current_joint_cannot_hide_behind_another_finite_joint(invalid):
    with pytest.raises(ValueError):
        require_snapshot_drift({'left_elbow_joint': .5, 'right_elbow_joint': invalid},
                               {'left_elbow_joint': .5, 'right_elbow_joint': .5})


def test_nonfinite_original_pre_target_is_refused():
    with pytest.raises(ValueError):
        require_stop_pre({'right_elbow_joint': .5}, dict(joints=['right_elbow_joint'], pre=[float('nan')]))


def test_actual_speed_fork_explicitly_changes_times_not_geometry(monkeypatch):
    observed = []
    def physical_move(self, samples, phase, rate):
        observed.append((samples, phase, rate))
    monkeypatch.setattr(DualCycle, 'move', physical_move)
    cycle = mock_cycle()
    cycle.holders.rows = {'left': {}, 'right': {}}
    records = []
    cycle.record = lambda kind, payload: records.append((kind, payload))
    samples = [{'unchanged': 1}]
    cycle.move(samples, 'paired_retrieval_transfer', .25)
    assert observed == [(samples, 'paired_retrieval_transfer', .4)]
    assert records[0][1]['only_motion_times_changed'] is True
    assert records[0][1]['original_points_groups_floor_and_physical_guards_preserved'] is True
    assert records[0][1]['physical_speed_pass_not_preclaimed'] is True

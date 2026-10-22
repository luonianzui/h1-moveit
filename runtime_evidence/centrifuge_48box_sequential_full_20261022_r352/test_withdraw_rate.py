import pytest

import table_alignment_cycle as runtime

@pytest.mark.parametrize('phase', ['machine_preparation_open_withdraw', 'machine_close_withdraw', 'machine_stop_withdraw', 'machine_reopen_withdraw'])
def test_only_empty_withdraw_rate_capped(monkeypatch, phase):
    monkeypatch.setattr(runtime.ParentSpeedPriorityCycle, 'button_noncontact_rate', lambda self: .8)
    cycle = runtime.SpeedPriorityCycle.__new__(runtime.SpeedPriorityCycle)
    cycle.phase = phase
    assert cycle.button_noncontact_rate() == .6

@pytest.mark.parametrize('phase', ['machine_preparation_open_approach', 'machine_close_press', 'seq_load_transport', 'seq_retrieval_single_pilot'])
def test_other_phase_rate_unchanged(monkeypatch, phase):
    monkeypatch.setattr(runtime.ParentSpeedPriorityCycle, 'button_noncontact_rate', lambda self: .8)
    cycle = runtime.SpeedPriorityCycle.__new__(runtime.SpeedPriorityCycle)
    cycle.phase = phase
    assert cycle.button_noncontact_rate() == .8

def test_parent_rejection_is_preserved(monkeypatch):
    def reject(self):
        raise ValueError('actual holder not empty')
    monkeypatch.setattr(runtime.ParentSpeedPriorityCycle, 'button_noncontact_rate', reject)
    cycle = runtime.SpeedPriorityCycle.__new__(runtime.SpeedPriorityCycle)
    cycle.phase = 'machine_open_withdraw'
    with pytest.raises(ValueError, match='holder'):
        cycle.button_noncontact_rate()

def test_contact_timing_does_not_call_noncontact_override():
    from pathlib import Path
    root = Path(__file__).resolve().parent
    assert 'self.button_noncontact_rate() if fixed_interval is None else .8' in (root / 'sequential_speed_cycle.py').read_text()
    from speed_policy import PROFILE
    assert PROFILE['button_tracking_rad'] == .15
    assert PROFILE['button_contact_interval_sim_s'] == .02
    assert PROFILE['steady_spin_sim_s'] == 30.

from pathlib import Path


EVIDENCE = Path(__file__).resolve().parent


def test_actual_paired_runtime_uses_qualified_interface_and_original_rates():
    source = (EVIDENCE / 'execute_dual.py').read_text()
    assert 'coarse, fine, split = split_paired_setdown(stage)' in source
    assert 'braking_setdown_segments(stage' not in source
    assert "self.move(coarse, 'paired_retrieval_setdown', self.coarse_setdown_rate())" in source
    assert "self.move(fine, 'paired_retrieval_setdown_fine', self.fine_setdown_rate())" in source
    assert 'fine_rate_rad_s=self.fine_setdown_rate()' in source
    from execute_dual import DualCycle
    assert DualCycle.fine_setdown_rate(DualCycle.__new__(DualCycle)) == .03
    assert DualCycle.coarse_setdown_rate(DualCycle.__new__(DualCycle)) == .20
    assert 'unchanged_path=True' in source


def test_planner_declares_real_same_300mm_path_without_removing_seed():
    source = (EVIDENCE / 'dual_worker.py').read_text()
    assert "planner.checked_stage(table, down, seed, 'setdown', count=121, table_end=True)" in source
    assert "stage['setdown_contract'] = dict(cartesian_samples=121, extra_initial_seed=True" in source
    assert 'float(table[hand][2, 3] - down[hand][2, 3])' in source
    assert 'samples = [dict(seed)]' in source


def test_single_hand_contract_and_prior_failure_protection_are_preserved():
    legacy = (EVIDENCE / 'setdown_braking.py').read_text()
    assert 'if len(samples) != 121:' in legacy
    assert "'centrifuge_48box_startup_window_20261013_r247'" in (EVIDENCE / 'verify_dual_protection.py').read_text()

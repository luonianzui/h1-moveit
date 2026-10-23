from pathlib import Path


def test_fast_full_cycle_requires_actual_dual_unloading_and_original_phase_certificate():
    source = Path(__file__).with_name('seal_identity.py').read_text()
    assert 'centrifuge_48box_pilot_braking_20261010_r208' in source
    assert "local / 'unloading_audit.json'" in source
    assert "local / 'pilot_contract_audit.json'" in source
    assert "local / 'SHA256SUMS'" in source
    assert "full / 'cycle_audit.json'" not in source

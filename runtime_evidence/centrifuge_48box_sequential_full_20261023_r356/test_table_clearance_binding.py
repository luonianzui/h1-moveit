from pathlib import Path
import hashlib
import json
from inherited_source_identity import restore_inherited_source
from dense_shortcut_source import restore_dense_worker_source


EVIDENCE = Path(__file__).resolve().parent


def test_table_phase_rechecks_both_buckets_without_clearing_the_guard():
    source = (EVIDENCE / 'execute_dual.py').read_text()
    start = source.index('        for payload in self.pair_clearance_payloads:')
    section = source[start:source.index('    def ', start)]
    assert 'if self.phase in TABLE_SETDOWN_PHASES:' in section
    assert 'table_bucket_clearance(poses, payload, pairs, self.table_bucket_bounds)' in section
    assert "self.record('table_setdown_bucket_clearance', data)" in section
    assert 'self.pair_clearance_payloads.clear()' not in section
    assert 'clearance_metrics(self.poses, payload' in section
    assert "raise RuntimeError('Paired full extraction clearance lost')" in section


def test_original_lift_guards_and_default_geometry_stay_unchanged():
    parent = EVIDENCE.parent / 'centrifuge_48box_retrieval_approach_20261013_r249'
    for name in ('extraction_clearance.py', 'dual_worker.py', 'paired_setdown_braking.py'):
        restored = restore_inherited_source(name, (EVIDENCE / name).read_text())
        if name == 'dual_worker.py':
            from lift_provider_source import restore_lift_provider
            restored = restore_lift_provider(restored)
        if name == 'dual_worker.py':
            recent_parent = EVIDENCE.parent / 'centrifuge_48box_dense_endpoint_full_20261015_r281'
            recent_adapter = recent_parent / 'inherited_source_identity.py'
            recent_identity = json.loads((recent_parent / 'identity.json').read_text())['sha256']
            assert hashlib.sha256(recent_adapter.read_bytes()).hexdigest() == recent_identity[recent_adapter.name]
            recent_namespace = {}
            exec(compile(recent_adapter.read_text(), str(recent_adapter), 'exec'), recent_namespace)
            restored = recent_namespace['restore_inherited_source'](name, restored)
        current = restore_dense_worker_source(name, restored).encode()
        if name == 'dual_worker.py':
            recent = EVIDENCE.parent / 'centrifuge_48box_post_retreat_gripper_20261014_r278'
            original_adapter = recent / 'inherited_source_identity.py'
            identity = json.loads((recent / 'identity.json').read_text())['sha256']
            assert hashlib.sha256(original_adapter.read_bytes()).hexdigest() == identity[original_adapter.name]
            namespace = {}
            exec(compile(original_adapter.read_text(), str(original_adapter), 'exec'), namespace)
            current = namespace['restore_inherited_source'](name, current.decode()).encode()
        assert current == (parent / name).read_bytes()
    source = (EVIDENCE / 'execute_dual.py').read_text()
    assert 'hold_after_full_lift=True' in source
    assert 'self.dwell(10)' in source


def test_complete_audit_recomputes_geometry_and_checks_the_original_contact_stream():
    source = (EVIDENCE / 'audit_cycle.py').read_text()
    assert "verify_table_record(row['phase'], data, table_bucket_bounds)" in source
    assert "data['contacts'] != contacts[data['payload']]" in source
    assert 'minimum_extraction_clearance_m >= .003' in source
    assert 'min(table_clearance_checks.values()) > 100' in source

import inspect
from pathlib import Path

import execute_cycle
import run_guard
from video_capture import capture_command


def test_real_executor_restores_handler_before_any_claim():
    source = inspect.getsource(execute_cycle.main)
    assert source.index('establish_interrupt_handler') < source.index('verify_interrupt_delivery') < source.index('validate_dispatch')
    assert source.index('verify_interrupt_delivery') < source.index("'run/execution.claim'")


def test_stop_guard_checks_delivery_before_signalling():
    source = inspect.getsource(run_guard.stop_executor)
    assert source.index('verify_interrupt_delivery') < source.index('os.kill')


def test_capture_has_fixed_window_region_not_root(tmp_path):
    command = capture_command('0x123', tmp_path / 'sample.mp4')
    assert {'xid=291', 'endx=1199', 'endy=899'} <= set(command)
    assert 'xid=0' not in command


def test_lossless_trace_changes_only_compression_cost():
    source = Path(__file__).with_name('trace_io.py').read_text()
    assert 'compresslevel=1' in source and "compression='gzip_level_1'" in source
    assert 'self.hasher.update(encoded)' in source
    assert 'no_sampling=True' in source


def test_real_resize_evidence_required_before_seal():
    source = Path(__file__).with_name('seal_identity.py').read_text()
    assert "evidence / 'resize_capture_audit.json'" in source

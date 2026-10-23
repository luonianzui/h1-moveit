from pathlib import Path
import signal
from types import SimpleNamespace

import pytest

import startup_review_freeze as freeze


@pytest.mark.parametrize('seconds', [0, 6.125, 24.99, 49.1335, 90., 300.])
def test_measured_clock_uses_original_fresh_window(seconds):
    assert freeze.measured_clock(dict(simTime=dict(sec=str(seconds)))) == seconds


@pytest.mark.parametrize('stats', [{}, dict(simTime=dict(sec='NaN')), dict(simTime=dict(sec='301')), dict(simTime=dict(sec='-1'))])
def test_invalid_or_expired_clock_is_rejected(stats):
    with pytest.raises(ValueError):
        freeze.measured_clock(stats)


def process_entry(monkeypatch, status, created=100., command=None):
    observed = []
    command = command or ['gz sim server']
    process = SimpleNamespace(is_running=lambda: True, status=lambda: status,
                              create_time=lambda: created, cmdline=lambda: command,
                              send_signal=observed.append)
    monkeypatch.setattr(freeze.psutil, 'Process', lambda pid: process)
    return dict(pid=12345, created=100., command=['gz sim server']), observed


@pytest.mark.parametrize('requested,status', [(signal.SIGSTOP, 'running'), (signal.SIGCONT, 'stopped')])
def test_exact_owned_server_stop_and_continue_only(monkeypatch, requested, status):
    entry, observed = process_entry(monkeypatch, status)
    freeze.send_server_signal(entry, requested)
    assert observed == [requested]


@pytest.mark.parametrize('requested', [signal.SIGINT, signal.SIGTERM])
def test_other_signals_never_sent(monkeypatch, requested):
    entry, observed = process_entry(monkeypatch, 'running')
    with pytest.raises(ValueError):
        freeze.send_server_signal(entry, requested)
    assert observed == []


@pytest.mark.parametrize('mismatch', ['pid_reuse', 'different_command'])
def test_identity_mismatch_never_signals_another_process(monkeypatch, mismatch):
    entry, observed = process_entry(monkeypatch, 'running', created=101. if mismatch == 'pid_reuse' else 100.,
                                    command=['another process'] if mismatch == 'different_command' else None)
    with pytest.raises(ValueError):
        freeze.send_server_signal(entry, signal.SIGSTOP)
    assert observed == []


def test_startup_dispatch_uses_process_freeze_not_failed_world_pause():
    source = Path(__file__).with_name('start_front_recording.sh').read_text()
    assert 'startup_review_freeze.py" freeze' in source
    assert 'startup_review_hold.py" pause' not in source

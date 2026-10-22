import json
from types import SimpleNamespace

import pytest

import startup_review_hold as hold


@pytest.mark.parametrize('seconds', [0, 6.125, 24.99, 46., 300.])
def test_actual_paused_stamp_still_uses_original_fresh_window(seconds):
    whole = int(seconds)
    assert hold.validate_paused(dict(paused=True, simTime=dict(sec=str(whole), nsec=round((seconds - whole) * 1e9)))) == seconds


@pytest.mark.parametrize('stats', [dict(paused=False, simTime=dict(sec='6')),
                                  dict(paused=True, simTime=dict(sec='301')),
                                  dict(paused=True, simTime=dict(sec='NaN')),
                                  dict(paused=True), dict(paused=True, simTime=dict(sec='-1'))])
def test_unpaused_or_aged_world_is_not_accepted(stats):
    with pytest.raises(ValueError):
        hold.validate_paused(stats)


@pytest.mark.parametrize('filename', ['execution.claim', 'process_identities.json', 'result.json', 'review_resume.claim'])
def test_claim_result_or_prior_resume_prevents_startup_pause_reuse(tmp_path, filename):
    (tmp_path / 'run').mkdir()
    (tmp_path / 'run' / filename).write_text('{}')
    with pytest.raises(ValueError):
        hold.require_unclaimed(tmp_path)


@pytest.mark.parametrize('paused', [True, False])
def test_exact_pause_only_world_control_has_no_reset_step_or_pose_fields(monkeypatch, paused):
    observed = []
    def run(command, **kwargs):
        observed.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout='data: true\n', stderr='')
    monkeypatch.setattr(hold.subprocess, 'run', run)
    hold.world_control(paused)
    command, options = observed[0]
    assert command[-1] == ('pause: true' if paused else 'pause: false')
    assert command[command.index('--req') + 1:] == [command[-1]]
    assert options['timeout'] == 8 and '--timeout' in command


def test_service_nonacknowledgement_is_not_a_valid_pause(monkeypatch):
    monkeypatch.setattr(hold.subprocess, 'run', lambda *args, **kwargs: SimpleNamespace(returncode=0, stdout='data: false', stderr=''))
    with pytest.raises(RuntimeError):
        hold.world_control(True)

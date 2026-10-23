import inspect
import json

import run_guard


def test_finished_task_never_signals_even_when_recorder_already_finalizing(monkeypatch, tmp_path):
    directory = tmp_path / 'run'
    directory.mkdir()
    (directory / 'video_pid.json').write_text(json.dumps(dict(pid=99999999, created=0, command=[])))
    (directory / 'full_cycle.mp4').touch()
    (directory / 'result.json').write_text(json.dumps(dict(status='PASS')))
    monkeypatch.setattr(run_guard, 'EVIDENCE', tmp_path)
    monkeypatch.setattr(run_guard, 'stop_executor', lambda target: (_ for _ in ()).throw(AssertionError(target)))
    run_guard.main()
    result = json.loads((directory / 'run_guard.json').read_text())
    assert result['status'] == 'PASS' and result['no_stop_sent']


def test_only_exact_executor_identity_is_siginted_never_gazebo_or_payload():
    source = inspect.getsource(run_guard.stop_executor)
    assert "process.create_time() == target['create_time']" in source
    assert "process.cmdline()[-1] == str(EVIDENCE / 'execute_cycle.py')" in source
    assert 'signal.SIGINT' in source and 'SIGKILL' not in source
    assert 'publisher' not in source and 'gz service' not in source


def test_missing_video_and_20_second_stall_are_supervised():
    source = inspect.getsource(run_guard.main)
    assert 'recorder_missing_or_identity_changed' in source
    assert 'video_not_growing_20_wall_seconds' in source
    assert 'stop_executor(target)' in source

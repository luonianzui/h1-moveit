import inspect
import json

import disk_watch


def test_completed_result_causes_no_signal_and_preserves_minimum_disk_evidence(monkeypatch, tmp_path):
    (tmp_path / 'run').mkdir()
    (tmp_path / 'run/result.json').write_text(json.dumps(dict(status='PASS')))
    monkeypatch.setattr(disk_watch, 'EVIDENCE', tmp_path)
    monkeypatch.setattr(disk_watch.os, 'kill', lambda *arguments: (_ for _ in ()).throw(AssertionError(arguments)))
    disk_watch.main()
    result = json.loads((tmp_path / 'run/disk_watch.json').read_text())
    assert result['status'] == 'PASS' and result['no_stop_sent']
    assert result['minimum_free_bytes'] > 0


def test_low_disk_can_only_sigint_exact_pid_ctime_and_script_not_gazebo_reset():
    source = inspect.getsource(disk_watch.main)
    assert "process.create_time() == target['create_time']" in source
    assert "process.cmdline()[-1] == str(EVIDENCE / 'execute_cycle.py')" in source
    assert 'signal.SIGINT' in source and 'signal.SIGKILL' not in source
    assert 'publisher' not in source and 'gz service' not in source

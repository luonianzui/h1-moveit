import json
from types import SimpleNamespace

import pytest
import launch_run


@pytest.mark.parametrize('seconds,code', [(120., 0), (181., 0), (120., 1)])
def test_monotonic_executor_measurement_includes_observed_exit_not_simulation(tmp_path, monkeypatch, seconds, code):
    (tmp_path / 'run').mkdir()
    monkeypatch.setattr(launch_run, 'EVIDENCE', tmp_path)
    captures = []
    monkeypatch.setattr(launch_run, 'capture_owned_telemetry', lambda root: captures.append(root))
    starts = [100, 200, 300, 400]
    ends = [int(seconds * 1e9) + 100, int(seconds * 1e9) + 200,
            int(seconds * 1e9) + 300, int(seconds * 1e9) + 400]
    clock = iter(starts + ends)
    monkeypatch.setattr(launch_run.time, 'monotonic_ns', lambda: next(clock))
    pids = iter([101, 102, 103, 104])
    def spawn(command, **options):
        pid = next(pids)
        return SimpleNamespace(pid=pid, wait=lambda: code if pid == 101 else 0)
    monkeypatch.setattr(launch_run.subprocess, 'Popen', spawn)
    monkeypatch.setattr(launch_run.psutil, 'Process', lambda pid: SimpleNamespace(create_time=lambda: 42.))
    if code:
        with pytest.raises(AssertionError):
            launch_run.main()
    else:
        launch_run.main()
    result = json.loads((tmp_path / 'run/executor_wall_clock.json').read_text())
    assert captures == [tmp_path]
    assert result['elapsed_wall_s'] == seconds
    assert result['within_180_wall_s'] is (seconds <= 180.)
    assert result['execution_returncode'] == code
    assert result['not_independent_physical_certificate']

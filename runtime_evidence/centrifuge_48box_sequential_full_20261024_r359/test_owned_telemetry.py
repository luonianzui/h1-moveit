import ctypes
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import psutil
import pytest
import run_guard

from owned_telemetry import owned_telemetry_fault

ROOT = Path(__file__).resolve().parent


@pytest.mark.parametrize('changed', ['none', 'created', 'command', 'zombie', 'missing'])
def test_exact_owned_identity_not_any_process(changed):
    record = dict(processes=dict(relay=dict(pid=71, created=2., command=['owned', 'relay'])))
    def process_factory(pid):
        assert pid == 71
        if changed == 'missing':
            raise psutil.NoSuchProcess(pid)
        return SimpleNamespace(is_running=lambda: True,
            status=lambda: psutil.STATUS_ZOMBIE if changed == 'zombie' else psutil.STATUS_RUNNING,
            create_time=lambda: 3. if changed == 'created' else 2.,
            cmdline=lambda: ['unrelated'] if changed == 'command' else ['owned', 'relay'])
    assert (owned_telemetry_fault(record, process_factory) is None) == (changed == 'none')


@pytest.mark.parametrize('capacity', ['1000', '20000'])
def test_installed_transport_reads_finite_capacity(capacity):
    code = ("import ctypes,json; library=ctypes.CDLL('/opt/ros/jazzy/opt/gz_transport_vendor/lib/libgz-transport13.so'); "
        "print(json.dumps([getattr(library,name)() for name in "
        "('_ZN2gz9transport3v136rcvHwmEv','_ZN2gz9transport3v136sndHwmEv')]))")
    result = subprocess.run([sys.executable, '-c', code], env=dict(os.environ,
        GZ_TRANSPORT_RCVHWM=capacity, GZ_TRANSPORT_SNDHWM=capacity),
        capture_output=True, text=True, check=True)
    assert json.loads(result.stdout) == [int(capacity), int(capacity)]


def test_relay_and_physics_contract_not_relaxed():
    parent = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r349'
    assert (ROOT / 'inputs/robot_joint_state_relay_05ms.py').read_bytes() == (parent / 'inputs/robot_joint_state_relay_05ms.py').read_bytes()
    assert (ROOT / 'speed_policy.py').read_bytes() == (parent / 'speed_policy.py').read_bytes()
    assert "GZ_TRANSPORT_RCVHWM='20000', GZ_TRANSPORT_SNDHWM='20000'" in (ROOT / 'launch_scene.py').read_text()
    assert "reason = owned_telemetry_fault(telemetry)" in (ROOT / 'run_guard.py').read_text()


def test_owned_relay_loss_stops_only_exact_executor_without_waiting_for_sim_clock(monkeypatch, tmp_path):
    directory = tmp_path / 'run'
    directory.mkdir()
    (directory / 'owned_telemetry.json').write_text(json.dumps(dict(processes={})))
    (directory / 'video_pid.json').write_text('{}')
    (directory / 'full_cycle.mp4').write_bytes(b'video')
    target = dict(pid=71, create_time=2., cmdline=[str(tmp_path / 'execute_cycle.py')])
    stopped = []
    monkeypatch.setattr(run_guard, 'EVIDENCE', tmp_path)
    monkeypatch.setattr(run_guard.psutil, 'process_iter', lambda fields: [SimpleNamespace(info=target)])
    monkeypatch.setattr(run_guard, 'recorder_alive', lambda video: True)
    monkeypatch.setattr(run_guard, 'owned_telemetry_fault', lambda record: 'owned_relay_unavailable')
    monkeypatch.setattr(run_guard, 'stop_executor', lambda process: stopped.append(process))
    monkeypatch.setattr(run_guard.shutil, 'disk_usage', lambda path: SimpleNamespace(free=4 * 1024**3))
    monkeypatch.setattr(run_guard.time, 'sleep', lambda delay: (_ for _ in ()).throw(AssertionError('must stop immediately')))
    run_guard.main()
    result = json.loads((directory / 'run_guard.json').read_text())
    assert stopped == [target]
    assert result['status'] == 'STOP_AND_HOLD' and result['reason'] == 'owned_relay_unavailable'
    assert result['signal'] == 'SIGINT'
